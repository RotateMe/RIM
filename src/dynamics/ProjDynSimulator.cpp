/*
********************************************************************************
MIT License

Copyright(c) 2019 Christopher Brandt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
********************************************************************************
*/
#include <iostream>
#include <omp.h>
#include <limits>

#include "dynamics/ProjDynSimulator.h"
#include "dynamics/ProjDynConstraints.h"
#include "dynamics/ProjDynUtil.h"
#include "dynamics/ProjDynMeshSampler.h"
#include "dynamics/StopWatch.h"
#include "dynamics/ProjDynTetGen.h"
#include "dynamics/ProjDynRHSInterpol.h"

#include "dynamics/ProjDynPICFLIP.h"

#include <imgui.h>

using namespace PD;
using namespace SPH;

ProjDynSimulator::GUI::GUI(ProjDynSimulator * pdsim)
{
	pdsimptr = pdsim;
	visible = true;
	displayname = "Simulation runtime options";
}

ProjDynSimulator::GUI::~GUI() {}


void ProjDynSimulator::GUI::draw(GLFWwindow* window, int order)
{
	if (!pdsimptr) return;

	ImVec2 outerSize = ImGui::GetItemRectSize();

	if (ImGui::Button("Reset Positions")) {
		pdsimptr->resetPositions();
	}

	ImGui::DragInt("#local-global its.", &pdsimptr->m_numIterations, 1, 1, 50);
    ImGui::DragInt("#Jacobi its.", &pdsimptr->m_jacobiIts, 10, 1, 1000);
    if (ImGui::DragFloat("Material stiffness", &pdsimptr->m_stiffnessFactor, 0.05, 0.01, 5)) {
		pdsimptr->setStiffnessFactor(pdsimptr->m_stiffnessFactor);
	}
	ImGui::DragFloat("Solid:Fluid density", &pdsimptr->m_fluidDensity, 0.01, 0.25, 0.85);
	ImGui::DragFloat("Fluid calmness", &pdsimptr->m_flipness, 0.005, 0.01, 0.99);
    ImGui::Checkbox("Skip fluid step", &pdsimptr->m_skipGridStep);

}


ProjDynSimulator::ProjDynSimulator
(PDTriangles& triangles, PDPositions& initialPositions, PDPositions& initialVelocities,
	PDScalar timeStep, int numSamplesPosSubspace, PDScalar baseFunctionRadius,
	int interpolBaseSize,
	PDScalar rhsInterpolWeightRadius,
	int numConstraintSamples,
	PDScalar massPerUnitArea,
	PDScalar dampingAlpha,
	bool makeTets,
	std::string meshURL,
	PDScalar rhsRegularizationWeight,
	PDScalar yTrans,
	PDScalar tetMeshQ,
	PDScalar gripWeight,
	int initialNumIterations) :

	m_gripWeight(gripWeight),

	m_flatBending(false),
	m_rhsRegularizationWeight(rhsRegularizationWeight),

	m_parallelVUpdate(false),
	m_parallelVUpdateBSize(PROJ_DYN_VPOS_BLOCK_SIZE),

	m_rhsInterpolBaseSize(interpolBaseSize),

	m_numSamplesPosSubspace(numSamplesPosSubspace),
	m_numConstraintSamples(numConstraintSamples),

	m_meshURL(meshURL),

	m_numTets(0),
	m_hasTetrahedrons(false),

	m_rayleighDampingAlpha(dampingAlpha),

	m_usingSubspaces(numSamplesPosSubspace > 0),
	m_rhsInterpolation(numConstraintSamples > 0),
	m_baseFunctionRadius(baseFunctionRadius),

	m_localStepStopWatch(10000, 10000),
	m_globalStepStopWatch(10000, 10000),
	m_totalStopWatch(10000, 10000),
	m_localStepOnlyProjectStopWatch(10000, 10000),
	m_localStepRestStopWatch(10000, 10000),
	m_surroundingBlockStopWatch(10000, 10000),
	m_updatingVPosStopWatch(10000, 10000),
	m_fullUpdateStopWatch(10000, 10000),
	m_picflipStopWatch(10000, 10000),
	m_particleTransferStopWatch(10000, 10000),
	m_precomputationStopWatch(10, 10),
	m_elasticityStopWatch(10000, 10000),

	m_positionsUpdated(false),

	m_particleBufferID(NULL),

	m_numIterations(initialNumIterations),
	m_floorCoordinate(-1),
	m_floorHeight(0),
	m_ceilingHeight(1000),
	m_recomputeFactorization(false),
	m_floorCollisionWeight(0),

	m_constraintSamplesChanged(true),
	m_numRefactorizations(0),
	m_constraintSummationStopWatch(10000, 10000),
	m_momentumStopWatch(10000, 10000),
	m_multiplicationForPosUpdate(10000, 10000),
	m_sortingForPosUpdate(10000, 10000),
	m_timeStep(timeStep),

	m_tetrahedrons(0, 4),
	m_frameCount(0),

	m_gravity(0),

	m_usedVertexMap(nullptr),

	m_rhsInterpolReusableWeights(0, 0),
	m_rhsInterpolWeightRadiusMultiplier(rhsInterpolWeightRadius),

	m_useSparseMatricesForSubspace(PROJ_DYN_SPARSIFY),

	m_picflipsimulator(nullptr)

#ifdef PROJ_DYN_USE_CUBLAS_REST
	,
	m_usedVertexUpdater(nullptr),
	m_rhsEvaluator(nullptr)
#endif
{
	// This is the most important line of code in this project, don't remove it.
	// EIGEN_DONT_PARALLELIZE just doesn't work.
	Eigen::setNbThreads(1);

	once = false;

	// Construct position and triangle matrices
	m_numOuterVertices = initialPositions.rows();
	m_positions = initialPositions;
	for (unsigned int v = 0; v < m_positions.rows(); v++) m_positions(v, 1) += yTrans;
	m_numTriangles = triangles.rows();
	m_triangles = triangles;

	// Create tet mesh out of triangle mesh, if desired
	// The new vertices and triangles are appended to the old matrices
	// m_numVertices will be the number of all vertices, m_numOuterVertices
	// is the number of only the original vertices
	if (makeTets) {
		TetGen test;
		PDPositions positionsNew;
		if (test.tetrahedralizeMesh(m_positions, m_triangles, positionsNew, m_tetrahedrons, tetMeshQ)) {
			m_positions = positionsNew;
			m_hasTetrahedrons = true;
			m_numTets = m_tetrahedrons.rows();
		}
		else {
			std::cout << "Error: Could not generate tets! Mesh has self-intersections or other problems..." << std::endl;
		}
	}

	m_numVertices = m_positions.rows();

	// Move center of shape to origin and store the norm
	PDPositions origPos = m_positions;
	for (int d = 0; d < 3; d++) {
		PDScalar avg = origPos.col(d).sum() / origPos.rows();
		for (int v = 0; v < m_numVertices; v++)
			origPos(v, d) -= avg;
	}
	m_initPosNorm = origPos.norm();

	std::cout << "Initiating vertex sampler (i.e. distance fields) ..." << std::endl;

	m_sampler.init(m_positions, m_triangles, m_tetrahedrons);

	std::cout << "Initiatlizing external force and velocity vectors ..." << std::endl;

	m_frictionCoeff = 0;

	m_fExt.resize(m_numVertices, 3);
	m_fExt.setZero();

	m_fGravity.resize(m_numVertices, 3);
	m_fGravity.setZero();

	m_velocities = initialVelocities;
	m_velocities.conservativeResize(m_numVertices, 3);

	for (int v = m_numOuterVertices; v < m_numVertices; v++) {
		m_velocities.row(v).setConstant(0);
	}

	if (!m_hasTetrahedrons) {
		m_vertexMasses = vertexMasses(m_triangles, m_positions);
	}
	else {
		m_vertexMasses = vertexMasses(m_tetrahedrons, m_positions);
	}

	std::cout << "Average vertex mass: " << m_vertexMasses.sum() / (float)m_numVertices << std::endl;

	m_totalMass = m_vertexMasses.sum();

	// Normalize vertex masses to integrate to 1 for numerical reasons
	m_normalization = (1. / m_totalMass);
	m_vertexMasses *= m_normalization * massPerUnitArea;

	std::vector<Eigen::Triplet<PDScalar>> massEntries;
	massEntries.reserve(m_numVertices);
	for (int v = 0; v < m_numVertices; v++) {
		massEntries.push_back(Eigen::Triplet<PDScalar>(v, v, m_vertexMasses(v)));
	}
	m_massMatrix = PDSparseMatrix(m_numVertices, m_numVertices);
	m_massMatrix.setFromTriplets(massEntries.begin(), massEntries.end());

	m_vertexStars = makeVertexStars(m_numVertices, m_numTriangles, m_triangles);

	int vStarSize = 0;
	for (int v = 0; v < m_numVertices; v++) {
		vStarSize += m_vertexStars.at(v).size();
	}
	std::cout << "Average vertex star size: " << ((float)vStarSize / (float)m_numVertices) << std::endl;

	for (unsigned int i = 0; i < m_numVertices; i++) {
		m_allVerts.push_back(i);
	}

	// Create or load position subspace basis functions
	if (m_usingSubspaces) {
		bool loadSuccess = false;
		if (!loadSuccess) {
			std::cout << "Creating subspaces..." << std::endl;
			createPositionSubspace(m_numSamplesPosSubspace, true);
		}
		finalizeBaseFunctions();
	}

	m_isSetup = false;

	m_rhs.setZero(m_numVertices, 3);

	omp_set_num_threads(PROJ_DYN_NUM_THREADS);

#ifndef EIGEN_DONT_PARALLELIZE
	Eigen::setNbThreads(1);
	Eigen::initParallel();
#endif

}

void ProjDynSimulator::finalizeBaseFunctions() {
	m_baseFunctionsTransposed = m_baseFunctions.transpose();
	m_baseFunctionsSquared = m_baseFunctionsTransposed * m_baseFunctions;
}

int ProjDynSimulator::getNumVertices() {
	return m_numVertices;
}

PDPositions& ProjDynSimulator::getPositions()
{
	if (m_usingSubspaces && !m_positionsUpdated) {
		PROJ_DYN_PARALLEL_FOR
			for (int d = 0; d < 3; d++) {
				m_positions.col(d) = m_baseFunctionsSparse * m_positionsSubspace.col(d);
			}
	}
	return m_positions;
}

PDPositions& ProjDynSimulator::getVelocities()
{
	return m_velocities;
}

void PD::ProjDynSimulator::recomputeWeightedForces() {
	m_fExtWeighted.resize(m_numVertices, 3);
	m_fGravWeighted.resize(m_numVertices, 3);
	PROJ_DYN_PARALLEL_FOR
		for (int v = 0; v < m_numVertices; v++) {
			for (int d = 0; d < 3; d++) {
				m_fExtWeighted(v, d) = m_fExt(v, d) * (1. / m_vertexMasses(v));
				m_fGravWeighted(v, d) = m_fGravity(v, d) * (1. / m_vertexMasses(v));
			}
		}
	m_fExtWeighted *= m_timeStep * m_timeStep;
	m_fGravWeighted *= m_timeStep * m_timeStep;

	if (m_usingSubspaces && m_isSetup) {
		projectToSubspace(m_fExtWeightedSubspace, m_fExtWeighted);
		projectToSubspace(m_fGravWeightedSubspace, m_fGravWeighted);
	}
}

void PD::ProjDynSimulator::projectToSubspace(PDPositions& b, PDPositions& x)
{
	PDPositions rhs = m_baseFunctionsTransposed * m_massMatrix * x;
	b.resize(m_baseFunctionsTransposed.rows(), 3);
	for (int d = 0; d < 3; d++) {
		b.col(d) = m_subspaceSolver.solve(rhs.col(d));
	}
}

void PD::ProjDynSimulator::createConstraintSampling(unsigned int numSamples) {
	// If not using snapshots, or loading was not successful, use the semi-
	// equidistant sampling approach via furthest point sampling
	m_constraintVertexSamples = m_samples;
	if (m_constraintVertexSamples.empty()) {
		m_constraintVertexSamples.push_back(std::rand() % m_numVertices);
	}
	m_sampler.extendSamples(numSamples, m_constraintVertexSamples);

	PD::fillSamplesRandomly(m_constraintVertexSamples, numSamples, m_numVertices - 1);

	std::sort(m_constraintVertexSamples.begin(), m_constraintVertexSamples.end());
	m_constraintVertexSamples.erase(std::unique(m_constraintVertexSamples.begin(), m_constraintVertexSamples.end()), m_constraintVertexSamples.end());

	m_constraintTriSamples.clear();
	m_constraintTriSamples.reserve(m_constraintVertexSamples.size());
	for (unsigned int vInd : m_constraintVertexSamples) {
		if (m_vertexStars.at(vInd).size() > 0) {
			m_constraintTriSamples.push_back(m_vertexStars.at(vInd).at(0).t1);
		}
	}
	std::sort(m_constraintTriSamples.begin(), m_constraintTriSamples.end());
	m_constraintTriSamples.erase(std::unique(m_constraintTriSamples.begin(), m_constraintTriSamples.end()), m_constraintTriSamples.end());

	if (m_hasTetrahedrons) {
		m_tetsPerVertex = makeTetsPerVertexList(m_numVertices, m_tetrahedrons);

		m_constraintTetSamples.clear();
		m_constraintTetSamples.reserve(m_constraintVertexSamples.size());
		for (unsigned int vInd : m_constraintVertexSamples) {
			if (m_tetsPerVertex.at(vInd).size() > 0) {
				m_constraintTetSamples.push_back(m_tetsPerVertex.at(vInd).at(0));
			}
		}
		std::sort(m_constraintTetSamples.begin(), m_constraintTetSamples.end());
		m_constraintTetSamples.erase(std::unique(m_constraintTetSamples.begin(), m_constraintTetSamples.end()), m_constraintTetSamples.end());

		/*
		PDMatrix tetSamples;
		tetSamples.setZero(m_numVertices, 1);
		for (unsigned int t : m_constraintTetSamples) {
		for (unsigned int i = 0; i < 4; i++) {
		tetSamples(m_tetrahedrons(t, i), 0) = 1.;
		}
		}
		storeBase(tetSamples, "D:\\temp.smp");
		*/

	}

}

void PD::ProjDynSimulator::evaluatePositionsAtUsedVertices(PDPositions& usedPos, PDPositions& subPos)
{
	int vSize = m_usedVertices.size();
	int subSize = subPos.rows();
	int i = 0;
	PROJ_DYN_PARALLEL_FOR
		for (i = 0; i < vSize; i++) {
			int nnz = m_usedVerticesBase[i].size();
			for (int d = 0; d < 3; d++) {
				PDScalar sum = 0;
				for (int j = 0; j < nnz; j++) sum += m_usedVerticesBase[i].at(j) * subPos(m_usedVerticesBaseNNZ[i].at(j), d);
				usedPos(i, d) = sum;
			}
		}
}

void PD::ProjDynSimulator::resolveCollision(unsigned int v, PDPositions & pos, PDPositions & posCorrect)
{
	posCorrect.row(v) = pos.row(v);


	if (m_floorCollisionWeight > 0) {
		if (pos(v, m_floorCoordinate) < m_floorHeight) {
			m_collisionCorrection = true;
			pos(v, m_floorCoordinate) = m_floorHeight;
		}
		if (pos(v, m_floorCoordinate) > m_ceilingHeight) {
			m_collisionCorrection = true;
			pos(v, m_floorCoordinate) = m_ceilingHeight;
		}
	}

	for (CollisionObject& col : m_collisionObjects) {
		PD3dVector posV = pos.row(v);
		if (col.resolveCollision(posV)) {
			m_collisionCorrection = true;
			pos.row(v) = posV.transpose();
		}
	}

	/*
	PDScalar dist = pos.row(v).norm();
	if (dist > 6) {
	m_collisionCorrection = true;
	pos.row(v) *= 6. / dist;
	}
	*/

	posCorrect.row(v) -= pos.row(v);
	posCorrect.row(v) *= -1.;
}

/* Updates the actual positions using the subspace positions, but if rhs interpolation is used
only updates positions in the list m_usedVertices, and expects that the vector fullPos is
size m_usedVertices.size() and will fill it corresponding to the list of used vertices.
Otherwise expects the usual full position vector. */
void PD::ProjDynSimulator::updatePositionsSampling(PDPositions& fullPos, PDPositions& subPos, bool usedVerticesOnly)
{
	if (usedVerticesOnly) {
#ifdef PROJ_DYN_USE_CUBLAS_REST
		PDScalar one = 1;
		for (int d = 0; d < 3; d++) {

			m_multiplicationForPosUpdate.startStopWatch();
			/* Sparse multiplication does NOT seem worth it in this case */
			m_usedVertexUpdater->mult(subPos.data() + (d * subPos.rows()), fullPos.data() + (d * fullPos.rows()), one);
			m_multiplicationForPosUpdate.stopStopWatch();
		}
#else
		evaluatePositionsAtUsedVertices(fullPos, subPos);
#endif
		}
	else {
		fullPos = m_baseFunctions * subPos;
	}
	}

void PD::ProjDynSimulator::addConstraintSample(ProjDynConstraint * c)
{
	m_sampledConstraints.push_back(c);
	m_constraintSamplesChanged = true;
}

void PD::ProjDynSimulator::setExamplePoses(std::vector<PDPositions> exPoses, PDScalar generalWeight, bool forSprings)
{
	if (!m_hasTetrahedrons) {
		std::cout << "Example poses currently only implemented when using tet constraints." << std::endl;
		return;
	}
	else {
		// We need to extend the surface positions to the rest of the tet-mesh, which is
		// done by minimizing the internal forces from the tet-strain energy
		// while constraining the surface positions to the deformations given 
		// by the example poses.
		m_exPoses.clear();
		for (PDPositions& exPos : exPoses) {
			m_exPoses.push_back(extendSurfaceDeformationToTets(exPos));
		}
		PDScalar weight = 0.;//1. / (m_exPoses.size() + 1);
		std::vector<PDScalar> weights;
		for (unsigned int i = 0; i < exPoses.size(); i++) weights.push_back(weight);

		if (forSprings) {
			makeEdges();
			for (auto& edge : m_edges) {
				int v1Ind = edge.first;
				int v2Ind = edge.second;
				PD3dVector edge = m_positions.row(v1Ind) - m_positions.row(v2Ind);
				ProjDynConstraint* c = new SpringConstraint(
					m_numVertices, v1Ind, v2Ind, edge.norm(), generalWeight * m_normalization,
					1, 1, m_exPoses, weights);
				m_springConstraints.push_back(c);
				addConstraint(c);
			}
		}
		else {
			for (unsigned int t = 0; t < m_numTets; t++) {
				// Add example pose constraint
				TetExampleBased* tec = new TetExampleBased(m_numVertices, t, m_tetrahedrons, m_positions, m_exPoses, weights, generalWeight * m_normalization);
				addConstraint(tec);
				m_tetExConstraints.push_back(tec);
			}
		}
	}
}

PDPositions PD::ProjDynSimulator::extendSurfaceDeformationToTets(PDPositions & surfacePos)
{

	//PD::storeMesh(surfacePos, m_triangles, "D:\\surfDefOrig.obj");
	//PD::storeMesh(m_positions, m_triangles, "D:\\orig.obj");

	int numInnerVerts = m_numVertices - m_numOuterVertices;
	if (!m_surfDefExtInit) {
		// Set up the constraints, the matrices and the solver for surface
		// deformation extensions
		std::vector<Eigen::Triplet<double>> s1Entries;
		std::vector<Eigen::Triplet<double>> s2Entries;
		s1Entries.reserve(m_numTets * 12);
		s2Entries.reserve(m_numTets * 12);
		for (unsigned int t = 0; t < m_numTets; t++) {
			TetStrainConstraint* tc = new TetStrainConstraint(m_numVertices, t, m_tetrahedrons, m_positions, 1, 1, m_normalization);
			m_surfDefExtConstraints.push_back(tc);
			PDSparseMatrixRM& sel = tc->getSelectionMatrix();
			PDScalar weight = std::sqrt(tc->getWeight());
			for (int k = 0; k < sel.outerSize(); ++k) {
				for (PDSparseMatrixRM::InnerIterator it(sel, k); it; ++it) {
					if (it.col() < m_numOuterVertices) {
						s1Entries.push_back(Eigen::Triplet<PDScalar>(t * 3 + it.row(), it.col(), it.value() * weight));
					}
					else {
						s2Entries.push_back(Eigen::Triplet<PDScalar>(t * 3 + it.row(), it.col() - m_numOuterVertices, it.value() * weight));
					}
				}
			}
		}

		PDSparseMatrix S1(m_numTets * 3, m_numOuterVertices);
		PDSparseMatrix S2(m_numTets * 3, numInnerVerts);
		S1.setFromTriplets(s1Entries.begin(), s1Entries.end());
		S2.setFromTriplets(s2Entries.begin(), s2Entries.end());
		m_surfDefExtRHSMat = S2.transpose();
		PDSparseMatrix lhs = m_surfDefExtRHSMat * S2;
		m_surfDefExtSolver.compute(lhs);
		m_surfDefExtFixedPartMat = S1;
	}

	// Compute the fixed part of the rhs (which is S2^T * (p - S1 * pos_outer))
	PDPositions rhsFixedPart = m_surfDefExtFixedPartMat * surfacePos;
	PDPositions innerVerts(numInnerVerts, 3);
	// Initial guess: set all to zero
	innerVerts.setZero();

	int numIterations = 100;
	PDPositions fullPos(m_numVertices, 3);
	PDPositions auxils(m_numTets * 3, 3);
	PDPositions rhs(numInnerVerts, 3);
	fullPos.block(0, 0, m_numOuterVertices, 3) = surfacePos;
	for (unsigned int it = 0; it < numIterations; it++) {
		// Local step:
		// First attach the inner and outer vertex positions together to evaluate projections
		fullPos.block(m_numOuterVertices, 0, numInnerVerts, 3) = innerVerts;
		// Then evaluate all projections in parallel
		int didCollide = -1;
		PROJ_DYN_PARALLEL_FOR
			for (int i = 0; i < m_surfDefExtConstraints.size(); i++) {
				auxils.block(i * 3, 0, 3, 3) = m_surfDefExtConstraints[i]->getP(fullPos, didCollide) * m_surfDefExtConstraints[i]->getWeight();
			}

		// Global step:
		// Form rhs:
		rhs = m_surfDefExtRHSMat * (auxils - rhsFixedPart);
		// Solve system
		PROJ_DYN_PARALLEL_FOR
			for (int d = 0; d < 3; d++) {
				innerVerts.col(d) = m_surfDefExtSolver.solve(rhs.col(d));
			}
	}

	fullPos.block(m_numOuterVertices, 0, numInnerVerts, 3) = innerVerts;

	PD::storeMesh(fullPos, m_triangles, "D:\\surfDefExt.obj");

	return fullPos;
}

void PD::ProjDynSimulator::setExampleWeights(std::vector<PDScalar>& exWeights)
{
	if (m_rhsInterpolation) {
		for (auto& g : m_snapshotGroups) {
			if (g.getName() == "tetex" || g.getName() == "spring") {
				g.setExampleWeights(exWeights);
			}
		}
	}
	else {
		for (ProjDynConstraint* tec : m_tetExConstraints) {
			((TetExampleBased*)tec)->setExampleWeights(exWeights);
		}
	}
}

void PD::ProjDynSimulator::updateUsedVertices()
{
	std::cout << "Creating a list of used vertices and adapting constraints to this list..." << std::endl;

	m_usedVertexMap = new int[m_numVertices];
	for (unsigned int i = 0; i < m_numVertices; i++) m_usedVertexMap[i] = -1;

	m_usedVertices.clear();
	for (unsigned int v : m_additionalUsedVertices) {
		m_usedVertices.push_back(v);
	}
	for (ProjDynConstraint* c : m_sampledConstraints) {
		bool first = true;
		PDSparseMatrix sel = c->getSelectionMatrixTransposed();
		for (int k = 0; k < sel.outerSize(); ++k) {
			for (PDSparseMatrix::InnerIterator it(sel, k); it; ++it)
			{
				if (it.value() != PDScalar(0)) {
					m_usedVertices.push_back(it.row());
					if (first) {
						m_usedVerticesSlim.push_back(it.row());
						first = false;
					}
				}
			}
		}
	}
	if (m_usedVertices.size() < 800) {
		fillSamplesRandomly(m_usedVertices, 800, m_numVertices);
	}
	std::sort(m_usedVertices.begin(), m_usedVertices.end());
	m_usedVertices.erase(std::unique(m_usedVertices.begin(), m_usedVertices.end()), m_usedVertices.end());

	std::sort(m_usedVerticesSlim.begin(), m_usedVerticesSlim.end());
	m_usedVerticesSlim.erase(std::unique(m_usedVerticesSlim.begin(), m_usedVerticesSlim.end()), m_usedVerticesSlim.end());

	m_slimToUsedIndices.resize(m_usedVerticesSlim.size());
	for (unsigned int i = 0; i < m_usedVerticesSlim.size(); i++) {
		auto const& it = std::find(m_usedVertices.begin(), m_usedVertices.end(), m_usedVerticesSlim[i]);
		if (it == m_usedVertices.end()) {
			std::cout << "Error while slimming used vertices..." << std::endl;
		}
		else {
			m_slimToUsedIndices[i] = *it;
		}
	}

	for (unsigned int i = 0; i < m_usedVertices.size(); i++) {
		m_usedVertexMap[m_usedVertices[i]] = i;
	}

	m_positionsUsedVs.setZero(m_usedVertices.size(), 3);
	m_positionCorrectionsUsedVs.setZero(m_usedVertices.size(), 3);
	m_velocitiesUsedVs.setZero(m_usedVertices.size(), 3);

	PDMatrix A(m_usedVertices.size(), m_baseFunctions.cols());
	unsigned int i = 0;
	for (unsigned int v : m_usedVertices) {
		A.row(i) = m_baseFunctions.row(v);
		i++;
	}
	m_usedVertexInterpolatorRHSMatrix = A.transpose();
	PDMatrix lhsMat = A.transpose() * A;
	m_usedVertexInterpolator.compute(lhsMat);

	if (m_usedVertexInterpolator.info() != Eigen::Success) {
		std::cout << "Warning: Factorization of lhs matrix for used vertex interoplation was not successful!" << std::endl;
	}

	if (m_useSparseMatricesForSubspace) {
		m_usedVertexInterpolatorRHSMatrixSparse = m_usedVertexInterpolatorRHSMatrix.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF);
		PDSparseMatrix lhsMatSparse = lhsMat.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF_HIGH_PREC);
		m_usedVertexInterpolatorSparse.compute(lhsMatSparse);
		if (m_usedVertexInterpolatorSparse.info() != Eigen::Success) {
			std::cout << "Warning: Factorization of the SPARSE lhs matrix for used vertex interoplation was not successful!" << std::endl;
		}
	}

	for (ProjDynConstraint* c : m_sampledConstraints) {
		c->setUsedVertices(m_usedVertices);
	}

#ifdef PROJ_DYN_USE_CUBLAS_REST
	if (m_usedVertexUpdater != nullptr) {
		delete m_usedVertexUpdater;
	}
	if (m_rhsEvaluator != nullptr) {
		delete m_rhsEvaluator;
	}
	m_projUsedVerts.setZero(m_usedVertices.size(), m_baseFunctions.cols());
	for (int i = 0; i < m_usedVertices.size(); i++) {
		m_projUsedVerts.row(i) = m_baseFunctions.row(m_usedVertices[i]);
	}
	m_usedVertexUpdater = new CUDAMatrixVectorMultiplier(m_projUsedVerts);
	m_curTempVec.setZero(m_usedVertices.size());

	m_rhsEvalMat = m_projUsedVerts.transpose();
	m_rhsEvaluator = new CUDAMatrixVectorMultiplier(m_rhsEvalMat);

#else
	// Collect non-zero entries required to evaluate 
	m_usedVerticesBase.resize(m_usedVertices.size());
	m_usedVerticesBaseNNZ.resize(m_usedVertices.size());
	for (int v = 0; v < m_usedVertices.size(); v++) {
		int nnz = 0;
		std::vector<PDScalar> entries;
		std::vector<unsigned int> inds;
		for (int u = 0; u < m_baseFunctions.cols(); u++) {
			if (abs(m_baseFunctions(m_usedVertices[v], u)) > PROJ_DYN_SPARSITY_CUTOFF) {
				entries.push_back(m_baseFunctions(m_usedVertices[v], u));
				inds.push_back(u);
				nnz++;
			}
		}
		m_usedVerticesBase[v] = entries;
		m_usedVerticesBaseNNZ[v] = inds;
}
#endif

	m_constraintSamplesChanged = false;
}


PDMatrix PD::ProjDynSimulator::createSkinningWeights(unsigned int numSamples, PDScalar rMultiplier) {
	std::vector<unsigned int> samples = m_sampler.getSamples(numSamples);
	PDScalar furthestDist = m_sampler.getSampleDiameter(samples);
	PDScalar r = furthestDist * rMultiplier;
	PDMatrix weightMat = m_sampler.getRadialBaseFunctions(samples, true, r);

	/*
	PD::storeMesh(m_positions, m_triangles, "D:\\temp.obj");
	PD::storeBase(weightMat, "D:\\temp.smp");
	*/

	return weightMat;
}

void PD::ProjDynSimulator::createPositionSubspace(unsigned int numSamples, bool useSkinningSpace)
{
	StopWatch samplingT(10, 10);

	samplingT.startStopWatch();
	m_samples = m_sampler.getSamples(numSamples);
	samplingT.stopStopWatch();
	std::cout << "	Time to choose samples: " << (samplingT.lastMeasurement() / 1000000.0) << "s" << std::endl;

	numSamples = m_samples.size();
	std::sort(m_samples.begin(), m_samples.end());
	m_samples.erase(std::unique(m_samples.begin(), m_samples.end()), m_samples.end());

	PDScalar furthestDist = m_sampler.getSampleDiameter(m_samples);
	m_usedRadius = furthestDist * m_baseFunctionRadius;

	samplingT.startStopWatch();
	m_baseFunctionWeights = m_sampler.getRadialBaseFunctions(m_samples, true, m_usedRadius);
	samplingT.stopStopWatch();
	std::cout << "	Time to compute weights: " << (samplingT.lastMeasurement() / 1000000.0) << "s" << std::endl;

	if (useSkinningSpace) {
		bool isFlat = false;
		if (m_positions.col(2).norm() < 1e-10) isFlat = true;
		m_baseFunctions = createSkinningSpace(m_positions, m_baseFunctionWeights, nullptr, 1U, nullptr, nullptr, isFlat);
	}
	else {
		m_baseFunctions = m_baseFunctionWeights;
		int ind = m_baseFunctions.cols();
		m_baseFunctions.conservativeResize(m_baseFunctions.rows(), m_baseFunctions.cols() + 3);
		m_baseFunctions.col(ind) = m_positions.col(0);
		m_baseFunctions.col(ind + 1) = m_positions.col(1);
		m_baseFunctions.col(ind + 2) = m_positions.col(2);
	}

	/*
	PD::storeMesh(m_positions, m_triangles, "D:\\temp.obj");
	PD::storeBase(m_baseFunctionWeights, "D:\\temp.smp");
	*/


	/*
	std::cout << "Storing subspaces... " << std::endl;
	storeBaseBinary(m_baseFunctions, getMeshFileName(m_meshURL, "_posSub.bas"));3
	storeBaseBinary(m_baseFunctionWeights, getMeshFileName(m_meshURL, "_weights.bas"));
	*/
}

void PD::ProjDynSimulator::setExternalForces(PDPositions fExt)
{
	m_fExt = fExt;
	recomputeWeightedForces();
}

void PD::ProjDynSimulator::addGravity(PDScalar gravity, int coord)
{
	m_gravity = gravity;
	m_fGravity.setZero(m_numVertices, 3);
	PROJ_DYN_PARALLEL_FOR
		for (int v = 0; v < m_numVertices; v++) {
			m_fGravity(v, coord) = -gravity * m_vertexMasses(v);
		}
	recomputeWeightedForces();
}

void PD::ProjDynSimulator::addFloor(int floorCoordinate, PDScalar floorHeight, PDScalar floorCollisionWeight, PDScalar ceilingHeight)
{
	m_floorCoordinate = floorCoordinate;
	m_floorHeight = floorHeight;
	m_floorCollisionWeight = floorCollisionWeight;
	m_ceilingHeight = ceilingHeight;
}

void PD::ProjDynSimulator::setFrictionCoefficient(PDScalar coeff, PDScalar rCoeff) {
	m_frictionCoeff = coeff;
	if (rCoeff < 0) {
		m_repulsionCoeff = coeff;
	}
	else {
		m_repulsionCoeff = rCoeff;
	}
}


void PD::ProjDynSimulator::addEdgeSprings(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax)
{

	makeEdges();

	for (auto& edge : m_edges) {

		int v1Ind = edge.first;
		int v2Ind = edge.second;

		PD3dVector edge = m_positions.row(v1Ind) - m_positions.row(v2Ind);
		ProjDynConstraint* c = new SpringConstraint(
			m_numVertices, v1Ind, v2Ind, edge.norm(), weight * m_normalization,
			rangeMin, rangeMax);
		m_springConstraints.push_back(c);
		addConstraint(c);

	}
}

void PD::ProjDynSimulator::addTriangleStrain(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax)
{
	for (int t = 0; t < m_numTriangles; t++) {
		StrainConstraint* sc = new StrainConstraint(m_numVertices, t, m_triangles, m_positions, rangeMin, rangeMax, weight * m_normalization);
		addConstraint(sc);
		m_strainConstraints.push_back(sc);
	}
}

void PD::ProjDynSimulator::addTetStrain(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax, PDScalar tetIncWeight)
{
	if (m_hasTetrahedrons) {
		for (int t = 0; t < m_numTets; t++) {
			PDScalar avgX = 1; // (m_positions(m_tetrahedrons(t, 0), 0) + m_positions(m_tetrahedrons(t, 1), 0) + m_positions(m_tetrahedrons(t, 2), 0) + m_positions(m_tetrahedrons(t, 3), 0)) / 4.;
			TetStrainConstraint* sc = new TetStrainConstraint(m_numVertices, t, m_tetrahedrons, m_positions, rangeMin, rangeMax, (weight * m_normalization) / (avgX < 0 ? 10. : 1.), tetIncWeight);
			addConstraint(sc);
			m_tetStrainConstraints.push_back(sc);
		}
	}
	else {
		std::cout << "Could not add tet-strain, no tetrahedrons available!" << std::endl;
	}
}

void PD::ProjDynSimulator::addBendingConstraints(PDScalar weight, bool preventBendingFlips, bool flatBending)
{
	std::vector< std::vector< Edge > >& vertexStars = m_vertexStars;

	PDVector vertexTriMasses = vertexMasses(m_triangles, m_positions);

	for (int v = 0; v < m_numVertices; v++) {
		bool dontAdd = false;
		if (vertexStars.at(v).empty()) {
			dontAdd = true;
		}
		for (Edge& e : vertexStars.at(v)) {
			if (e.t2 < 0) {
				dontAdd = true;
			}
		}

		if (!dontAdd) {
			BendingConstraint* bc = new BendingConstraint(m_numVertices, v, m_positions, m_triangles, vertexStars.at(v), vertexTriMasses(v), weight * m_normalization, preventBendingFlips, flatBending);
			addConstraint(bc);
			m_bendingConstraints.push_back(bc);
		}

	}

	if (flatBending) m_flatBending = true;
}


PD::ProjDynSimulator::~ProjDynSimulator()
{

}

void PD::ProjDynSimulator::setParallelVUpdateBlockSize(int blocks)
{
	m_parallelVUpdateBSize = blocks;
	updateParallelVUpdateBlocks();
}

void PD::ProjDynSimulator::setEnableVUpdate(bool enable)
{
	m_parallelVUpdate = enable;
}

void PD::ProjDynSimulator::makeEdges()
{
	std::vector<std::vector<int>> visitedEdge;
	visitedEdge.resize(m_numVertices);

	if (m_hasTetrahedrons) {
		for (int t = 0; t < m_numTets; t++) {
			for (int e = 0; e < 4; e++) {
				int v1Ind = m_tetrahedrons(t, e), v2Ind = m_tetrahedrons(t, (e + 1) % 4);
				if (v1Ind > v2Ind) {
					int tmp = v1Ind;
					v1Ind = v2Ind;
					v2Ind = tmp;
				}

				if (std::find(visitedEdge[v1Ind].begin(), visitedEdge[v1Ind].end(), v2Ind) != visitedEdge[v1Ind].end()) {
					continue;
				}

				m_edges.push_back(std::pair<unsigned int, unsigned int>(v1Ind, v2Ind));
				visitedEdge[v1Ind].push_back(v2Ind);
			}
		}
	}
	else {
		for (int t = 0; t < m_numTriangles; t++) {
			for (int e = 0; e < 3; e++) {
				int v1Ind = m_triangles(t, e), v2Ind = m_triangles(t, (e + 1) % 3);
				if (v1Ind > v2Ind) {
					int tmp = v1Ind;
					v1Ind = v2Ind;
					v2Ind = tmp;
				}

				if (std::find(visitedEdge[v1Ind].begin(), visitedEdge[v1Ind].end(), v2Ind) != visitedEdge[v1Ind].end()) {
					continue;
				}

				m_edges.push_back(std::pair<unsigned int, unsigned int>(v1Ind, v2Ind));
				visitedEdge[v1Ind].push_back(v2Ind);
			}
		}
	}

}

void PD::ProjDynSimulator::updateParallelVUpdateBlocks()
{
	if (m_parallelVUpdate) {
		std::cout << "Setting up parallel blocks for vertex positions updates..." << std::endl;

		int blockSize = PROJ_DYN_VPOS_BLOCK_SIZE;
		int numBlocks = std::ceil((float)m_positions.rows() / (float)blockSize);
		m_baseFunctionsSparseBlocks.clear();
		for (int b = 0; b < numBlocks; b++) {
			int curSize = blockSize;
			if (b == numBlocks - 1) {
				curSize = blockSize - (numBlocks * blockSize - m_positions.rows());
			}
			PDSparseMatrixRM curBlock(curSize, m_baseFunctionsSparse.cols());
			curBlock = m_baseFunctionsSparse.block(b*blockSize, 0, curSize, m_baseFunctionsSparse.cols());
			m_baseFunctionsSparseBlocks.push_back(curBlock);
		}
	}
}

void PD::ProjDynSimulator::initializeGPUVPosMapping(GLuint bufferId)
{
#ifdef PROJ_DYN_USE_CUBLAS_REST
#ifdef ENABLE_DIRECT_BUFFER_MAP
	if (m_vPosGPUUpdate) {
		delete m_vPosGPUUpdate;
	}

	if (m_usingSubspaces && m_baseFunctions.cols() > 0) {
		m_vPosGPUUpdate = new CUDAMatrixVectorMultiplier(m_baseFunctions);
		m_vPosGPUUpdate->setGLBuffer(bufferId);
	}
#endif
#endif
}

PDPositions & PD::ProjDynSimulator::getUsedVertexPositions()
{
	if (m_rhsInterpolation) {
		return m_positionsUsedVs;
	}
	else {
		return m_positions;
	}
}



void ProjDynSimulator::addConstraint(ProjDynConstraint* c, bool alwaysAdd) {
	m_isSetup = false;
	m_constraints.push_back(c);

	if (m_rhsInterpolation) {
		bool containsSample = false;
		if (!alwaysAdd) {
			if (c->getMainVertexIndex() > 0) {
				int vInd = c->getMainVertexIndex();
				if (std::find(m_constraintVertexSamples.begin(), m_constraintVertexSamples.end(), vInd) != m_constraintVertexSamples.end()) {
					containsSample = true;
				}
			}
			else if (c->getMainTriangleIndex() > 0) {
				int tInd = c->getMainTriangleIndex();
				if (std::find(m_constraintTriSamples.begin(), m_constraintTriSamples.end(), tInd) != m_constraintTriSamples.end()) {
					containsSample = true;
				}
			}
			else if (c->getMainTetIndex() > 0) {
				int tetInd = c->getMainTetIndex();
				if (std::find(m_constraintTetSamples.begin(), m_constraintTetSamples.end(), tetInd) != m_constraintTetSamples.end()) {
					containsSample = true;
				}
			}
		}
		if (containsSample || alwaysAdd) {
			addConstraintSample(c);
		}
		if (alwaysAdd) {
			m_additionalConstraints.push_back(c);
		}
	}
}

void PD::ProjDynSimulator::addConstraintDynamic(ProjDynConstraint * c)
{
	PDSparseMatrix L_add_sparse = m_baseFunctionsTransposedSparse * ((c->getSelectionMatrixTransposed() * c->getSelectionMatrix()) * m_baseFunctionsSparse);
	m_subspaceLHS_inner += c->getWeight() * L_add_sparse.toDense();
	c->setUsedVertices(m_usedVertices);

	m_sampledConstraints.push_back(c);
	m_additionalConstraints.push_back(c);

	refreshLHS();
}

void PD::ProjDynSimulator::addHangConstraint(PositionConstraint * c)
{
	m_hangConstraints.push_back(c);
}

void PD::ProjDynSimulator::removeConstraintDynamic(ProjDynConstraint * c)
{
	auto it = std::find(m_sampledConstraints.begin(), m_sampledConstraints.end(), c);
	if (it != m_sampledConstraints.end())
		m_sampledConstraints.erase(it);
	else
		return;
	auto it2 = std::find(m_additionalConstraints.begin(), m_additionalConstraints.end(), c);
	if (it2 != m_additionalConstraints.end())
		m_additionalConstraints.erase(it2);
	else
		return;

	PDSparseMatrix L_add_sparse = m_baseFunctionsTransposedSparse * ((c->getSelectionMatrixTransposed() * c->getSelectionMatrix()) * m_baseFunctionsSparse);
	m_subspaceLHS_inner -= c->getWeight() * L_add_sparse.toDense();


	refreshLHS();
}

void ProjDynSimulator::printTimeMeasurements() {
	std::cout << "m_timeStep: " << m_timeStep << std::endl;
	std::vector< ProjDynConstraint* >* usedConstraints = &m_constraints;
	if (m_rhsInterpolation) {
		usedConstraints = &m_sampledConstraints;
	}
	int numConstraints = usedConstraints->size();
	std::cout << "===========================================================" << std::endl;
	std::cout << "# of constraints: " << numConstraints << ", using " << m_usedVertices.size() << " of " << m_numVertices << " vertices." << std::endl;
	if (m_picflipsimulator) {
		std::cout << "# of particles: " << m_picflipsimulator->getNumParticles() << " (" << m_picflipsimulator->getNumFreeParticles() << " + " << (m_picflipsimulator->getNumParticles() - m_picflipsimulator->getNumFreeParticles()) << ")" << std::endl;
		std::cout << "# grid cells: " << m_picflipsimulator->getGridSize() << std::endl;
	}
	std::cout << "Time for precomputation: " << m_precomputationStopWatch.lastMeasurement() / 1000 << " milliseconds" << std::endl;
	PDScalar fps = 1000000. / (m_localStepStopWatch.evaluateAverage() * m_numIterations + m_globalStepStopWatch.evaluateAverage() * m_numIterations + m_updatingVPosStopWatch.evaluateAverage() * m_numIterations + m_picflipStopWatch.evaluateAverage());
	PDScalar fpsD = 1000000. / (m_localStepStopWatch.evaluateAverage() * m_numIterations + m_globalStepStopWatch.evaluateAverage() * m_numIterations + m_updatingVPosStopWatch.evaluateAverage() * m_numIterations + m_picflipStopWatch.evaluateAverage() + m_fullUpdateStopWatch.evaluateAverage());
	std::cout << "FPS: " << fps << " (" << fpsD << ")" << std::endl;
	std::cout << "Average time for one local step: " << m_localStepStopWatch.evaluateAverage() << " microseconds" << std::endl;
	std::cout << "		Average time for only the constraint projection in the local step (includes summation if using rhs interpolation): " << m_localStepOnlyProjectStopWatch.evaluateAverage() << " microseconds" << std::endl;
	std::cout << "		Average time for the rest of the local step: " << m_localStepRestStopWatch.evaluateAverage() << " microseconds" << std::endl;
	std::cout << "			----> RHS-Reset/Summation/Projection+Moment. " << (m_localStepRestStopWatch.evaluateAverage() - m_momentumStopWatch.evaluateAverage() - m_constraintSummationStopWatch.evaluateAverage()) << "/" << m_constraintSummationStopWatch.evaluateAverage() << "/" << m_momentumStopWatch.evaluateAverage() << " microseconds" << std::endl;
#ifdef PROJ_DYN_USE_CUBLAS_REST
	if (m_usingSubspaces && m_rhsInterpolation) {
		std::cout << "			----> Multiplication for Projection set/get/Dgemv: ";
		m_rhsEvaluator->printTimings();
		std::cout << std::endl;
	}
#endif

	std::cout << "Average time for one global step: " << m_globalStepStopWatch.evaluateAverage() << " microseconds" << std::endl;
	std::cout << "Average time for updating relevant v. pos.: " << m_updatingVPosStopWatch.evaluateAverage() << " microseconds" << std::endl;
#ifdef PROJ_DYN_USE_CUBLAS_REST
	if (m_usingSubspaces) {
		std::cout << "        ----> Multiplication(set/get/Dgemv)/Ordering: " << (m_multiplicationForPosUpdate.evaluateAverage()) << "(";
		m_usedVertexUpdater->printTimings();
		std::cout << ")" << "/" << (m_sortingForPosUpdate.evaluateAverage()) << " (three multiplciations per inner it.)" << std::endl;
	}
#endif
	std::cout << "Average time for local-global loop with " << m_numIterations << " iterations (excludes incomp.): " << m_elasticityStopWatch.evaluateAverage() << std::endl;
	std::cout << "Average time for the incompressiblity and fluids with " << m_pfStepsPerFrame << " substeps: " << m_picflipStopWatch.evaluateAverage() << std::endl;
	std::cout << "			----> Time for subspace-particle transfers: " << m_particleTransferStopWatch.evaluateAverage() << std::endl;
	std::cout << "Average time of only the surrounding block of a full step: " << m_surroundingBlockStopWatch.evaluateAverage() << std::endl;
	std::cout << "			----> Full vertex update: " << m_fullUpdateStopWatch.evaluateAverage() << std::endl;
	std::cout << "Average time for a full time-step (includes everything): " << m_totalStopWatch.evaluateAverage() << std::endl;
	float totalAverage = m_totalStopWatch.evaluateAverage();
	float surroundingAverage = m_surroundingBlockStopWatch.evaluateAverage();
	float picflipAverage = m_picflipStopWatch.evaluateAverage();
	float restTime = totalAverage - surroundingAverage - picflipAverage;
	float localToGlobal = (float)(m_localStepStopWatch.evaluateAverage() + m_updatingVPosStopWatch.evaluateAverage()) / (float)(m_globalStepStopWatch.evaluateAverage() + m_updatingVPosStopWatch.evaluateAverage() + m_localStepStopWatch.evaluateAverage());
	float localAverage = restTime * localToGlobal;
	float globalAverage = restTime * (1.f - localToGlobal);
	std::cout << "The local steps took " << (localAverage / totalAverage) * 100 << "% of the total time." << std::endl;
	std::cout << "The global (incl. v. pos. upd.) steps took " << (globalAverage / totalAverage) * 100 << "% of the total time." << std::endl;
	std::cout << "The incompressibility steps took " << (picflipAverage / totalAverage) * 100 << "% of the total time." << std::endl;
	std::cout << "The surrounding block took " << (surroundingAverage / totalAverage) * 100 << "% of the total time." << std::endl;
	std::cout << "		----> Full update of vertex positions: " << m_fullUpdateStopWatch.evaluateAverage() << std::endl;
	std::cout << "Total number of refactorizations: " << m_numRefactorizations << std::endl;
	std::cout << "===========================================================" << std::endl;
}

PDScalar PD::ProjDynSimulator::evaluateEnergy(PDPositions & q, PDPositions & s)
{
	PDPositions momVec = (q - s);
	PDScalar momE = (momVec.transpose() * m_massMatrix * momVec).trace();
	PDScalar innerE = 0;
	int dummyI = 0;
	PROJ_DYN_PARALLEL_FOR
		for (int i = 0; i < m_constraints.size(); i++) {
			ProjDynConstraint* c = m_constraints[i];
			PDPositions actualP = c->getSelectionMatrix() * q;
			PDPositions desiredP = c->getP(q, dummyI);
			PDScalar pNormSquared = (actualP - desiredP).norm();
			pNormSquared = pNormSquared * pNormSquared;
			innerE += (c->getWeight() / 2.) * pNormSquared;
		}
	PDScalar totE = (1. / (2. * m_timeStep * m_timeStep)) * momE + innerE;
	//std::cout << "Mom. En. : " << momE << "; Elastic Potential: " << innerE << "; Total: " << totE << std::endl;
	return totE;
}

void PD::ProjDynSimulator::setInitialPos(PDPositions & startPos)
{
	PDMatrix posMat;
	if (PD::loadBaseBinary(PD::getMeshFileName(m_meshURL, "_start.pos"), posMat)) {
		std::cout << "Found previously extended and possibly projected starting positions, loaded these..." << std::endl;
		m_positions = posMat;
		m_initialPos = m_positions;
	}
	else {
		std::cout << "Setting initial configuration..." << std::endl;

		if (!m_hasTetrahedrons) {
			m_positions = startPos;
		}
		else {
			std::cout << "	Extending initial surface configuration to interior..." << std::endl;
			m_positions = extendSurfaceDeformationToTets(startPos);
		}
		m_initialPos = m_positions;

	}
	if (m_usingSubspaces) {
		std::cout << "	Projecting initial configuration to subspace..." << std::endl;
		projectToSubspace(m_positionsSubspace, m_positions);
		m_positions = m_baseFunctions * m_positionsSubspace;
		m_initialPos = m_positions;
		m_initialPosSub = m_positionsSubspace;

	}
	posMat = m_positions;
	PD::storeBaseBinary(posMat, PD::getMeshFileName(m_meshURL, "_start.pos"));
}

void PD::ProjDynSimulator::setBlowup(PDScalar strength)
{
	m_blowupStrength = strength;
	for (auto& g : m_snapshotGroups) {
		if (g.getName() == "tetstrain" || g.getName() == "tetex") {
			g.setBlowup(strength);
		}
	}
}
void PD::ProjDynSimulator::addPICFLIP(int numParticlesLongestSide, bool preventSolidPen, bool addPool,
	bool addStream, double poolGap, double poolHeight, double gridExpac,
	double solidSamplingFac, bool useNormals, float forceFloorHeight, bool showMeshParticles,
	float streamSize)
{
	std::vector<PD3dVector> samplePoints;
	PDScalar particleSpacing = -1;
	std::vector<bool> onBd;
	if (!m_hasTetrahedrons) {
		errorExit("Mesh needs tetrahedrons for pressure constraints!");
	}
	std::pair<PD3dVector, PD3dVector> bb = meshBoundingBox(m_positions);
	particleSpacing = (bb.second - bb.first).maxCoeff() / (double)numParticlesLongestSide;
	PDPositions particleNormals(0, 3);
	PDPositions mesh = particlesFromTetMesh(m_positions, m_tetrahedrons, particleSpacing / solidSamplingFac,
		bb, onBd, useNormals ? m_triangles : PDTriangles(0, 3), particleNormals);
	m_numMeshParticles = mesh.rows();
	onBd.clear(); // Don't make use of boundary vertices!
	PDScalar partMass = (PDScalar)m_vertexMasses.sum() / (PDScalar)m_vertexMasses.rows();
	if (m_usingSubspaces) {
		samplePoints.resize(m_samples.size());
		for (int i = 0; i < m_samples.size(); i++) {
			samplePoints[i] = m_positions.row(m_samples[i]);
		}
	}

	float ppcSide = std::pow(PF_PPC, 1. / 3.);
	m_picflipsimulator = createPicFlip(addPool ? numParticlesLongestSide * 1.2 / ppcSide : 0,
		addPool ? numParticlesLongestSide * poolHeight / ppcSide : 0,
		particleSpacing * ppcSide, poolGap, partMass,
		m_gravity, m_particleBufferID, m_particleColorBufferID, mesh, samplePoints,
		m_timeStep, m_usedRadius, gridExpac, showMeshParticles, addStream,
		useNormals ? particleNormals : PDPositions(0, 3),
		preventSolidPen, forceFloorHeight, streamSize);

	m_particlesInitial.resize(m_picflipsimulator->getNumFreeParticles(), 3);
	m_picflipsimulator->getFreeParticlePositions(m_particlesInitial.data());
	m_numFreeParticles = m_picflipsimulator->getNumFreeParticles();

}


const PDPositions & PD::ProjDynSimulator::getParticles()
{
	return m_positions;
}

void PD::ProjDynSimulator::setPICFLIPParams(float maxDensity, float densityCorrection, float flipness, float sphRegularizationWeight, bool skipGridStep, float fluidDensity,
	float velocityRestore, int maxSSFluid, int maxSSSolid, int pfStepsPerFrame, float frictionFac, float solidInfluence, int jacobiIts, float colorDeviation,
	float maxDCorrection, bool skipIncompressibility)
{
	m_maxDensity = maxDensity;
	m_densityCorrection = densityCorrection;
	m_flipness = flipness;
	m_skipGridStep = skipGridStep;
	m_fluidDensity = fluidDensity;
	m_velocityRestore = velocityRestore;
	m_maxSSFluid = maxSSFluid;
	m_maxSSSolid = maxSSSolid;
	m_pfStepsPerFrame = pfStepsPerFrame;
	m_pfFriction = frictionFac;
	m_solidInfluence = solidInfluence;
	m_jacobiIts = jacobiIts;
	m_colorDeviation = colorDeviation;
	m_maxDCorrection = maxDCorrection;
	m_skipIncompressibility = skipIncompressibility;
	if (m_picflipsimulator) {
		m_picflipsimulator->setupSubspaceProjection(sphRegularizationWeight);
	}
}

bool PD::ProjDynSimulator::hasParticles()
{
	if (m_picflipsimulator) {
		return true;
	}
	return false;
}

int PD::ProjDynSimulator::getNumFreeParticles()
{
	if (m_picflipsimulator) {
		return m_picflipsimulator->getNumFreeParticles();
	}
	return 0;
}

int PD::ProjDynSimulator::getNumParticles()
{
	if (m_picflipsimulator) {
		return m_picflipsimulator->getNumParticles();
	}
	return 0;
}

void PD::ProjDynSimulator::addHandleConstraint(CenterConstraint * cc)
{
	m_handleConstraints.push_back(cc);
}

void PD::ProjDynSimulator::changeTimeStep(PDScalar newTimeStep)
{
	if (m_usingSubspaces) {
		m_timeStep = newTimeStep;

		refreshLHS();

		m_rhsFirstTermMatrix = m_rhsFirstTermMatrixPre * (1. / (m_timeStep * m_timeStep));

		if (m_useSparseMatricesForSubspace) {
			m_rhsFirstTermMatrixSparse = m_rhsFirstTermMatrix.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF_HIGH_PREC);
		}

		recomputeWeightedForces();
	}
	else {
		std::cout << "Changing time-steps is not supported for the full system!" << std::endl;
	}
}


void ProjDynSimulator::addAdditionalConstraints(PDPositions& pos, PDPositions& rhs, bool* collidedVertices) {
	if (!m_additionalConstraints.empty()) {

		int numAddConstraints = m_additionalConstraints.size();
		if (m_additionalConstraintsAuxTemp.size() != numAddConstraints) {
			m_additionalConstraintsAuxTemp.resize(numAddConstraints);
		}

		PROJ_DYN_PARALLEL_FOR
			for (int j = 0; j < numAddConstraints; j++) {
				ProjDynConstraint* c = m_additionalConstraints.at(j);
				int didCollide = -1;
				PDPositions aux = c->getP(pos, didCollide);
				if (m_usingSubspaces) {
					m_additionalConstraintsAuxTemp[j] = c->getSubspaceRHSMat(m_baseFunctionsTransposed) * (aux * c->getWeight()) * m_stiffnessFactor;
				}
				else {
					m_additionalConstraintsAuxTemp[j] = c->getSelectionMatrixTransposed() * (aux * c->getWeight())  * m_stiffnessFactor;
				}
				if (didCollide > 0) collidedVertices[didCollide] = true;
			}

		PROJ_DYN_PARALLEL_FOR
			for (int d = 0; d < 3; d++) {
				for (int j = 0; j < numAddConstraints; j++) {
					rhs.col(d) += m_additionalConstraintsAuxTemp[j].col(d);
				}
			}
	}
}

/* Initialized the rhs interpolation/fitting method. */
void PD::ProjDynSimulator::initRHSInterpolGroup(RHSInterpolationGroup& g, std::vector<unsigned int>& samples, PDMatrix& hessian) {
	if (!m_usingSubspaces) {
		std::cout << "Error! Not using subspaces and not loading snapshot space, cannot use rhs interpolation / constraint sampling!" << std::endl;
		return;
	}

	if (m_rhsInterpolReusableWeights.rows() <= 0) {
		std::cout << "Computing reusable weights for the rhs interpolation bases..." << std::endl;
		m_rhsInterpolReusableWeights = createSkinningWeights(m_rhsInterpolBaseSize * 0.5, m_rhsInterpolWeightRadiusMultiplier);
	}
	if (m_rhsInterpolReusableWeights.hasNaN()) {
		std::cout << "Warning: NaN entries in reusable weights!" << std::endl;
	}
	PDMatrix weights;
	if (g.getName() == "bend" || g.getName() == "spring") {
		weights = m_rhsInterpolReusableWeights;
	}
	else if (g.getName() == "strain") {
		std::cout << "Rescaling weights for triangles..." << std::endl;
		weights = PD::getTriangleWeightsFromVertexWeights(m_rhsInterpolReusableWeights, m_triangles);
	}
	else if (g.getName() == "tetstrain" || g.getName() == "tetex") {
		std::cout << "Rescaling weights for tets..." << std::endl;
		weights = PD::getTetWeightsFromVertexWeights(m_rhsInterpolReusableWeights, m_tetrahedrons);
	}
	if (weights.hasNaN()) {
		std::cout << "Warning: NaN entries in remapped reusable weights!" << std::endl;
	}
	g.createBasisViaSkinningWeights(m_rhsInterpolBaseSize, m_positions, weights, true, m_triangles, m_tetrahedrons);
	g.initInterpolation(m_numVertices, samples, m_baseFunctionsTransposedSparse);
}

void ProjDynSimulator::setup() {
	std::cout << "Setting simulation..." << std::endl;

#ifndef EIGEN_DONT_PARALLELIZE
	Eigen::setNbThreads(PROJ_DYN_NUM_THREADS);
#endif

	m_precomputationStopWatch.startStopWatch();

	m_positionCorrections.setZero(m_positions.rows(), 3);

	// Collect constraints for building global system and interpolation subspaces for rhs interpolation
	// (if no rhs interpolation is used, we simply collect all constraints, otherwise we only
	// use constraints from the main group and treat the rest as additional constraints)
	std::vector< ProjDynConstraint* >* usedConstraints = &m_constraints;
	std::vector< ProjDynConstraint* > collectedConstraints;
	if (m_rhsInterpolation) {
		std::cout << "Collecting constraints for interpolation..." << std::endl;
		collectedConstraints.clear();
		for (ProjDynConstraint* c : m_bendingConstraints) {
			collectedConstraints.push_back(c);
		}
		for (ProjDynConstraint* c : m_strainConstraints) {
			collectedConstraints.push_back(c);
		}
		for (ProjDynConstraint* c : m_collisionConstraints) {
			collectedConstraints.push_back(c);
		}
		for (ProjDynConstraint* c : m_tetStrainConstraints) {
			collectedConstraints.push_back(c);
		}
		for (ProjDynConstraint* c : m_additionalConstraints) {
			collectedConstraints.push_back(c);
		}
		usedConstraints = &collectedConstraints;
	}

	// Here we create a preliminary sampling of elements which are used
	// to choose which constraints should be evaluated.
	// These will be overwritten if constraint groups are used
	// that suggest DEIM samples.
	if (m_rhsInterpolation) {
		std::cout << "Sampling constraints..." << std::endl;
		createConstraintSampling(m_numConstraintSamples);
	}

	/* If using subspaces, set up projection of full positions into the subspace.
	Initial subspace positions/velocities will be computed from full positions/velocities. */
	if (m_usingSubspaces) {
		std::cout << "Projecting positions, velocities and forces into the subspace... " << std::endl;
		// We need to project the current positions to the subspace, which will be
		// done by solving a  least squares problem since the subspace is not assumed
		// to be orthonormal.
		PDMatrix L = m_baseFunctionsTransposed * m_massMatrix * m_baseFunctions;
		m_reducedMassMatrix = (m_baseFunctionsTransposed * m_massMatrix * m_baseFunctions).sparseView();
		m_subspaceSolver.compute(L);

		m_positionsSubspace.resize(m_baseFunctions.cols(), 3);
		m_velocitiesSubspace.resize(m_baseFunctions.cols(), 3);

		projectToSubspace(m_positionsSubspace, m_positions);
		projectToSubspace(m_velocitiesSubspace, m_velocities);

		m_positions = m_baseFunctions * m_positionsSubspace;
		m_velocities = m_baseFunctions * m_velocitiesSubspace;

		// If PICFLIP is used, the initial positions of the mesh particles have
		// to be set now (by specifying the subspace coordinates of the tet-mesh)
		if (m_picflipsimulator) {
			m_picflipsimulator->setMeshPositions(m_positionsSubspace);
		}

		if (m_positions.hasNaN()) {
			std::cout << "Warning: subspace interpolation did not work!" << std::endl;
		}
	}

	// Sparse subspace basis needs to be available before the snapshot groups get initialized
	m_baseFunctionsSparse = m_baseFunctions.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF);
	m_baseFunctionsTransposedSparse = m_baseFunctionsTransposed.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF);

	// If using r.h.s. interpolation, build interpolation subspaces
	// and constraint sampling for each group.
	// The LHS matrix will also be built from these constraint interpolation
	// groups.
	// Set up interpolation groups and adapt the lhs side matrix for using them
	if (m_rhsInterpolation) {
		std::cout << "Initiating snapshot groups for constraints ... " << std::endl;
		m_snapshotGroups.clear();
		// Create snapshot interpolation groups
		if (!m_springConstraints.empty()) {
			m_snapshotGroups.push_back(RHSInterpolationGroup("spring", m_springConstraints, m_positions,
				m_vertexMasses, m_triangles, m_tetrahedrons, m_rhsRegularizationWeight));
		}
		if (!m_bendingConstraints.empty() && !m_flatBending) {
			m_snapshotGroups.push_back(RHSInterpolationGroup("bend", m_bendingConstraints, m_positions,
				m_vertexMasses, m_triangles, m_tetrahedrons, m_rhsRegularizationWeight));
		}
		if (!m_strainConstraints.empty()) {
			m_snapshotGroups.push_back(RHSInterpolationGroup("strain", m_strainConstraints, m_positions,
				m_vertexMasses, m_triangles, m_tetrahedrons, m_rhsRegularizationWeight));
		}
		if (!m_tetStrainConstraints.empty()) {
			m_snapshotGroups.push_back(RHSInterpolationGroup("tetstrain", m_tetStrainConstraints, m_positions,
				m_vertexMasses, m_triangles, m_tetrahedrons, m_rhsRegularizationWeight));
		}
		if (!m_tetExConstraints.empty()) {
			m_snapshotGroups.push_back(RHSInterpolationGroup("tetex", m_tetExConstraints, m_positions,
				m_vertexMasses, m_triangles, m_tetrahedrons, m_rhsRegularizationWeight));
		}

		// If snapshot groups are used (and not only snapshots are being recorded)
		// they need to be initialized
		if (m_rhsInterpolation) {
			PDMatrix hessian(0, 0);

			for (auto& g : m_snapshotGroups) {
				// Initialization of the group depends on the constraint that's being used
				if (g.getName() == "bend") {
					initRHSInterpolGroup(g, m_constraintVertexSamples, hessian);
				}
				else if (g.getName() == "spring") {
					initRHSInterpolGroup(g, m_constraintVertexSamples, hessian);
				}
				else if (g.getName() == "strain") {
					initRHSInterpolGroup(g, m_constraintTriSamples, hessian);
				}
				else if (g.getName() == "tetstrain" || g.getName() == "tetex") {
					initRHSInterpolGroup(g, m_constraintTetSamples, hessian);
				}
				else {
					std::cout << "ERROR: unknown rhs interpolation group: " << g.getName() << "!" << std::endl;
				}

				// Maintain list of constraints that have been sampled
				std::vector<ProjDynConstraint*>& sampledCons = g.getSampledConstraints();
				for (ProjDynConstraint* c : sampledCons) m_sampledConstraints.push_back(c);
			}

		}
	}

	// Check which hang constraints are on sampled vertices
	// and add those.
	if (m_rhsInterpolation) {
		for (PositionConstraint* c : m_hangConstraints) {
			if (std::find(m_constraintVertexSamples.begin(), m_constraintVertexSamples.end(), c->getMainVertexIndex()) != m_constraintVertexSamples.end()) {
				m_constraints.push_back(c);
				m_additionalConstraints.push_back(c);
				m_sampledConstraints.push_back(c);
			}
		}
	}
	else {
		for (PositionConstraint* c : m_hangConstraints) {
			m_constraints.push_back(c);
			m_additionalConstraints.push_back(c);
		}
	}

	/* Create the LHS matrix for the global system: */
	// 1) Compute the momentum part of the lhs and rhs matrices of the global step
	std::cout << "Preparing (and possibly reducing) momentum term of LHS matrix ..." << std::endl;
	m_lhsMatrix = m_massMatrix;
	m_lhsMatrix *= 1.f / (m_timeStep * m_timeStep);
	m_rhsMasses.setZero(m_numVertices);
	for (int v = 0; v < m_numVertices; v++) {
		m_rhsMasses(v) = m_vertexMasses(v) / (m_timeStep * m_timeStep);
	}
	if (m_usingSubspaces) {
		m_rhsFirstTermMatrixPre = (m_baseFunctionsTransposed * m_massMatrix * m_baseFunctions);
		m_rhsFirstTermMatrix = m_rhsFirstTermMatrixPre * (1. / (m_timeStep * m_timeStep));
		rhs2.setZero(m_baseFunctions.cols(), 3);
	}
	// 2) Compute the constraint part of the global step
	// If using rhs interpolation, let the constraint groups set up the constraint part of the LHS matrix
	if (m_rhsInterpolation) {
		if (m_usingSubspaces) {
			std::cout << "Building lhs matrix for global system (snapshots, subspace) ..." << std::endl;
			// Momentum term
			m_subspaceLHS_mom = m_baseFunctionsTransposed * m_lhsMatrix * m_baseFunctions * (m_timeStep * m_timeStep);
			PDMatrix eps(m_subspaceLHS_mom.rows(), m_subspaceLHS_mom.rows());
			eps.setIdentity();
			eps *= 1e-10;
			m_subspaceLHS_mom += eps;

			m_subspaceLHS_inner.setZero(m_baseFunctions.cols(), m_baseFunctions.cols());
			// Projection terms for each snapshot group
			for (auto& g : m_snapshotGroups) {
				m_subspaceLHS_inner += g.getLHSMatrixSubspace(m_baseFunctions, m_baseFunctionsTransposed);
			}
			// In case of flat bending, there is no bending constraint group but the bending terms
			// still need to be added to the lhs
			if (m_flatBending && !m_bendingConstraints.empty()) {
				for (auto c : m_bendingConstraints) {
					PDMatrix tmp = (m_baseFunctionsTransposed * c->getSelectionMatrixTransposed()) * (c->getSelectionMatrix() * m_baseFunctions);
					tmp *= c->getWeight();
					if (tmp.hasNaN()) {
						std::cout << "Error while constructing lhs..." << std::endl;
					}
					m_subspaceLHS_inner += tmp;
				}
			}
			// Additional constraints
			for (auto c : m_additionalConstraints) {
				PDMatrix tmp = (m_baseFunctionsTransposed * c->getSelectionMatrixTransposed()) * (c->getSelectionMatrix() * m_baseFunctions);
				tmp *= c->getWeight();
				m_subspaceLHS_inner += tmp;
			}
			m_lhsMatrixSampled = m_subspaceLHS_mom * (1. / (m_timeStep * m_timeStep)) + m_subspaceLHS_inner;
			m_denseSolver.compute(m_lhsMatrixSampled);

			PDMatrix epsD(m_lhsMatrixSampled.rows(), m_lhsMatrixSampled.rows());
			epsD.setIdentity();
			epsD *= 1e-12;
			while (m_denseSolver.info() != Eigen::Success && epsD(0, 0) < 1e-10) {
				std::cout << "Adding small diagonal entries (" << epsD(0, 0) << ")..." << std::endl;
				m_lhsMatrixSampled += epsD;
				epsD *= 2;
				m_denseSolver.compute(m_lhsMatrixSampled);
			}

			if (m_denseSolver.info() != Eigen::Success) {
				std::cout << "Warning: Factorization of LHS matrix for global system was not successful!" << std::endl;
			}
		}
		else {
			// TODO: full lhs when using snapshots
			std::cout << "ERROR: RHS interpolation only available when using subspaces!" << std::endl;
		}
	}

	// In case constraint sampling / rhs interpolation is not used
	// we set up the constraint part of the LHS matrix manually here
	if (!m_rhsInterpolation)
	{
		std::cout << "Building lhs full lhs matrix... " << std::endl;
		PDSparseMatrix conMat(m_lhsMatrix.rows(), m_lhsMatrix.cols());
		conMat.setZero();
		std::vector<Eigen::Triplet<PDScalar>> entries;
		for (auto& c : m_constraints) {
			PDSparseMatrixRM& selMat = c->getSelectionMatrix();
			for (int k = 0; k < selMat.outerSize(); ++k)
				for (PDSparseMatrixRM::InnerIterator it(selMat, k); it; ++it)
				{
					for (PDSparseMatrixRM::InnerIterator it2(selMat, k); it2; ++it2)
					{
						entries.push_back(Eigen::Triplet<PDScalar>(it.col(), it2.col(), it.value() * it2.value() * c->getWeight()));
					}
				}
		}
		conMat.setFromTriplets(entries.begin(), entries.end());
		if (m_usingSubspaces) { // Slow case: using position subspaces but no rhs interpolation
			std::cout << "Constraining it to the subspace..." << std::endl;
			m_subspaceLHS_mom = m_baseFunctionsTransposed * m_lhsMatrix * m_baseFunctions * (m_timeStep * m_timeStep);
			m_subspaceLHS_inner = m_baseFunctionsTransposed * conMat * m_baseFunctions;
			m_lhsMatrixSampled = m_subspaceLHS_mom * (1. / (m_timeStep * m_timeStep)) + m_subspaceLHS_inner;

			m_denseSolver.compute(m_lhsMatrixSampled);
			std::cout << "Size of sampled, dense lhs mat: " << m_lhsMatrixSampled.rows() << ", " << m_lhsMatrixSampled.cols() << std::endl;
		}
		else { // Full simulation
			m_lhsMatrix += conMat;
			m_lhsMatrix.prune(0, 1e-9f);
			int nnz = m_lhsMatrix.nonZeros();
			// Factorize lhs matrix
			StopWatch tmpWatch(10, 10);
			tmpWatch.startStopWatch();
			m_linearSolver.analyzePattern(m_lhsMatrix);
			m_linearSolver.factorize(m_lhsMatrix);
			tmpWatch.stopStopWatch();
			std::cout << "Factorization of the system matrix took " << tmpWatch.lastMeasurement() << " microseconds." << std::endl;
		}

		// After the lhs has been constructed, if flat bending is desired,
		// the bending constraints can now be thrown away!
		if (m_flatBending) {
			for (ProjDynConstraint* c : m_bendingConstraints) {
				auto const& bc = std::find(m_constraints.begin(), m_constraints.end(), c);
				if (bc != m_constraints.end()) {
					m_constraints.erase(bc);
				}
			}
		}
	}
	m_recomputeFactorization = false;

	// Now, all sampled constraints should have been initialized and the used vertices can be updated,
	// and the constraints can be updated to use this list
	if (m_rhsInterpolation) {
		std::cout << "Determining used vertices..." << std::endl;
		updateUsedVertices();
	}
	else if (m_usingSubspaces) {
		for (unsigned int v : m_samples) {
			m_additionalUsedVertices.push_back(v);
		}
		updateUsedVertices();
	}
	if (!m_usingSubspaces && !m_rhsInterpolation) {
		m_usedVertices.clear();
		for (unsigned int v = 0; v < m_numVertices; v++) {
			m_usedVertices.push_back(v);
		}
	}

	// Optional sparsification of matrices
	if (m_usingSubspaces && m_useSparseMatricesForSubspace) {
		std::cout << "Sparsifying matrices..." << std::endl;
		PDSparseMatrix lhsMatrixSampledSparse = m_lhsMatrixSampled.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF_HIGH_PREC);
		m_subspaceSystemSolverSparse.compute(lhsMatrixSampledSparse);
		if (m_subspaceSystemSolverSparse.info() != Eigen::Success) {
			std::cout << "Warning: Factorization of the sparse LHS matrix for the global step was not successful!" << std::endl;
			PDSparseMatrix eps(lhsMatrixSampledSparse.rows(), lhsMatrixSampledSparse.rows());
			eps.setIdentity();
			eps *= 1e-12;
			while (m_subspaceSystemSolverSparse.info() != Eigen::Success && eps.coeff(0, 0) < 1e-10) {
				std::cout << "Adding small diagonal entries (" << eps.coeff(0, 0) << ")..." << std::endl;
				lhsMatrixSampledSparse += eps;
				eps *= 2;
				m_subspaceSystemSolverSparse.compute(lhsMatrixSampledSparse);
			}
		}
		m_rhsFirstTermMatrixSparse = m_rhsFirstTermMatrix.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF_HIGH_PREC);
		updateParallelVUpdateBlocks();
	}

	m_collidedVerts = new bool[m_numVertices];
	for (int i = 0; i < m_numVertices; i++) m_collidedVerts[i] = false;

	m_isSetup = true;
	m_collisionCorrection = false;
	m_grippedVertices.clear();

	m_initialPos = m_positions;
	m_initialPosSub = m_positionsSubspace;

	// Precompute the weighted external forces that will appear on the rhs
	recomputeWeightedForces();

	m_precomputationStopWatch.stopStopWatch();

#ifndef EIGEN_DONT_PARALLELIZE
	Eigen::setNbThreads(1);
#endif
}

void ProjDynSimulator::resetPositions() {
	m_positions = m_initialPos;
	m_positionsSubspace = m_initialPosSub;
	m_velocities.setZero();
	m_velocitiesSubspace.setZero();
	m_collisionCorrection = false;
	m_positionCorrectionsUsedVs.setZero();
	if (m_picflipsimulator) {
		m_picflipsimulator->setFreeParticlePositions(m_particlesInitial);
		m_picflipsimulator->setMeshPositions(m_positionsSubspace);
		m_picflipsimulator->resetVelocities();
	}
	releaseGrip();
}

void PD::ProjDynSimulator::resetVelocities()
{
	m_velocities.setZero();
	m_velocitiesSubspace.setZero();
	m_velocitiesUsedVs.setZero();
	m_particlesV.setZero();
	if (m_picflipsimulator) {
		m_picflipsimulator->resetVelocities();
	}
}

/* Specify a list of vertices who should be constrained to temporarily fixed positions.
vInds should be a list of vertex indices into the vertex positions matrix.
NOTE THAT vInds MUST BE SORTED AND ONLY CONTAIN UNIQUE ENTRIES
gripPos is a k by 3 matrix, where k is the number of gripped vertices, containing
the positions of the gripped vertices */
void PD::ProjDynSimulator::setGrip(std::vector<unsigned int>& vInds, PDPositions gripPos)
{
  	m_grippedVertices = vInds;
	m_grippedVertexPos = gripPos;

	if (m_picflipsimulator) {
		// Bound constraints to the simulation grid
		PDMatrix& bds = m_picflipsimulator->getGridBounds();
		for (int i = 0; i < gripPos.rows(); i++) {
			for (int d = 0; d < 3; d++) {
				gripPos(i, d) = std::max(bds(0, d), std::min(bds(1, d), gripPos(i, d)));
			}
		}
	}
    

	if (m_useCenterConstraintGrip) {
		if (!m_gripCenterConstraint) {

			m_gripCenterConstraint = new CenterConstraint(m_numVertices, vInds, m_positions, m_gripWeight, true);
			addConstraintDynamic(m_gripCenterConstraint);
		}
		else {
			PDPositions c;
			c.setZero(1, 3);
			for (int i = 0; i < gripPos.rows(); i++) {
				c.row(0) += gripPos.row(i);
			}
			c /= (double)(gripPos.rows());
			m_gripCenterConstraint->setCenter(c);
		}
	}
	else {
		if (!m_gripPosConstraint) {
			m_gripPosConstraint = new PositionMultiConstraint(m_numVertices, vInds, m_positions, m_gripWeight);
			addConstraintDynamic(m_gripPosConstraint);
		}
		else {
			m_gripPosConstraint->setPos(gripPos);
		}
	}
}

void PD::ProjDynSimulator::releaseGrip()
{
	m_grippedVertices.clear();
	if (m_useCenterConstraintGrip) {
		if (m_gripCenterConstraint) {
			removeConstraintDynamic(m_gripCenterConstraint);
		}
		m_gripCenterConstraint = nullptr;
	}
	else {
		if (m_gripPosConstraint) {
			removeConstraintDynamic(m_gripPosConstraint);
		}
		m_gripPosConstraint = nullptr;

	}
}


/* List of all used vertices, i.e. vertices that appear in any sampled constraint
projection. Provides a map via usedVertices[i_sub] = i_full, where the indices
i_sub are used, for example, in set grip, and i_full are the corresponding vertices
of the full mesh.*/
std::vector<unsigned int>& PD::ProjDynSimulator::getUsedVertices()
{
	return m_usedVertices;
}

void PD::ProjDynSimulator::setGripType(bool centerGrip)
{
	m_useCenterConstraintGrip = centerGrip;
}


void ProjDynSimulator::step(int numIterations)
{
	if (numIterations > 0) m_numIterations = numIterations;

	m_totalStopWatch.startStopWatch();
	m_surroundingBlockStopWatch.startStopWatch();

	if (!m_isSetup) {
		std::cout << "Constraints or external forces have changed since last setup() call, so setup() will be called now!" << std::endl;
		setup();
	}

	PDPositions s;
	PDPositions oldPos, oldFullPos;

	//************************************
	// Compute s and handle collisions/user interaction
	//************************************
	if (m_usingSubspaces) {
		if (!m_rhsInterpolation) {
			oldFullPos = m_positions;
		}
		// Store previous positions for velocitiy computation at the end of the timestep
		oldPos = m_positionsSubspace;
		// If there has been a collision in the last step, handle repulsion and friction:
		if (m_collisionCorrection) {
			// Get actual velocities for used vertices
			updatePositionsSampling(m_velocitiesUsedVs, m_velocitiesSubspace, true);
			// Remove tangential movement and add repulsion movement from collided vertices
			PROJ_DYN_PARALLEL_FOR
				for (int v = 0; v < m_velocitiesUsedVs.rows(); v++) {
					if (m_positionCorrectionsUsedVs.row(v).norm() > 1e-12) {
						PDVector tangentialV = m_velocitiesUsedVs.row(v) - m_velocitiesUsedVs.row(v).dot(m_positionCorrectionsUsedVs.row(v)) * m_positionCorrectionsUsedVs.row(v);
						tangentialV *= (1. - m_frictionCoeff);
						tangentialV += m_positionCorrectionsUsedVs.row(v) * m_repulsionCoeff;
						m_velocitiesUsedVs.row(v) = tangentialV;
					}
				}
			// Project full velocities back to subspace via interpolation of the velocities on used vertices
			PROJ_DYN_PARALLEL_FOR
				for (int d = 0; d < 3; d++) {
					if (m_useSparseMatricesForSubspace) {
						m_velocitiesSubspace.col(d) = m_usedVertexInterpolatorSparse.solve(m_usedVertexInterpolatorRHSMatrixSparse * m_velocitiesUsedVs.col(d));
					}
					else {
						m_velocitiesSubspace.col(d) = m_usedVertexInterpolator.solve(m_usedVertexInterpolatorRHSMatrix * m_velocitiesUsedVs.col(d));
					}
				}
		}
		PDScalar blowupFac = (10. - m_blowupStrength) / 9.;
		// Compute s:
		s = m_positionsSubspace + m_timeStep * m_velocitiesSubspace + m_fExtWeightedSubspace + blowupFac * m_fGravWeightedSubspace;
		if (m_rayleighDampingAlpha > 0) {
			s = s - m_timeStep * m_rayleighDampingAlpha * m_velocitiesSubspace;
		}
		// s is also the initial guess for the updated positions
		m_positionsSubspace = s;
		// Compute vertex positions on vertices involved in the computation of sampled constraints
		updatePositionsSampling(m_positionsUsedVs, m_positionsSubspace, true);
		if (!m_rhsInterpolation) {
			updatePositionsSampling(m_positions, m_positionsSubspace, false);
		}
		// Correct vertex positions of gripped and collided vertices
		if ((!m_picflipsimulator) || m_didSkipGridStep) handleCollisionsUsedVs(s, true);
	}
	else { // Simplified computations for full simulations
		oldPos = m_positions;
		PDScalar blowupFac = (10. - m_blowupStrength) / 9.;
		s = m_positions + m_timeStep * m_velocities + m_fExtWeighted + blowupFac * m_fGravWeighted;
		if (m_rayleighDampingAlpha > 0) {
			s = s - m_timeStep * m_rayleighDampingAlpha * m_velocities;
		}
		PROJ_DYN_PARALLEL_FOR
			for (int v = 0; v < m_positions.rows(); v++) {
				resolveCollision(v, s, m_positionCorrections);
			}
		// Change position of gripped vertices
		if (m_grippedVertices.size() > 0) {
			for (unsigned int i = 0; i < m_grippedVertices.size(); i++) {
				s.row(m_grippedVertices[i]) = m_grippedVertexPos.row(i);
			}
		}
		m_positions = s;
	}

	// Some additional initializations/updates
	if (m_usingSubspaces && m_constraintSamplesChanged) {
		updateUsedVertices();
		if (m_rhsInterpolation) {
			updatePositionsSampling(m_positionsUsedVs, m_positionsSubspace, true);
		}
		else {
			updatePositionsSampling(m_positions, m_positionsSubspace, false);
		}
	}
	std::vector< ProjDynConstraint* >* usedConstraints = &m_constraints;
	if (m_rhsInterpolation) {
		usedConstraints = &m_sampledConstraints;
	}
	int numConstraints = usedConstraints->size();
	std::vector< PDPositions > currentAuxilaries(numConstraints);
	int c_chunk = numConstraints / (omp_get_num_threads() * 100);
	int v_chunk = m_numVertices / (omp_get_num_threads() * 100);
	m_surroundingBlockStopWatch.pauseStopWatch();


	m_elasticityStopWatch.startStopWatch();
	//************************************
	// Local global loop
	//************************************
	for (int i = 0; i < m_numIterations; i++) {

		//************************************
		// Local step: Constraint projections
		//************************************
		m_localStepStopWatch.startStopWatch();
		if (m_rhsInterpolation) { // Approximation via fitting method
			m_localStepOnlyProjectStopWatch.startStopWatch();
			if (m_usingSubspaces) {
				rhs2.setZero();
				for (auto& g : m_snapshotGroups)
					g.approximateRHS(m_positionsUsedVs, rhs2, m_collidedVerts);
				addAdditionalConstraints(m_positionsUsedVs, rhs2, m_collidedVerts);
			}
			else {
				m_rhs.setZero();
				for (auto& g : m_snapshotGroups)
					g.approximateRHS(m_positions, m_rhs, m_collidedVerts);
				addAdditionalConstraints(m_positions, m_rhs, m_collidedVerts);
			}
			m_localStepOnlyProjectStopWatch.stopStopWatch();


			m_localStepRestStopWatch.startStopWatch();
		}
		else { // Full local step
			   // Compute projections
			m_localStepOnlyProjectStopWatch.startStopWatch();
#pragma omp parallel for num_threads(PROJ_DYN_NUM_THREADS)
			for (int ind = 0; ind < numConstraints; ind++) {
				ProjDynConstraint* c = usedConstraints->at(ind);
				int didCollide = -1;
				currentAuxilaries[ind] = c->getP(m_positions, didCollide);
				if (didCollide >= 0) m_collidedVerts[didCollide] = true;
			}
			m_localStepOnlyProjectStopWatch.stopStopWatch();
			m_localStepRestStopWatch.startStopWatch();
			// Sum projections
			m_rhs.setZero();
			m_constraintSummationStopWatch.startStopWatch();
			int d = 0;
			PROJ_DYN_PARALLEL_FOR
				for (d = 0; d < 3; d++) {
					for (int mind = 0; mind < numConstraints; mind++) {
						PDScalar curWeight = usedConstraints->at(mind)->getWeight();
						fastDensePlusSparseTimesDenseCol(m_rhs, usedConstraints->at(mind)->getSelectionMatrixTransposed(), currentAuxilaries[mind], d, curWeight);
					}
				}
			m_constraintSummationStopWatch.stopStopWatch();

		}

		m_momentumStopWatch.startStopWatch();
		// Add the term from the conservation of momentum 
		if (m_usingSubspaces) {
			// If using subspaces, transform the r.h.s to the subspace
			// (which is already the case when using rhs interpolation)
			if (!m_rhsInterpolation) {
				if (m_useSparseMatricesForSubspace) {
					rhs2 = m_baseFunctionsTransposedSparse * m_rhs;
				}
				else {
					rhs2 = m_baseFunctionsTransposed * m_rhs;
				}
			}

			// Now add the term from the conservation of momentum, where s is already in the subspace,
			// and the product of the subspace matrix and M/h^2 has been evaluated already.
			PROJ_DYN_PARALLEL_FOR
				for (int d = 0; d < 3; d++) {
					if (m_useSparseMatricesForSubspace) {
						rhs2.col(d) += m_rhsFirstTermMatrixSparse * s.col(d);
					}
					else {
						rhs2.col(d) += m_rhsFirstTermMatrix * s.col(d);
					}
				}

		}
		if (!m_usingSubspaces) {
			// If no subspaces are used, the full rhs from the constraints just needs to be added
			// to the conservation of momentum terms
			int v = 0;
			PROJ_DYN_PARALLEL_FOR
				for (v = 0; v < m_numVertices; v++) {
					for (int d = 0; d < 3; d++) {
						m_rhs(v, d) += m_rhsMasses(v) * s(v, d);
					}
				}
		}

		m_momentumStopWatch.stopStopWatch();
		m_localStepRestStopWatch.stopStopWatch();
		m_localStepStopWatch.stopStopWatch();

		//************************************
		// Global step: Solve the linear system with fixed constraint projections
		//************************************
		m_globalStepStopWatch.startStopWatch();
		// Solve, for x, y and z in parallel
		int d = 0;
		PROJ_DYN_PARALLEL_FOR
			for (d = 0; d < 3; d++) {
				if (m_usingSubspaces) {
					// Use sparse solver, if desired. For SPH constraints, always use dense solver,
					// since updating a sparse LHS matrix would take too long.
					if (m_useSparseMatricesForSubspace) {
						m_positionsSubspace.col(d) = m_subspaceSystemSolverSparse.solve(rhs2.col(d));
					}
					else {
						m_positionsSubspace.col(d) = m_denseSolver.solve(rhs2.col(d));
					}
				}
				if (!m_usingSubspaces) {
					m_positions.col(d) = m_linearSolver.solve(m_rhs.col(d));
				}
			}
		m_globalStepStopWatch.stopStopWatch();


		// Partially update vertex positions (vertices involved in the evaluation of constraints)
		// Does not need to be done in the last iteration, since vertex positions are fully
		// updated after.
		m_updatingVPosStopWatch.startStopWatch();
		if (m_usingSubspaces && !(i == numIterations - 1)) {
			if (m_rhsInterpolation) {
				updatePositionsSampling(m_positionsUsedVs, m_positionsSubspace, true);
			}
			else {
				updatePositionsSampling(m_positions, m_positionsSubspace, false);
			}
		}
		m_updatingVPosStopWatch.stopStopWatch();

} // End of local global loop
	m_elasticityStopWatch.stopStopWatch();

	if (m_rhsInterpolation && m_collisionFreeDraw) {
		updatePositionsSampling(m_positionsUsedVs, m_positionsSubspace, true);
		handleCollisionsUsedVs(s, false);
	}
	else if (m_rhsInterpolation) {
		s = m_positionsSubspace;
	}

	//************************************
	// Incompressiblity and advection step
	//************************************
	if (!m_skipGridStep) {
        if (m_picflipsimulator) {
			m_picflipStopWatch.startStopWatch();
			if (m_didSkipGridStep) {
				m_picflipsimulator->setMeshPositions(oldPos);
			}

            // Update velocities of the particles belonging to the mesh
			m_picflipsimulator->setTimeStep(m_timeStep);
			m_particleTransferStopWatch.startStopWatch();
			m_picflipsimulator->setMeshVelocities(s);
			m_particleTransferStopWatch.pauseStopWatch();
			// Time step of fluid dynamics

			for (int i = 0; i < m_pfStepsPerFrame; i++) {
				m_picflipsimulator->step(m_timeStep / (float)m_pfStepsPerFrame, m_gravity, m_flipness, m_maxDensity, m_densityCorrection,
					m_fluidDensity, m_maxSSFluid, m_maxSSSolid, m_pfFriction, m_solidInfluence, false, m_jacobiIts,
					m_colorDeviation, m_maxDCorrection, m_skipIncompressibility);
			}

			// Updte open GL buffer for updated particle positions to display
			m_picflipsimulator->updateGLBuffer();
			// Update mesh as projection to the mesh's particle positions
			m_particleTransferStopWatch.unpauseStopWatch();
			m_picflipsimulator->projectParticlesToSubspace(s);
			m_picflipsimulator->setMeshPositions(s);
			m_particleTransferStopWatch.stopStopWatch();
			if (s.hasNaN()) {
				resetPositions();
				s = m_positionsSubspace;
			}

            // Update subspace positions
			m_positionsSubspace = s;
			m_picflipStopWatch.stopStopWatch();
		}
	}
	m_didSkipGridStep = m_skipGridStep;

	m_positionsUpdated = false;
	m_surroundingBlockStopWatch.unpauseStopWatch();

	// Evaluate full positions once so that outside methods
	// have access to them (for e.g. displaying the mesh)
	m_fullUpdateStopWatch.startStopWatch();
	if (m_usingSubspaces) {
		if (m_useSparseMatricesForSubspace) {
#ifdef PROJ_DYN_USE_CUBLAS_REST
			if (m_vPosGPUUpdate) {
				// The vertex position update is done on the GPU
				PDScalar one = 1.;
				for (int d = 0; d < 3; d++) {
					m_vPosGPUUpdate->mult(s.data() + (d * s.rows()), nullptr, one, false, d, m_numOuterVertices);
				}
			}
			else
#endif
				if (!m_parallelVUpdate) {
					// The vertex update is still done in parallel, but only for the three columns
					PROJ_DYN_PARALLEL_FOR
						for (int d = 0; d < 3; d++) {
							m_positions.col(d) = m_baseFunctionsSparse * s.col(d);
						}
					m_positionsUpdated = true;
				}
				else {
					// The vertex update is done in finer granularity, by splitting the vertex
					// positions into blocks of n rows and building the vector from this
					int blockSize = m_baseFunctionsSparseBlocks[0].rows();
					int numBlocks = std::ceil((float)m_positions.rows() / (float)blockSize);
					PROJ_DYN_PARALLEL_FOR
						for (int b = 0; b < numBlocks; b++) {
							int curSize = blockSize;
							if (b == numBlocks - 1) {
								curSize = blockSize - (numBlocks * blockSize - m_positions.rows());
							}
							m_positions.block(b*blockSize, 0, curSize, 3) = m_baseFunctionsSparseBlocks[b] * s;
						}
					m_positionsUpdated = true;
				}
		}
		else {
			PROJ_DYN_PARALLEL_FOR
				for (int d = 0; d < 3; d++) {
					m_positions.col(d) = m_baseFunctions * s.col(d);
				}
		}
	}
	m_fullUpdateStopWatch.stopStopWatch();

	// Update Velocities
	if (m_usingSubspaces) {
		if (!m_rhsInterpolation) {
			// Update full velocities
			m_velocities = (m_positions - oldFullPos) / m_timeStep;
		}
		// Update subspace velocities as well
		m_velocitiesSubspace = (m_positionsSubspace - oldPos) / m_timeStep;
	}
	else {
		m_velocities = (m_positions - oldPos) / m_timeStep;
	}

	m_frameCount++;

	std::vector<unsigned int>* usedVerts = &m_allVerts;
	if (m_rhsInterpolation) {
		usedVerts = &m_constraintVertexSamples;
	}

	m_surroundingBlockStopWatch.stopStopWatch();
	m_totalStopWatch.stopStopWatch();
}

PD::CollisionObject::CollisionObject()
	:
	m_type(-1),
	m_s1(0),
	m_s2(0),
	m_s3(0),
	m_pos(0, 0, 0)
{
}

PD::CollisionObject::CollisionObject(int type, PD3dVector& pos, PDScalar s1, PDScalar s2, PDScalar s3)
	:
	m_type(type),
	m_s1(s1),
	m_s2(s2),
	m_s3(s3),
	m_pos(pos)
{
}

bool PD::CollisionObject::resolveCollision(PD3dVector& newPos)
{
	PD3dVector relPos;
	switch (m_type) {
	case CollisionType::sphere:
		if ((newPos - m_pos).norm() < m_s1) {
			PD3dVector dir = (newPos - m_pos);
			dir.normalize();
			dir *= m_s1;
			newPos = m_pos + dir;
			return true;
		}
		break;
	case CollisionType::floor:
		if (newPos(1) < m_s1) {
			newPos(1) = m_s1;
			return true;
		}
		break;
	case CollisionType::block:
		relPos = newPos - m_pos;
		if (relPos(0) > 0 && relPos(0) < m_s1) {
			if (relPos(1) > 0 && relPos(1) < m_s2) {
				if (relPos(2) > 0 && relPos(2) < m_s3) {

					PDScalar corX = 0, corY = 0, corZ = 0;
					if (relPos(0) < m_s1 - relPos(0)) {
						corX = -relPos(0);
					}
					else {
						corX = m_s1 - relPos(0);
					}

					if (relPos(1) < m_s2 - relPos(1)) {
						corY = -relPos(1);
					}
					else {
						corY = m_s2 - relPos(1);
					}

					if (relPos(2) < m_s3 - relPos(2)) {
						corZ = -relPos(2);
					}
					else {
						corZ = m_s3 - relPos(2);
					}

					if (std::abs(corX) <= std::abs(corY) && std::abs(corX) <= std::abs(corZ)) {
						newPos(0) += corX;
					}
					else if (std::abs(corY) <= std::abs(corX) && std::abs(corY) <= std::abs(corZ)) {
						newPos(1) += corY;
					}
					else if (std::abs(corZ) < std::abs(corX) && std::abs(corZ) < std::abs(corY)) {
						newPos(2) += corZ;
					}

					return true;
				}
			}
		}
		break;
	default:
		return false;
	}
	return false;
}

void PD::ProjDynSimulator::addCollisionsFromFile(std::string fileName)
{
	// Read collisions file
	std::ifstream nodeFile;
	nodeFile.open(fileName);
	if (nodeFile.is_open()) {
		std::cout << "Reading and adding collision objects from the " << fileName << "..." << std::endl;
		std::string line;
		while (std::getline(nodeFile, line))
		{
			std::istringstream iss(line);
			char typeC;
			PDScalar x = 0, y = 0, z = 0;
			PDScalar s1 = 0, s2 = 0, s3 = 0;
			iss >> typeC >> x >> y >> z >> s1 >> s2 >> s3;
			int type = -1;
			switch (typeC) {
			case 's':
				type = CollisionType::sphere;
				break;
			case 'f':
				type = CollisionType::floor;
				break;
			case 'b':
				type = CollisionType::block;
				break;
			}
			PD3dVector pos = PD3dVector(x, y, z);
			CollisionObject col(type, pos, s1, s2, s3);
			m_collisionObjects.push_back(col);
		}
	}
}

void PD::ProjDynSimulator::setEnforceCollisionFreeDraw(bool enable)
{
	m_collisionFreeDraw = enable;
}

void PD::ProjDynSimulator::setStiffnessFactor(PDScalar w)
{
	for (auto& g : m_snapshotGroups) {
		g.setWeightFactor(w);
	}

	m_stiffnessFactor = w;
	refreshLHS();
}

void PD::ProjDynSimulator::handleCollisionsUsedVs(PDPositions& s, bool update)
{
	// Detect and resolve collisions on evaluated vertices
	m_collisionCorrection = false;
	PROJ_DYN_PARALLEL_FOR
		for (int v = 0; v < m_positionsUsedVs.rows(); v++) {
			resolveCollision(v, m_positionsUsedVs, m_positionCorrectionsUsedVs);
		}

	// If collisions were resolved, update s in the subspace via interpolation of the corrected vertices
	if (m_collisionCorrection || m_grippedVertices.size() > 0) {
		PROJ_DYN_PARALLEL_FOR
			for (int d = 0; d < 3; d++) {
				if (m_useSparseMatricesForSubspace) {
					s.col(d) = m_usedVertexInterpolatorSparse.solve(m_usedVertexInterpolatorRHSMatrixSparse * m_positionsUsedVs.col(d));
				}
				else {
					s.col(d) = m_usedVertexInterpolator.solve(m_usedVertexInterpolatorRHSMatrix * m_positionsUsedVs.col(d));
				}
			}
		if (update) m_positionsSubspace = s;
		updatePositionsSampling(m_positionsUsedVs, m_positionsSubspace, true);
	}
}

/* If the values m_timeStep or m_stiffnessFactor have changed, this method recomputes the matrices involved and
and updates their factorizations. */
void PD::ProjDynSimulator::refreshLHS()
{
	if (m_rhsInterpolation) {
		m_lhsMatrixSampled = m_subspaceLHS_mom * (1. / (m_timeStep * m_timeStep)) + m_stiffnessFactor * m_subspaceLHS_inner;
		if (!m_useSparseMatricesForSubspace) {
			m_denseSolver.compute(m_lhsMatrixSampled);
		}
		else {
			PDSparseMatrix lhsMatrixSampledSparse = m_lhsMatrixSampled.sparseView(0, PROJ_DYN_SPARSITY_CUTOFF_HIGH_PREC);
			m_subspaceSystemSolverSparse.compute(lhsMatrixSampledSparse);
		}
	}
}

void PD::errorExit(std::string msg)
{
	std::cout << "Error: " << msg << std::endl;
	system("pause");
	std::exit;
}
