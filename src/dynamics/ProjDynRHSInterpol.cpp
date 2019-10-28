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
#include <math.h>

#include "dynamics/ProjDynRHSInterpol.h"
#include "dynamics/ProjDynConstraints.h"
#include "dynamics/ProjDynSimulator.h"
#include "dynamics/ProjDynUtil.h"
#include <Eigen/Eigenvalues>

PDMatrix PD::RHSInterpolationGroup::snapshotPCA(PDMatrix& Y, PDVector& masses, unsigned int size)
{
	/* PCA via the method of snapshots*/

	std::cout << "Performing snapshot PCA..." << std::endl;

	PDMatrix base(Y.rows(), size + 1);

	std::cout << "	Removing column wise mean..." << std::endl;

	// Remove column wise mean
	PROJ_DYN_PARALLEL_FOR
		for (int j = 0; j < Y.cols(); j++) {
			PDScalar mean = Y.col(j).mean();
			for (unsigned int i = 0; i < Y.rows(); i++) {
				Y(i, j) -= mean;
			}
		}

	// We solve the SVD by solving an eigenvalue problem on the smaller matrix A = Y^T M Y
	// and converting the eigenvectors to that problem back to v = Y u
	std::cout << "	Computing Y^T M Y..." << std::endl;
	PDMatrix A;
#ifdef PROJ_DYN_USE_CUBLAS_IN_PRE
	std::cout << "		uploading Y to GPU..." << std::endl;
	CUDAMatrixVectorMultiplier* yMulti = new CUDAMatrixVectorMultiplier(Y, masses);
	std::cout << "		computing Y^T (M Y) on GPU..." << std::endl;
	A.resize(Y.cols(), Y.cols());
	for (unsigned int i = 0; i < Y.cols(); i++) {
		PDScalar one = 1;
		yMulti->mult(Y.data() + (i * Y.rows()), A.data() + (i * A.rows()), one, true);
	}
#else
	PDMatrix Y_T = Y.transpose();
	PROJ_DYN_PARALLEL_FOR
		for (int j = 0; j < Y.cols(); j++) {
			Y_T.col(j) *= masses(j);
		}
	A.resize(Y.cols(), Y.cols());
	PROJ_DYN_PARALLEL_FOR
		for (int j = 0; j < Y.cols(); j++) {
			A.col(j) = Y_T * Y.col(j);
		}
#endif
	std::cout << "	Computing eigenvectors v of Y^T M Y..." << std::endl;
	Eigen::EigenSolver< PDMatrix > eSolver;
	eSolver.compute(A, true);
	Eigen::Matrix< std::complex< double >, -1, -1> eVecsC = eSolver.eigenvectors();
	Eigen::Matrix< std::complex< double >, -1, 1> eValsC = eSolver.eigenvalues();
	std::cout << "	Extracting PCA vectors as Y v..." << std::endl;
	PDScalar currentEVal = -1;
	std::vector<unsigned int> eVecInds;
	for (int v = 0; v < size; v++) {
		PDScalar currentLargest = 0;
		int indOfLargest = -1;
		for (int j = 0; j < eValsC.rows(); j++) {
			if (eValsC(j).real() > currentLargest && (currentEVal < 0 || eValsC(j).real() < currentEVal)) {
				currentLargest = eValsC(j).real();
				indOfLargest = v;
			}
		}
		if (indOfLargest >= 0) {
			eVecInds.push_back(indOfLargest);
			currentEVal = eValsC(indOfLargest).real();
			//std::cout << eValsC(indOfLargest).real() << std::endl;
		}
		else {
			std::cout << "Could only find " << (v - 1) << " eigenvectors with strictly positive eigenvalues during snapshot PCA ..." << std::endl;
			base.conservativeResize(base.rows(), std::max(1, v - 10));
			break;
		}
	}

#ifdef PROJ_DYN_USE_CUBLAS_IN_PRE
	for (int v = 0; v < std::min((int)base.cols(), (int)eVecInds.size()); v++) {
		PDScalar weight = (1. / std::sqrt(eValsC(eVecInds[v]).real()));
		PDVector eVec = eVecsC.col(eVecInds[v]).real();
		yMulti->mult(eVec.data(), base.data() + (v * base.rows()), weight, false);
	}
	delete yMulti;
#else
	PROJ_DYN_PARALLEL_FOR
		for (int v = 0; v < std::min((int)base.cols(), (int)eVecInds.size()); v++) {
			PDVector eVec = eVecsC.col(eVecInds[v]).real();
			base.col(v) = Y * eVec;
			PDScalar weight = (1. / std::sqrt(eValsC(eVecInds[v]).real()));
			base.col(v) *= weight;
		}
#endif

	if (base.hasNaN()) {
		std::cout << "Warning: base contains NaN values!" << std::endl;
	}

	// Make sure to include global translations since we removed column wise mean
	base.col(base.cols() - 1).setConstant(1.);

	std::cout << "	Done." << std::endl;

	return base;
}


PD::RHSInterpolationGroup::RHSInterpolationGroup(std::string groupName, std::vector<ProjDynConstraint*>& constraints, PDPositions& restPositions,
	PDVector& vertexMasses, PDTriangles& tris, PDTets& tets, PDScalar regularizationWeight)
	:
	m_auxiliarySize(0),
	m_basis(0, 0),
	m_interpolReady(false),
	m_basisReady(false),
	m_groupName(groupName),
	m_regularizationWeight(regularizationWeight),
	m_restPositionFull(restPositions)
{
	for (auto c : constraints) {
		//ProjDynConstraint* cc = c->copy();
		m_constraints.push_back(c);
	}

	m_isTetExampleGroup = (m_groupName == "tetex");

	if (m_constraints.size() >= 1) {
		int dummy = -1;
		m_auxiliarySize = m_constraints.at(0)->getP(restPositions, dummy).rows();

		// Compute the mass matrix for the unassembled auxiliaries
		computeMassMatrix(vertexMasses, tris, tets);
	}
}

PD::RHSInterpolationGroup::RHSInterpolationGroup(std::string groupName, unsigned int auxiliarySize)
	:
	m_groupName(groupName),
	m_auxiliarySize(auxiliarySize),
	m_basis(0, 0),
	m_selectionMatrix(0, 0),
	m_interpolReady(false),
	m_basisReady(false),
	m_regularizationWeight(0.),
	m_restPositionFull(0, 3)
{
}

void PD::RHSInterpolationGroup::computeMassMatrix(PDVector& vertexMasses, PDTriangles& tris, PDTets& tets) {
	// Compute the mass matrix for the unassembled auxiliaries
	m_massMatrix.resize(m_constraints.size() * m_auxiliarySize, m_constraints.size() * m_auxiliarySize);
	m_massMatrixDiag.resize(m_constraints.size() * m_auxiliarySize);
	int i = 0;
	for (auto& c : m_constraints) {
		double weight = 1.;
		if (!NO_MASSES_IN_PCA) {
			if (c->getMainVertexIndex() > 0) {
				weight = vertexMasses(c->getMainVertexIndex());
			}
			else if (c->getMainTriangleIndex() > 0) {
				weight = 0;
				for (int j = 0; j < 3; j++) {
					weight += vertexMasses(tris(c->getMainTriangleIndex(), j));
				}
			}
			else if (c->getMainTetIndex() > 0) {
				weight = 0;
				for (int j = 0; j < 4; j++) {
					weight += vertexMasses(tets(c->getMainTetIndex(), j));
				}
			}
		}
		for (int j = 0; j < m_auxiliarySize; j++) {
			m_massMatrix.insert(i * m_auxiliarySize + j, i * m_auxiliarySize + j) = weight;
			m_massMatrixDiag(i * m_auxiliarySize + j) = weight;
		}
		i++;
	}
}

void PD::RHSInterpolationGroup::createBasisViaSkinningWeights(unsigned int size, PDPositions& restPos, PDMatrix& skinningWeights, bool usePCA,
	PDTriangles& tris, PDTets &tets)
{
	std::cout << "Creating interpolation basis via skinning construction for " << getName() << "... " << std::endl;
	std::cout << "	Computing rest state auxiliary variables..." << std::endl;
	PDSparseMatrix assMat = PD::getAssemblyMatrix(m_constraints, true, NO_WEIGHTS_IN_CONSTRUCTION);
	for (int k = 0; k < assMat.outerSize(); ++k) {
		for (PDSparseMatrix::InnerIterator it(assMat, k); it; ++it) {
			if (isnan(it.value())) {
				std::cout << "	WARNING: assembly matrix to create skinning space contains NaN values..." << std::endl;
				break;
			}
		}
	}
	PDPositions restStateAuxils = assMat.transpose() * restPos;
	if (skinningWeights.hasNaN()) {
		std::cout << "	WARNING: weights to create skinning space contain NaN values..." << std::endl;
	}
	if (restPos.hasNaN()) {
		std::cout << "	WARNING: positions to create skinning space contain NaN values..." << std::endl;
	}
	if (restStateAuxils.hasNaN()) {
		std::cout << "	WARNING: constraint projections to create skinning space contain NaN values..." << std::endl;
	}
	std::cout << "	Computing skinning space..." << std::endl;
	PDMatrix Y = PD::createSkinningSpace(restStateAuxils, skinningWeights, &m_constraints, m_auxiliarySize, &tris, &tets);

	if (!usePCA) {
		m_basis = Y;
	}
	else {
		m_basis = RHSInterpolationGroup::snapshotPCA(Y, m_massMatrixDiag, size);
	}
	m_basisReady = true;
	m_interpolReady = false;
}

void PD::RHSInterpolationGroup::setBasis(PDMatrix & base)
{
	m_basis = base;

	m_basisReady = true;
	m_interpolReady = false;

}

void PD::RHSInterpolationGroup::initInterpolation(unsigned int numVertices, std::vector<unsigned int>& sampledElements,
	PDSparseMatrix const& positionSubspaceT)
{
	std::cout << "Finalizing rhs interpolation for " << getName() << "... " << std::endl;
	std::cout << "	Computing assembly, selection and weight matrix..." << std::endl;

	std::vector<unsigned int> sampledElsCopy = sampledElements;

	// Create assembly matrix 
	m_assemblyMatrix = PD::getAssemblyMatrix(m_constraints, true, NO_WEIGHTS_IN_CONSTRUCTION);

	// Create selection and weights matrix
	std::vector<Eigen::Triplet<PDScalar>> selectedInds;
	m_weightMatrix.resize(m_constraints.size() * m_auxiliarySize, m_constraints.size() * m_auxiliarySize);
	PDVector weightVector(m_constraints.size() * m_auxiliarySize);
	weightVector.setConstant(1.);
	for (unsigned int i = 0; i < m_constraints.size(); i++) {
		ProjDynConstraint* c = m_constraints[i];
		unsigned int constraintElement = -1;
		if (c->getMainVertexIndex() >= 0) {
			constraintElement = c->getMainVertexIndex();
		}
		else if (c->getMainTriangleIndex() >= 0) {
			constraintElement = c->getMainTriangleIndex();
		}
		else if (c->getMainTetIndex() >= 0) {
			constraintElement = c->getMainTetIndex();
		}
		auto const& it = std::find(sampledElsCopy.begin(), sampledElsCopy.end(), constraintElement);
		if (m_sampledConstraints.size() < sampledElements.size() && it != sampledElsCopy.end()) {
			sampledElsCopy.erase(it);
			m_sampledConstraints.push_back(c->copy());
			int ind = m_sampledConstraints.size() - 1;
			for (int d = 0; d < m_auxiliarySize; d++) {
				selectedInds.push_back(Eigen::Triplet<PDScalar>(ind * m_auxiliarySize + d, i * m_auxiliarySize + d, 1.));
			}
		}
		for (int j = 0; j < m_auxiliarySize; j++) {
			if (NO_WEIGHTS_IN_CONSTRUCTION) {
				m_weightMatrix.insert(i * m_auxiliarySize + j, i * m_auxiliarySize + j) = c->getWeight();
				weightVector(i * m_auxiliarySize + j) = c->getWeight();
			}
			else {
				// Weights are included in the interpolation problem and snapshot basis
				m_weightMatrix.insert(i * m_auxiliarySize + j, i * m_auxiliarySize + j) = 1.;
			}
		}
	}
	m_selectionMatrix = PDSparseMatrix(sampledElements.size() * m_auxiliarySize, m_constraints.size() * m_auxiliarySize);
	m_selectionMatrix.setZero();
	m_selectionMatrix.setFromTriplets(selectedInds.begin(), selectedInds.end());
	// If we couldn't find as many constraint samples as we have sampled
	// elements, we overestimated the number of rows in the selection matrix
	// and need to cut off the rest.
	if (m_sampledConstraints.size() < sampledElements.size()) {
		m_selectionMatrix.conservativeResize(m_sampledConstraints.size() * m_auxiliarySize, m_selectionMatrix.cols());
	}
	if (m_selectionMatrix.rows() < m_basis.cols() - MIN_OVERSAMPLING) {
		std::cout << "Warning: not enough sampled constraints (" << m_sampledConstraints.size()
			<< ") for this rhs interpolation base (" << m_basis.cols() << ")." << std::endl;
		std::cout << "	Cutting off last vectors in base." << std::endl;
		m_basis.conservativeResize(m_basis.rows(), m_selectionMatrix.rows() - MIN_OVERSAMPLING);
	}

	// Compute lhs and rhs matrices and solver for the inteprolation problem
	std::cout << "	Computing lhs and rhs matrices for linear interpolation system and factorizing..." << std::endl;
	m_interpolRHSMatrix = m_basis.transpose() * m_selectionMatrix.transpose();
	PDMatrix lhsMat;
	if (m_regularizationWeight > 0) {
		lhsMat = m_basis.transpose() * ((m_selectionMatrix.transpose() * m_selectionMatrix) + m_regularizationWeight * m_massMatrix) * m_basis;
	}
	else {
		lhsMat = m_basis.transpose() * ((m_selectionMatrix.transpose() * m_selectionMatrix)) * m_basis;
	}
	m_interpolSolver.compute(lhsMat);
	m_tempSolution.resize(lhsMat.rows(), 3);
	m_solutionLastFrame.resize(lhsMat.rows(), 3);
	m_tempAuxils.resize(m_selectionMatrix.rows(), 3);
	m_tempRHS.resize(lhsMat.rows(), 3);

	// If regularization is used we have to get some initial value for m_solutionLastFrame
	// For this we have to evaluate the auxilaries for the rest state and set up a projection
	// problem, which returns the best subspace approximation of these auxilaries in
	// the interpolation base.
	if (m_regularizationWeight > 0) {
		std::cout << "	Computing extra stuff for regularization..." << std::endl;
		m_regularizationRHSMatrix = m_basis.transpose() * m_massMatrix * m_basis;
		PDPositions rhsFull(m_constraints.size() * m_auxiliarySize, 3);
		// Collect auxiliary variables for sampled constraints
		int didCollide = -1;
		PROJ_DYN_PARALLEL_FOR
			for (int i = 0; i < m_constraints.size(); i++) {
				PDScalar weight = 1.;
				if (!NO_WEIGHTS_IN_CONSTRUCTION) {
					weight = std::sqrt(m_constraints[i]->getWeight());
				}
				didCollide = -1;
				rhsFull.block(i*m_auxiliarySize, 0, m_auxiliarySize, 3) = m_constraints[i]->getP(m_restPositionFull, didCollide)
					* weight;
			}
		PDPositions rhsSub = m_basis.transpose() * m_massMatrix * rhsFull;
		PDMatrix projLHS = m_basis.transpose() * m_massMatrix * m_basis;
		Eigen::LLT<PDMatrix> projSolver;
		projSolver.compute(projLHS);
		for (unsigned int d = 0; d < 3; d++) {
			m_solutionLastFrame.col(d) = projSolver.solve(rhsSub.col(d));
		}
	}

	// Compute the finalization matrix, i.e. the one that maps a vector from the subspace
	// spanned by the snapshots' PCA to a fully assembled rhs.
	// In case a positions subspace is used, the transpose of the subspace matrix will
	// also be applied to the matrix
	std::cout << "	Computing 'finalize' matrices..." << std::endl;
	if (positionSubspaceT.rows() > 0) {
		std::cout << "		sparse part..." << std::endl;
		PDSparseMatrix finalTemp = positionSubspaceT * m_assemblyMatrix;
		if (NO_WEIGHTS_IN_CONSTRUCTION) {
			PROJ_DYN_PARALLEL_FOR
				for (int c = 0; c < finalTemp.cols(); c++) {
					finalTemp.col(c) *= weightVector(c);
				}
		}
		std::cout << "		dense part..." << std::endl;
		m_finalizeMatrix.resize(positionSubspaceT.rows(), m_basis.cols());
		PROJ_DYN_PARALLEL_FOR
			for (int c = 0; c < m_basis.cols(); c++) {
				m_finalizeMatrix.col(c) = finalTemp * m_basis.col(c);
			}
		m_usingPositionSubspace = true;
	}
	else {
		PDSparseMatrix temp = m_assemblyMatrix * m_weightMatrix;
		m_finalizeMatrixBig = (temp * m_basis).sparseView(0, 1e-16);
		m_usingPositionSubspace = false;
	}

	// Finally compute the solver for the projection problem if desired
	if (PROJ_DYN_ENABLE_RHS_PROJECTION) {
		std::cout << "	Computing projection solver for this group..." << std::endl;
		m_projectionSolver.compute(m_basis.transpose() * m_basis);
		if (m_usingPositionSubspace) {
			m_projectionFinalizeMatrix = positionSubspaceT * m_assemblyMatrix;
		}
	}

	std::cout << "Initiated snapshot interpolation for group " << m_groupName << "." << std::endl;

	m_interpolReady = true;
}

void PD::RHSInterpolationGroup::approximateRHS(PDPositions & pos, PDPositions & rhs, bool * collidedVertices)
{
	if (!m_interpolReady) {
		std::cout << "Interpolation is not initialized!!!" << std::endl;
		return;
	}

	interpolate(pos, m_tempSolution, collidedVertices);

	// Apply finalization matrix and add to current rhs
	if (m_usingPositionSubspace) {
		PROJ_DYN_PARALLEL_FOR
			for (int d = 0; d < 3; d++) {
				rhs.col(d) += m_weightFac * m_finalizeMatrix * m_tempSolution.col(d);
				//std::cout << m_finalizeMatrix * m_tempSolution.col(d) << std::endl;
			}
	}
	else {
		PROJ_DYN_PARALLEL_FOR
			for (int d = 0; d < 3; d++) {
				rhs.col(d) += m_weightFac * m_finalizeMatrixBig * m_tempSolution.col(d);
			}
	}
}

void PD::RHSInterpolationGroup::evaluateRHS(PDPositions & pos, PDPositions & rhs, bool projectToInterpolationSpaceAndBack, bool forceFull)
{
	if (PROJ_DYN_ENABLE_RHS_PROJECTION) {
		PDPositions auxils(m_constraints.size() * m_auxiliarySize, 3);
		PDPositions auxilsNoWeights(m_constraints.size() * m_auxiliarySize, 3);
		// Collect auxiliary variables for sampled constraints
		int didCollide = -1;
		PROJ_DYN_PARALLEL_FOR
			for (int i = 0; i < m_constraints.size(); i++) {
				didCollide = -1;
				PDScalar weight = m_constraints[i]->getWeight();
				if (!NO_WEIGHTS_IN_CONSTRUCTION) {
					weight = std::sqrt(weight);
				}
				auxils.block(i*m_auxiliarySize, 0, m_auxiliarySize, 3) = m_constraints[i]->getP(pos, didCollide)
					* weight * m_weightFac;
				auxilsNoWeights.block(i*m_auxiliarySize, 0, m_auxiliarySize, 3) = m_constraints[i]->getP(pos, didCollide);
			}

		if (projectToInterpolationSpaceAndBack) {
			PDPositions solutionSub = m_projectionSolver.solve(m_basis.transpose() * auxilsNoWeights);
			auxils = m_basis * solutionSub;
			PROJ_DYN_PARALLEL_FOR
				for (int i = 0; i < m_constraints.size(); i++) {
					didCollide = -1;
					PDScalar weight = m_constraints[i]->getWeight();
					if (!NO_WEIGHTS_IN_CONSTRUCTION) {
						weight = std::sqrt(weight);
					}
					auxils.block(i*m_auxiliarySize, 0, m_auxiliarySize, 3) *= weight * m_weightFac;
				}
		}

		if (m_usingPositionSubspace && !forceFull) {
			rhs += m_projectionFinalizeMatrix * auxils;
		}
		else {
			rhs += m_assemblyMatrix * auxils;
		}
	}
	else {
		std::cout << "Enable PROJ_DYN_ENABLE_RHS_PROJECTION if you desire to evaluate full rhs!" << std::endl;
	}
}

void PD::RHSInterpolationGroup::interpolate(PDPositions & pos, PDPositions & interpol, bool* collidedVertices)
{
	// Collect auxiliary variables for sampled constraints
	int didCollide = -1;
	PROJ_DYN_PARALLEL_FOR
		for (int i = 0; i < m_sampledConstraints.size(); i++) {
			didCollide = -1;
			PDScalar weight = 1.;
			if (!NO_WEIGHTS_IN_CONSTRUCTION) {
				weight = std::sqrt(m_sampledConstraints[i]->getWeight());
			}
			m_tempAuxils.block(i*m_auxiliarySize, 0, m_auxiliarySize, 3) = m_sampledConstraints[i]->getP(pos, didCollide)
				* weight * m_blowUp;
			if (collidedVertices && didCollide > 0) {
				collidedVertices[didCollide] = true;
			}
		}

	// Set up rhs from auxiliaries and solve the system for all three columns
	PROJ_DYN_PARALLEL_FOR
		for (int d = 0; d < 3; d++) {
			m_tempRHS.col(d) = m_interpolRHSMatrix * m_tempAuxils.col(d);
			if (m_regularizationWeight > 0.) {
				m_tempRHS.col(d) += m_regularizationWeight * m_regularizationRHSMatrix * m_solutionLastFrame.col(d);
			}
			interpol.col(d) = m_interpolSolver.solve(m_tempRHS.col(d));
		}

	m_solutionLastFrame = interpol;
}

PDMatrix PD::RHSInterpolationGroup::getLHSMatrixSubspace(PDMatrix& posSubspaceMat, PDMatrix& posSubspaceMat_T)
{
	if (!m_lhsMatrixSubspaceInitialized) {
		PDSparseMatrix temp;
		if (NO_WEIGHTS_IN_CONSTRUCTION) {
			temp = m_assemblyMatrix * m_weightMatrix * m_assemblyMatrix.transpose();
		}
		else {
			temp = m_assemblyMatrix * m_assemblyMatrix.transpose();
		}
		m_lhsMatrixSubspace = posSubspaceMat_T * temp * posSubspaceMat;
		m_lhsMatrixSubspaceInitialized = true;
	}
	return m_lhsMatrixSubspace;
}

PDSparseMatrix PD::RHSInterpolationGroup::getLHSMatrix()
{
	PDSparseMatrix temp;
	if (NO_WEIGHTS_IN_CONSTRUCTION) {
		temp = m_assemblyMatrix * m_weightMatrix * m_assemblyMatrix.transpose();
	}
	else {
		temp = m_assemblyMatrix * m_assemblyMatrix.transpose();
	}

	return temp;
}

bool PD::RHSInterpolationGroup::hasBasis()
{
	return m_basis.cols() >= 1 && m_basis.rows() > 1;
}

std::vector<ProjDynConstraint*>& PD::RHSInterpolationGroup::getSampledConstraints()
{
	return m_sampledConstraints;
}

std::vector<ProjDynConstraint*>& PD::RHSInterpolationGroup::getConstraints()
{
	return m_constraints;
}

void PD::RHSInterpolationGroup::setExampleWeights(std::vector<PDScalar>& weights)
{
	if (m_isTetExampleGroup) {
		for (ProjDynConstraint* c : m_sampledConstraints) {
			TetExampleBased* tc = (TetExampleBased*)c;
			tc->setExampleWeights(weights);
		}
	}
	else if (getName() == "spring") {
		for (ProjDynConstraint* c : m_sampledConstraints) {
			SpringConstraint* sc = (SpringConstraint*)c;
			sc->setExWeights(weights);
		}
	}
	else {
		return;
	}
}

void PD::RHSInterpolationGroup::setWeightFactor(PDScalar w)
{
	m_weightFac = w;
}

void PD::RHSInterpolationGroup::setBlowup(PDScalar s)
{
	m_blowUp = s;
}

std::string PD::RHSInterpolationGroup::getName()
{
	return m_groupName;
}
