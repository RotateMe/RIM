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

#include "dynamics/ProjDynConstraints.h"
#include "dynamics/ProjDynSimulator.h"
#include "dynamics/ProjDynUtil.h"

using namespace PD;

PD::ProjDynConstraint::ProjDynConstraint()
:
m_mainVInd(-1),
m_mainTInd(-1),
m_mainTetInd(-1),
weight(0),
m_isStiffConstraint(false),
m_subspaceRHSInit(false)
{
}

PDSparseMatrixRM& ProjDynConstraint::getSelectionMatrix()
{
	return m_selectionMatrix;
}

PDSparseMatrix& PD::ProjDynConstraint::getSelectionMatrixTransposed()
{
	return m_selectionMatrixTransposed;
}


PDScalar PD::ProjDynConstraint::getWeight() {
	return weight;
}

bool PD::ProjDynConstraint::isStiffConstraint()
{
	return m_isStiffConstraint;
}

int PD::ProjDynConstraint::getMainVertexIndex()
{
	return m_mainVInd;
}

int PD::ProjDynConstraint::getMainTriangleIndex()
{
	return m_mainTInd;
}

int PD::ProjDynConstraint::getMainTetIndex()
{
	return m_mainTetInd;
}

PDMatrix & PD::ProjDynConstraint::getSubspaceRHSMat(PDMatrix & baseMatT)
{
	if (!m_subspaceRHSInit) {
		m_subspaceRHS = baseMatT * m_selectionMatrixTransposed;
		m_subspaceRHSInit = true;
	}
	return m_subspaceRHS;
}

void PD::ProjDynConstraint::setSchurMatrices(PDMatrix & D, PDMatrix & A_inv_D_T)
{
	m_D = D;
	m_A_inv_D_T = A_inv_D_T;
}

PDMatrix & PD::ProjDynConstraint::getSchurD()
{
	return m_D;
}

PDMatrix & PD::ProjDynConstraint::getSchurAinvDT()
{
	return m_A_inv_D_T;
}

void PD::ProjDynConstraint::postInit()
{
}

unsigned int PD::ProjDynConstraint::remapVertex(std::vector<unsigned int>& usedVertices, unsigned int index) {
	for (unsigned int i = 0; i < usedVertices.size(); i++) {
		if (usedVertices[i] == index) return i;
	}
	std::cout << "Error: a constraint is using a vertex that does not appear in the list of used vertices!" << std::endl;
	return 0;
}


namespace PD {

	FloorCollisionConstraint::FloorCollisionConstraint(unsigned int numVertices,
		unsigned int constrainedVertex, unsigned int floorCoord, PDScalar floorHeight,
		PDScalar collisionWeight)
		:
		ProjDynConstraint()
	{
		m_isStiffConstraint = false;

		// Select z coordinate of the vertex with index constrainedVertex
		m_constrainedCoord = floorCoord;
		m_floorHeight = floorHeight;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.insert(0, constrainedVertex) = 1;

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		m_constrainedVertex = constrainedVertex;

		weight = collisionWeight;

		m_mainVInd = constrainedVertex;

		postInit();
	}

	void FloorCollisionConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices) {
		m_constrainedVertex = remapVertex(usedVertices, m_constrainedVertex);
	}


	PDPositions FloorCollisionConstraint::getP(PDPositions& positions, int& didCollide)
	{
		PDScalar p_vz = positions(m_constrainedVertex, m_constrainedCoord);
		PDPositions targetPos = positions.row(m_constrainedVertex);
		if (p_vz <= m_floorHeight) {
			didCollide = m_constrainedVertex;
			targetPos(0, m_constrainedCoord) = m_floorHeight;
		}
		return targetPos;
	}


	SpringConstraint::SpringConstraint(unsigned int numVertices, unsigned int v1Ind, unsigned int v2Ind, PDScalar restLength,
		PDScalar sweight, PDScalar rangeMin, PDScalar rangeMax,
		const std::vector<PDPositions>& examplePoses, const std::vector<PDScalar>& exWeights)
		:
		ProjDynConstraint()
	{
		m_isStiffConstraint = true;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.insert(0, v1Ind) = 1;
		m_selectionMatrix.insert(0, v2Ind) = -1;

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		m_v1Ind = v1Ind;
		m_v2Ind = v2Ind;

		weight = sweight * restLength;

		m_restLength = restLength;
		m_origRestLength = restLength;

		m_rangeMin = m_restLength * rangeMin;
		m_rangeMax = m_restLength * rangeMax;

		m_mainVInd = (v1Ind < v2Ind) ? v1Ind : v2Ind;

		for (auto& p : examplePoses) {
			m_exEdgeLengths.push_back((p.row(v1Ind) - p.row(v2Ind)).norm());
		}
		for (const PDScalar& w : exWeights) {
			m_exWeights.push_back(w);
		}

		setExWeights(m_exWeights);
		postInit();
	}

	PDPositions SpringConstraint::getP(PDPositions& positions, int& didCollide)
	{
		PDPositions edge = (positions.row(m_v1Ind) - positions.row(m_v2Ind));
		PDScalar curLength = edge.norm();
		edge /= curLength;
		PDScalar targetLength = clamp(curLength, m_rangeMin, m_rangeMax);
		edge *= targetLength;
		return edge;
	}

	void SpringConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_v1Ind = remapVertex(usedVertices, m_v1Ind);
		m_v2Ind = remapVertex(usedVertices, m_v2Ind);
	}

	void SpringConstraint::setExWeights(std::vector<PDScalar>& weights)
	{
		m_exWeights = weights;
		m_restLength = m_origRestLength;
		for (int i = 0; i < std::min(m_exEdgeLengths.size(), m_exWeights.size()); i++) {
			PDScalar& w = m_exWeights[i];
			m_restLength += w * (m_exEdgeLengths[i] - m_origRestLength);
		}
		m_rangeMin = m_restLength;
		m_rangeMax = m_restLength;
	}

	BendingConstraint::BendingConstraint(unsigned int numVertices, unsigned int vInd,
		PDPositions& positions, PDTriangles& tris, std::vector<Edge> vertexStar, PDScalar voronoiArea, PDScalar sweight,
		bool preventBendingFlips, bool flatBending)
		:
		ProjDynConstraint()
	{
		m_isStiffConstraint = true;

		m_vInd = vInd;
		m_mainVInd = vInd;
		m_vertexStar = vertexStar;

		zeroVec = PDPositions(1, 3);
		zeroVec.setZero();

		m_preventBendingFlips = preventBendingFlips;

		PDVector cotanWeights(vertexStar.size());
		int nb = 0, nb2 = 0;
		std::vector<unsigned int> trisSeen;
		trisSeen.reserve(vertexStar.size());
		m_triangles.resize(vertexStar.size(), 3);
		for (Edge& e : vertexStar) {
			PD3dVector edgeT11 = (positions.row(m_vInd) - positions.row(e.vOtherT1));
			PD3dVector edgeT12 = (positions.row(e.v2) - positions.row(e.vOtherT1));
			PDScalar angle1 = std::acos(edgeT11.dot(edgeT12) / (edgeT11.norm() * edgeT12.norm()));
			PDScalar cotWeight = (.5 / std::tan(angle1));
			if (e.t2 > 0) {
				PD3dVector edgeT21 = (positions.row(m_vInd) - positions.row(e.vOtherT2));
				PD3dVector edgeT22 = (positions.row(e.v2) - positions.row(e.vOtherT2));
				PDScalar angle2 = std::acos(edgeT21.dot(edgeT22) / (edgeT21.norm() * edgeT22.norm()));
				cotWeight += (.5 / std::tan(angle2));
			}

			cotanWeights(nb2) = cotWeight / voronoiArea;

			if (std::find(trisSeen.begin(), trisSeen.end(), e.t1) == trisSeen.end()) {
				trisSeen.push_back(e.t1);
				m_triangles.row(nb2) = tris.row(e.t1);
				nb2++;
			}
			else if (e.t2 >= 0) {
				trisSeen.push_back(e.t2);
				m_triangles.row(nb2) = tris.row(e.t2);
				nb2++;
			}
			nb++;
		}

		if (nb2 < nb) {
			PDTriangles triTemp = m_triangles.block(0, 0, nb2, 3);
			m_triangles = triTemp;
		}

		m_cotanWeights = cotanWeights;

		weight = sweight * voronoiArea;

		PD3dVector meanCurvatureVector;
		meanCurvatureVector.setZero();
		nb = 0;
		for (Edge e : vertexStar) {
			meanCurvatureVector += (positions.row(m_vInd) - positions.row(e.v2)) * cotanWeights(nb);
			nb++;
		}
		m_restMeanCurvature = meanCurvatureVector.norm();

		if (flatBending) m_restMeanCurvature = 0.;

		PD3dVector triNormal = getTriangleNormal(m_triangles, positions);
		m_dotProductWithNormal = triNormal.dot(meanCurvatureVector);

		//std::cout << "dot with normal " << m_dotProductWithNormal << ", " << triNormal.dot(meanCurvatureVector.normalized()) << std::endl;

		if (m_cotanWeights.hasNaN()) {
			std::cout << "NaN value in cotan weights!" << std::endl;
		}

		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.setZero();
		m_selectionMatrix.insert(0, m_vInd) = cotanWeights.sum();
		nb = 0;
		for (Edge e : vertexStar) {
			m_selectionMatrix.insert(0, e.v2) = -cotanWeights(nb);
			nb++;
		}

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		postInit();
	}

	PDPositions BendingConstraint::getP(PDPositions& positions, int& didCollide)
	{
		if (m_restMeanCurvature < 1e-12) {
			return zeroVec;
		}

		PDPositions starEdgeSum;
		starEdgeSum.resize(1, 3);
		starEdgeSum.setZero();
		int nb = 0;
		for (Edge e : m_vertexStar) {
			starEdgeSum.row(0) += (positions.row(m_vInd) - positions.row(e.v2)) * m_cotanWeights(nb);
			nb++;
		}

		PDScalar norm = starEdgeSum.row(0).norm();
		if (norm < 1e-10) {
			starEdgeSum.row(0) = getTriangleNormal(m_triangles, positions) * m_restMeanCurvature;
		}
		else {
			starEdgeSum *= m_restMeanCurvature / norm;
		}

		if (m_preventBendingFlips) {
			PD3dVector triNormal = getTriangleNormal(m_triangles, positions);
			PDScalar dot = triNormal.dot(starEdgeSum.row(0));
			if (norm > PROJ_DYN_BENDING_FLIP_THRESHOLD && dot * m_dotProductWithNormal < 0) starEdgeSum *= -1;
		}

		/*
		if (starEdgeSum.hasNaN()) {
			std::cout << "Error in bending constraint!" << std::endl;
		}
		*/

		return starEdgeSum;
	}

	void BendingConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_vInd = remapVertex(usedVertices, m_vInd);
		for (Edge& e : m_vertexStar) {
			e.v1 = remapVertex(usedVertices, e.v1);
			e.v2 = remapVertex(usedVertices, e.v2);
		}
		for (unsigned int i = 0; i < m_triangles.rows(); i++) {
			for (unsigned int d = 0; d < 3; d++) {
				m_triangles(i, d) = remapVertex(usedVertices, m_triangles(i, d));
			}
		}
	}

	StrainConstraint::StrainConstraint(unsigned int numVertices, unsigned int tInd, PDTriangles& triangles, PDPositions& positions,
		PDScalar minStrain, PDScalar maxStrain, PDScalar sweight) :
		ProjDynConstraint(),
		m_minStrain(minStrain),
		m_maxStrain(maxStrain),
		m_tInd(tInd)
	{
		m_isStiffConstraint = true;
		m_mainTInd = tInd;

		m_vInd1 = triangles(tInd, 0);
		m_vInd2 = triangles(tInd, 1);
		m_vInd3 = triangles(tInd, 2);

		Eigen::Matrix<PDScalar, 3, 2> edges, P;
		// 3d edges of triangle
		edges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		edges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));

		// Projection that embeds these edges isometrically in 2d
		P.col(0) = edges.col(0).normalized();
		P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
		// Inverse of matrix containing 2d rest edges
		Eigen::Matrix<PDScalar, 2, 2> restEdges = P.transpose() * edges;
		m_restEdgesInv = restEdges.inverse();

		// Compute the weight (triangle area times defined weight)
		PDScalar area = (P.transpose() * edges).determinant() / 2.0f;
		weight = sweight * std::sqrt(std::abs(area));

		// Set up S matrix to select the three vertices of this triangle
		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(2, numVertices);
		for (int coord2d = 0; coord2d < 2; coord2d++) {
			m_selectionMatrix.insert(coord2d, triangles(tInd, 0)) = -(m_restEdgesInv(0, coord2d) + m_restEdgesInv(1, coord2d));
			m_selectionMatrix.insert(coord2d, triangles(tInd, 1)) = m_restEdgesInv(0, coord2d);
			m_selectionMatrix.insert(coord2d, triangles(tInd, 2)) = m_restEdgesInv(1, coord2d);
		}
		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		postInit();
	}

	PDPositions StrainConstraint::getP(PDPositions& positions, int & didCollide)
	{
		// Project the current edges isometrically into 2d, compute the deformation gradient there
		// then perform the SVD on the def.grad., clamp the singular values, and project the deformation
		// gradient back, which will then be equal to the optimal configuration TIMES the inverse rest
		// state edges (as a 3x2 times 2x2 matrix multiplication)
		Eigen::Matrix<PDScalar, 3, 2> edges, P;

		// 3d edges of triangle
		edges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		edges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));

		// Projection that embeds these edges isometrically in 2d
		P.col(0) = edges.col(0).normalized();
		P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
		// Compute the deformation gradient (current 2d edges times inverse of original 2d edges)
		Eigen::Matrix<PDScalar, 2, 2> F = P.transpose() * edges * m_restEdgesInv;
		// Compute SVD
		Eigen::JacobiSVD<Eigen::Matrix<PDScalar, 2, 2>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PD2dVector S = svd.singularValues();
		// Clamp singular values
		S(0) = clamp(S(0), m_minStrain, m_maxStrain);
		S(1) = clamp(S(1), m_minStrain, m_maxStrain);
		// Compute clamped deformation gradient
		F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
		// Create optimal 3d edge configuration (times the inverse original 2d edges)...
		PDPositions PF = (P * F).transpose();

		return PF;
	}

	void StrainConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_vInd1 = remapVertex(usedVertices, m_vInd1);
		m_vInd2 = remapVertex(usedVertices, m_vInd2);
		m_vInd3 = remapVertex(usedVertices, m_vInd3);
	}

	TetStrainConstraint::TetStrainConstraint(unsigned int numVertices, unsigned int tInd, PDTets& tets,
		PDPositions& positions, PDScalar minStrain, PDScalar maxStrain, PDScalar strainWeight,
		PDScalar incompressibilityWeight) :
		ProjDynConstraint(),
		m_minStrain(minStrain),
		m_maxStrain(maxStrain),
		m_incWeight(incompressibilityWeight),
		m_strainWeight(strainWeight),
		m_tInd(tInd)
	{
		m_isStiffConstraint = true;
		m_mainTetInd = tInd;

		// Initialize vertex indices 
		m_vInd1 = tets(tInd, 0);
		m_vInd2 = tets(tInd, 1);
		m_vInd3 = tets(tInd, 2);
		m_vInd4 = tets(tInd, 3);

		if (m_vInd1 < 0 || m_vInd1 >= numVertices ||
			m_vInd2 < 0 || m_vInd2 >= numVertices ||
			m_vInd3 < 0 || m_vInd3 >= numVertices ||
			m_vInd4 < 0 || m_vInd4 >= numVertices) {
			std::cout << "ERROR: FAULTY CONSTRAINT!!!" << std::endl;
		}

		// 3d edges of tet
		Eigen::Matrix<PDScalar, 3, 3> edges;
		edges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		edges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));
		edges.col(2) = (positions.row(m_vInd4) - positions.row(m_vInd1));

		// Inverse of edges matrix for computation of the deformation gradient
		m_restEdgesInv = edges.inverse();
		bool didCorrect = false;
		while (!m_restEdgesInv.allFinite()) {
			std::cout << "Illegal edges in mesh!" << std::endl;
			for (int c = 0; c < 3; c++) {
				if (edges.col(c).norm() < 1e-12) {
					for (int r = 0; r < 3; r++) {
						edges(r,c) = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 1e-11;
					}
				}
			}
			for (int r = 0; r < 3; r++) {
				if (edges.row(r).norm() < 1e-12) {
					for (int c = 0; c < 3; c++) {
						edges(r, c) = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 1e-11;
					}
				}
			}			
			m_restEdgesInv = edges.inverse();
			didCorrect = true;
		}
		if (didCorrect) std::cout << "Fixed!" << std::endl;

		// Weight gets multiplied by tet volume
		PDScalar vol = std::abs((edges).determinant()) / 6.0f;
		weight = (m_strainWeight + m_incWeight) * vol;

		// The selection matrix computes the current deformation gradient w.r.t the current positions
		// (i.e. multiplication of the current edges with the inverse of the original edge matrix)
		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(3, numVertices);
		for (int coord3d = 0; coord3d < 3; coord3d++) {
			m_selectionMatrix.insert(coord3d, tets(tInd, 0)) = -(m_restEdgesInv(0, coord3d) + m_restEdgesInv(1, coord3d) + m_restEdgesInv(2, coord3d));
			m_selectionMatrix.insert(coord3d, tets(tInd, 1)) = m_restEdgesInv(0, coord3d);
			m_selectionMatrix.insert(coord3d, tets(tInd, 2)) = m_restEdgesInv(1, coord3d);
			m_selectionMatrix.insert(coord3d, tets(tInd, 3)) = m_restEdgesInv(2, coord3d);
		}
		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		postInit();
	}


	PDScalar TetStrainConstraint::getDet(PDPositions& positions) {
		// 3d edges of tet
		Eigen::Matrix<PDScalar, 3, 3> edges;
		edges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		edges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));
		edges.col(2) = (positions.row(m_vInd4) - positions.row(m_vInd1));

		// Compute the deformation gradient (current edges times inverse of original edges)
		Eigen::Matrix<PDScalar, 3, 3> F = edges * m_restEdgesInv;

		// Compute SVD
		Eigen::JacobiSVD<Eigen::Matrix<PDScalar, 3, 3>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PD3dVector S = svd.singularValues();
		PDScalar detU = svd.matrixU().determinant();
		PDScalar detV = svd.matrixV().determinant();

		// Compute the determinant
		PDScalar det = S(0) * S(1) * S(2) * detU * detV;
		return det;
	}

	PDPositions TetStrainConstraint::getP(PDPositions& positions, int& didCollide)
	{
		// Compute deformation gradient, clamp its singular values and output
		// corrected deformation gradient as the projection

		// 3d edges of tet
		Eigen::Matrix<PDScalar, 3, 3> edges;
		edges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		edges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));
		edges.col(2) = (positions.row(m_vInd4) - positions.row(m_vInd1));

		// Compute the deformation gradient (current edges times inverse of original edges)
		Eigen::Matrix<PDScalar, 3, 3> F = edges * m_restEdgesInv;
		// Compute SVD
		Eigen::JacobiSVD<Eigen::Matrix<PDScalar, 3, 3>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PD3dVector S = svd.singularValues();
		PDScalar detU = svd.matrixU().determinant();
		PDScalar detV = svd.matrixV().determinant();

		// Clamp singular values
		S(0) = clamp(S(0), m_minStrain, m_maxStrain);
		S(1) = clamp(S(1), m_minStrain, m_maxStrain);
		S(2) = clamp(S(2), m_minStrain, m_maxStrain);
		// Reverse largest singular value if tet is inverted:
		if (detU*detV < 0.0f) S(2) = -S(2);

		// Compute optimal deformation gradient
		if (m_incWeight <= 0) {
			// No volume preservation: optimal def. grad. is the optimal rotation
			F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
		}
		else {
			// Volume preservation: optimal def. grad. is a linear combination between optimal rotation
			// and the rescaled current deformation gradient that has determinant 1.
			F = (m_strainWeight / (m_strainWeight + m_incWeight)) * svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
			// Compute closest matrix to F with determinant 1
			PD3dVector d(0.0f, 0.0f, 0.0f);
			for (int i = 0; i < 8; ++i) {
				PDScalar v = S(0) * S(1) * S(2);
				PDScalar f = v - 1.;
				PD3dVector g(S(1)*S(2), S(0)*S(2), S(0)*S(1));
				d = -((f - g.dot(d)) / g.dot(g)) * g;
				S = svd.singularValues() + d;
			}
			if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0f) S(2) = -S(2);
			F += (m_incWeight / (m_strainWeight + m_incWeight)) * svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
		}

		if (F.hasNaN()) {
			F.setIdentity();
		}

		// Output corrected deformation gradient
		return F.transpose();
	}

	void TetStrainConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_vInd1 = remapVertex(usedVertices, m_vInd1);
		m_vInd2 = remapVertex(usedVertices, m_vInd2);
		m_vInd3 = remapVertex(usedVertices, m_vInd3);
		m_vInd4 = remapVertex(usedVertices, m_vInd4);
	}

	PositionConstraint::PositionConstraint(unsigned int numVertices, unsigned int vInd, PDPositions pos, PDScalar sweight)
		:
		ProjDynConstraint()
	{
		m_isStiffConstraint = false;

		if (vInd >= numVertices) {
			m_vInd = 0;
			std::cout << "Illegal position constraint added!" << std::endl;
		}
		m_vInd = vInd;
		m_mainVInd = vInd;
		m_pos = pos;
		weight = sweight;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.insert(0, m_vInd) = 1;

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();
	}

	PDPositions PositionConstraint::getP(PDPositions& positions, int& didCollide) {
		return m_pos;
	}

	void PositionConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_vInd = remapVertex(usedVertices, m_vInd);
	}

	PDPositions PositionConstraint::getPos()
	{
		return m_pos;
	}

	void PositionConstraint::setPos(PDPositions newPos)
	{
		m_pos = newPos;
	}

	void PositionConstraintGroup::addConstraint(PositionConstraint* con)
	{
		m_constraints.push_back(con);
		m_originalPositions.push_back(con->getPos());
	}

	void PositionConstraintGroup::translateGroup(PDPositions trans)
	{
		for (int i = 0; i < m_constraints.size(); i++) {
			PDPositions newPos = m_originalPositions[i] + trans;
			m_constraints[i]->setPos(newPos);
		}
	}

	void PositionConstraintGroup::setGroup(PDPositions pos)
	{
		PDPositions center(1, 3);
		center.setZero();
		for (int i = 0; i < m_constraints.size(); i++) {
			center += m_originalPositions[i];
		}
		center /= (double)m_constraints.size();
		for (int i = 0; i < m_constraints.size(); i++) {
			m_constraints[i]->setPos(pos + (m_originalPositions[i] - center));
		}
	}

	void PositionConstraintGroup::reset()
	{
		m_constraints.clear();
		m_originalPositions.clear();
	}

	CenterConstraint::CenterConstraint(unsigned int numVertices, std::vector<unsigned int>& vInds,
		PDPositions& positions, PDScalar sweight, bool initiallyActive)
	{
		m_isStiffConstraint = false;

		m_vInds = vInds;
		std::sort(m_vInds.begin(), m_vInds.end());
		m_vInds.erase(std::unique(m_vInds.begin(), m_vInds.end()), m_vInds.end());

		m_vInds_orig = m_vInds;

		m_mainVInd = m_vInds[0];
		m_center.setZero(1, 3);
		for (unsigned int v : m_vInds) {
			m_center += positions.row(v);
		}
		m_center /= (PDScalar)m_vInds.size();
		m_origCenter = m_center;
		weight = sweight;

		m_isActive = initiallyActive;
		m_initiallyActive = initiallyActive;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		for (unsigned int v : m_vInds) {
			m_selectionMatrix.insert(0, v) = 1./(PDScalar)m_vInds.size();
		}

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();
	}

	PDPositions CenterConstraint::getP(PDPositions& positions, int& didCollide)
	{
		if (m_isActive) {
			return m_center;
		}
		else {
			PDPositions center;
			center.setZero(1, 3);
			for (unsigned int v : m_vInds) {
				center += positions.row(v);
			}
			center /= (PDScalar)m_vInds.size();
			return center;
		}
	}

	void CenterConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		for (unsigned int i = 0; i < m_vInds.size(); i++) {
			m_vInds[i] = remapVertex(usedVertices, m_vInds[i]);
		}
	}

	PDPositions CenterConstraint::getCenter()
	{
		return m_center;
	}

	void CenterConstraint::setCenter(PDPositions newCenter)
	{
		m_center = newCenter.row(0);
	}

	void CenterConstraint::translateCenter(PDPositions tCenter)
	{
		m_center = m_origCenter + tCenter;
	}

	std::vector<unsigned int>& CenterConstraint::getVertexInds()
	{
		return m_vInds_orig;
	}

	void CenterConstraint::setActive(bool active)
	{
		m_isActive = active;
	}

	bool CenterConstraint::wasInitiallyActive()
	{
		return m_initiallyActive;
	}

	FreePosition::FreePosition(unsigned int numVertices, unsigned int vInd)
	{
		m_mainVInd = vInd;
		m_vInd = vInd;

		weight = 1.;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.insert(0, m_vInd) = 1.;

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();
	}

	void FreePosition::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_vInd = remapVertex(usedVertices, m_vInd);
	}

	PDPositions FreePosition::getP(PDPositions & positions, int & didCollide)
	{
		PDPositions myPos = positions.row(m_vInd);
		return myPos;
	}

	MiddleVertConstraint::MiddleVertConstraint(unsigned int numVertices, unsigned int v1Ind, unsigned int v2Ind, unsigned int v3Ind, PDScalar sweight)
		:
		ProjDynConstraint()
	{
		m_isStiffConstraint = true;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.insert(0, v1Ind) = 0.5;
		m_selectionMatrix.insert(0, v2Ind) = -1;
		m_selectionMatrix.insert(0, v3Ind) = 0.5;

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		m_v1Ind = v1Ind;
		m_v2Ind = v2Ind;
		m_v3Ind = v3Ind;

		weight = sweight;

		m_mainVInd = v2Ind;

		zero.setZero(1, 3);

		postInit();
	}

	PDPositions MiddleVertConstraint::getP(PDPositions & positions, int & didCollide)
	{
		return zero;
	}

	void MiddleVertConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_v1Ind = remapVertex(usedVertices, m_v1Ind);
		m_v2Ind = remapVertex(usedVertices, m_v2Ind);
		m_v3Ind = remapVertex(usedVertices, m_v3Ind);
	}

	HingeConstraint::HingeConstraint(unsigned int numVertices, unsigned int v1Ind, unsigned int v2Ind, unsigned int v3Ind, unsigned int v4Ind, bool preventObtuse, PDScalar sweight)
		:
		ProjDynConstraint()
	{
		m_isStiffConstraint = true;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(1, numVertices);
		m_selectionMatrix.insert(0, v1Ind) = 0.5;
		m_selectionMatrix.insert(0, v2Ind) = -1;
		m_selectionMatrix.insert(0, v3Ind) = 0.5;

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		m_v1Ind = v1Ind;
		m_v2Ind = v2Ind;
		m_v3Ind = v3Ind;
		m_v4Ind = v4Ind;

		m_preventObtuse = preventObtuse;

		weight = sweight;

		m_mainVInd = v2Ind;

		zero.setZero(1, 3);

		postInit();
	}

	PDPositions HingeConstraint::getP(PDPositions& positions, int& didCollide)
	{
		// Vector that points from v2 to the middle of v1 and v3
		PDPositions curVec = positions.row(m_v1Ind) * 0.5 + positions.row(m_v3Ind) * 0.5 - positions.row(m_v2Ind);
		// Vector from v2 to v4
		PDPositions v4Vec = positions.row(m_v4Ind) - positions.row(m_v2Ind);
		// Compute vector orthogonal to the edges v1 -> v3 and v2 -> v4
		PDPositions orthVec = ((positions.row(m_v1Ind) - positions.row(m_v3Ind)).row(0)).cross(v4Vec.row(0));
		// Project the hinge back to the original plane
		curVec -= orthVec * orthVec.row(0).dot(curVec.row(0));

		if (m_preventObtuse) {
			// Prevent angle from becoming obtuse
			if (curVec.row(0).dot(v4Vec.row(0)) < 0) {
				curVec.setZero();
			}
		}
		if (curVec.hasNaN()) {
			curVec.setZero();
		}
		return curVec;
	}

	void HingeConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_v1Ind = remapVertex(usedVertices, m_v1Ind);
		m_v2Ind = remapVertex(usedVertices, m_v2Ind);
		m_v3Ind = remapVertex(usedVertices, m_v3Ind);
		m_v4Ind = remapVertex(usedVertices, m_v4Ind);
	}

	TetExampleBased::TetExampleBased(unsigned int numVertices, unsigned int tInd,
				PDTets & tets, PDPositions & positions,
				std::vector<PDPositions>& examplePos,
				std::vector<PDScalar> initialExampleWeights,
				PDScalar sweight)
		:
		ProjDynConstraint(),
		m_tInd(tInd)
	{
		m_isStiffConstraint = true;
		m_mainTetInd = tInd;

		// Initialize vertex indices 
		m_vInd1 = tets(tInd, 0);
		m_vInd2 = tets(tInd, 1);
		m_vInd3 = tets(tInd, 2);
		m_vInd4 = tets(tInd, 3);

		if (m_vInd1 < 0 || m_vInd1 >= numVertices ||
			m_vInd2 < 0 || m_vInd2 >= numVertices ||
			m_vInd3 < 0 || m_vInd3 >= numVertices ||
			m_vInd4 < 0 || m_vInd4 >= numVertices) {
			std::cout << "ERROR: FAULTY CONSTRAINT!!!" << std::endl;
		}

		// 3d edges of tet
		m_restEdges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		m_restEdges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));
		m_restEdges.col(2) = (positions.row(m_vInd4) - positions.row(m_vInd1));

		// Inverse of edges matrix for computation of the deformation gradient
		m_restEdgesInv = m_restEdges.inverse();

		// Weight gets multiplied by tet volume
		PDScalar vol = std::abs((m_restEdges).determinant()) / 6.0f;
		weight = sweight * vol;

		// The selection matrix computes the current deformation gradient w.r.t the current positions
		// (i.e. multiplication of the current edges with the inverse of the original edge matrix)
		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(3, numVertices);
		for (int coord3d = 0; coord3d < 3; coord3d++) {
			m_selectionMatrix.insert(coord3d, tets(tInd, 0)) = -(m_restEdgesInv(0, coord3d) + m_restEdgesInv(1, coord3d) + m_restEdgesInv(2, coord3d));
			m_selectionMatrix.insert(coord3d, tets(tInd, 1)) = m_restEdgesInv(0, coord3d);
			m_selectionMatrix.insert(coord3d, tets(tInd, 2)) = m_restEdgesInv(1, coord3d);
			m_selectionMatrix.insert(coord3d, tets(tInd, 3)) = m_restEdgesInv(2, coord3d);
		}
		m_selectionMatrixTransposed = m_selectionMatrix.transpose();

		// Find optimal rotations that rotate the edges of the example shapes
		// to the edges of this shape (locally, for this tet)
		for (PDPositions& exPos : examplePos) {
			// 3d edges of example shape's tet
			Eigen::Matrix<PDScalar, 3, 3> exEdges;
			exEdges.col(0) = (exPos.row(m_vInd2) - exPos.row(m_vInd1));
			exEdges.col(1) = (exPos.row(m_vInd3) - exPos.row(m_vInd1));
			exEdges.col(2) = (exPos.row(m_vInd4) - exPos.row(m_vInd1));

			// Optimal rotation
			Eigen::Matrix<PDScalar, 3, 3> R = PD::findBestRotation(m_restEdges, exEdges);
			m_rotatedExampleEdges.push_back(exEdges); // R * exEdges);

		}

		setExampleWeights(initialExampleWeights);

		postInit();
	}

	PDPositions TetExampleBased::getP(PDPositions & positions, int & didCollide)
	{
		// Compute deformation gradient, clamp its singular values and output
		// corrected deformation gradient as the projection

		// 3d edges of tet
		Eigen::Matrix<PDScalar, 3, 3> edges;
		edges.col(0) = (positions.row(m_vInd2) - positions.row(m_vInd1));
		edges.col(1) = (positions.row(m_vInd3) - positions.row(m_vInd1));
		edges.col(2) = (positions.row(m_vInd4) - positions.row(m_vInd1));

		// Compute the deformation gradient (current edges times inverse of original edges)
		Eigen::Matrix<PDScalar, 3, 3> F = (edges * m_restEdgesInv).transpose();
		
		// Compute optimal rotation R that minimizes || F - R G ||, where G is 
		// the current desired deformation gradient, defined from the example
		// poses and the weights
		Eigen::Matrix<PDScalar, 3, 3> G = PD::findBestRotation(F, m_curOptiDefGrad) * m_curOptiDefGrad;

		// Output corrected deformation gradient
		return G;
	}

	void TetExampleBased::setExampleWeights(std::vector<PDScalar> exampleWeights)
	{
		m_curOptiDefGrad = m_restEdges;
		if (exampleWeights.size() > m_rotatedExampleEdges.size()) {
			std::cout << "Mismatch in the number of weights/example shapes provided!" << std::endl;
			return;
		}
		for (unsigned int i = 0; i < exampleWeights.size(); i++) {
			m_curOptiDefGrad += exampleWeights[i] * (m_rotatedExampleEdges[i] - m_restEdges);
		}
		m_curOptiDefGrad = (m_curOptiDefGrad * m_restEdgesInv).transpose();
	}

	void TetExampleBased::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		m_vInd1 = remapVertex(usedVertices, m_vInd1);
		m_vInd2 = remapVertex(usedVertices, m_vInd2);
		m_vInd3 = remapVertex(usedVertices, m_vInd3);
		m_vInd4 = remapVertex(usedVertices, m_vInd4);
	}

	PositionMultiConstraint::PositionMultiConstraint(unsigned int numVertices, std::vector<unsigned int>& vInds, PDPositions & positions, PDScalar sweight)
	{
		m_isStiffConstraint = false;

		m_vInds = vInds;
		std::sort(m_vInds.begin(), m_vInds.end());
		m_vInds.erase(std::unique(m_vInds.begin(), m_vInds.end()), m_vInds.end());

		m_mainVInd = m_vInds[0];
		m_pos.setZero(m_vInds.size(), 3);
		int ind = 0;
		for (unsigned int v : m_vInds) {
			m_pos.row(ind) = positions.row(v);
			ind++;
		}
		weight = sweight;

		m_selectionMatrix.setZero();
		m_selectionMatrix.resize(m_vInds.size(), numVertices);
		ind = 0;
		for (unsigned int v : m_vInds) {
			m_selectionMatrix.insert(ind, v) = 1.;
			ind++;
		}

		m_selectionMatrixTransposed = m_selectionMatrix.transpose();
	}

	PDPositions PositionMultiConstraint::getP(PDPositions & positions, int & didCollide)
	{
		return m_pos;
	}

	void PositionMultiConstraint::setUsedVertices(std::vector<unsigned int>& usedVertices)
	{
		for (unsigned int i = 0; i < m_vInds.size(); i++) {
			m_vInds[i] = remapVertex(usedVertices, m_vInds[i]);
		}
	}

	void PositionMultiConstraint::setPos(PDPositions newPos)
	{
		m_pos = newPos;
	}


}