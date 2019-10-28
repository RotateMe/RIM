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
#pragma once

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseCholesky"

#include "ProjDynTypeDef.h"

#define PROJ_DYN_BENDING_FLIP_THRESHOLD PDScalar(1e-5)

namespace PD {

	struct Edge;

	/*
	A "constraint" that can be added to a simulator, which imposes some material behaviour, i.e. models the internal forces.
	Specifically, it represents an energy term weight/2 * ||A S q - B p||^2 + delta_C(p), where q are the current positions,
	p is an auxilary variable, S is a selection matrix that usually selects specific vertices of q that are connected to this
	constraint and A and B are matrices that model the distance of the current state to the constraint.
	The most important term, delta_C(p) evaluates to 0 when p is in some set C and to infinity otherwise.
	The constraints are realized by implementing functions that return the (throughout one simulation constant) matrices A,
	B and S and, most importantly, the ability to return the best value of p, if q is fixed and the energy term for this constraint
	is minimized in isolation, i.e. the solution to the local solve.

	Note that A and B are usually very small and not necessarily sparse, but since we want several arithmetic operations to return
	sparse matrices, it is helpful to declare them as sparse matrices here.
	*/
	class ProjDynConstraint {
	public:
		ProjDynConstraint();
		PDSparseMatrixRM& getSelectionMatrix();
		PDSparseMatrix& getSelectionMatrixTransposed();
		virtual PDPositions getP(PDPositions& positions, int& didCollide) = 0;
		virtual void setUsedVertices(std::vector<unsigned int>& usedVertices) = 0;
		virtual ProjDynConstraint* copy() = 0;
		PDScalar getWeight();
		virtual bool isTetStrain() { return false; };
		bool isStiffConstraint();
		int getMainVertexIndex();
		int getMainTriangleIndex();
		int getMainTetIndex();
		PDMatrix& getSubspaceRHSMat(PDMatrix& baseMatT);
		void setSchurMatrices(PDMatrix& D, PDMatrix& A_inv_D_T);
		PDMatrix& getSchurD();
		PDMatrix& getSchurAinvDT();
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		unsigned int remapVertex(std::vector<unsigned int>& usedVertices, unsigned int index);
		void postInit();
		PDScalar weight;
		PDSparseMatrixRM m_selectionMatrix;
		PDSparseMatrix m_selectionMatrixTransposed;
		bool m_isStiffConstraint;
		int m_mainVInd;
		int m_mainTInd;
		int m_mainTetInd;
		PDMatrix m_subspaceRHS;
		bool m_subspaceRHSInit;

		PDMatrix m_D, m_A_inv_D_T;
	};

	class FloorCollisionConstraint : public ProjDynConstraint {
	public:
		FloorCollisionConstraint(unsigned int numVertices, unsigned int constrainedVertex,
			unsigned int floorCoord, PDScalar floorHeight, PDScalar collisionWeight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		int m_constrainedVertex;
		ProjDynConstraint *copy()
		{
			return new FloorCollisionConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		PDScalar m_floorHeight;
		int m_constrainedCoord;
	};


	class SpringConstraint : public ProjDynConstraint {
	public:
		SpringConstraint(unsigned int numVertices, unsigned int v1Ind, unsigned int v2Ind, PDScalar restLength, PDScalar weight,
			PDScalar rangeMin = 1., PDScalar rangeMax = 1.,
			const std::vector<PDPositions>& examplePoses = std::vector<PDPositions>(), const std::vector<PDScalar>& weights = std::vector<PDScalar>());
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		void setExWeights(std::vector<PDScalar>& weights);
		ProjDynConstraint *copy()
		{
			return new SpringConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		PDScalar m_rangeMin, m_rangeMax;
		unsigned int m_v1Ind, m_v2Ind;
		PDScalar m_restLength;
		PDScalar m_origRestLength;
		std::vector<PDScalar> m_exEdgeLengths;
		std::vector<PDScalar> m_exWeights;
	};

	/* Constraint that enforces that a vertex is exactly in the middle of two other vertices */
	class MiddleVertConstraint : public ProjDynConstraint {
	public:
		/* Enforces that v2 is exactly in the middle between v1 and v3. The weight will NOT be scaled by the size
		   of the edges between the vertices or anyhting (like for all other constraints), so a rescaling might
		   be in order. */
		MiddleVertConstraint(unsigned int numVertices, unsigned int v1Ind, unsigned int v2Ind, unsigned int v3Ind, PDScalar weight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		ProjDynConstraint *copy()
		{
			return new MiddleVertConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_v1Ind, m_v2Ind, m_v3Ind;
		PDPositions zero;
	};

	/* Constraint that turns an edge chain v1 -> v2 -> v3 into a hinge, i.e. it keeps the edges to remain in a plane (that is defined via
	    the initial configuration with the help of a fourth vertex v4 between v1 and v3) and optionally restricts the angle
		between the two angles from becoming obtuse.
	    This requires a forth vertex v4 that is actually part of the mesh, which, in the initial configuration
		is somewhere on the line segment between v1 and v3. Otherwise there is no reference point to define an obtuse angle between
		the edges.*/
	class HingeConstraint : public ProjDynConstraint {
	public:
		HingeConstraint(unsigned int numVertices, unsigned int v1Ind, unsigned int v2Ind, unsigned int v3Ind, unsigned int v4ind, bool preventObtuse, PDScalar weight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		ProjDynConstraint *copy()
		{
			return new HingeConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_v1Ind, m_v2Ind, m_v3Ind, m_v4Ind;
		bool m_preventObtuse;
		PDPositions zero;
	};


	class BendingConstraint : public ProjDynConstraint {
	public:
		BendingConstraint(unsigned int numVertices, unsigned int vInd, PDPositions& positions,
			PDTriangles& tris, std::vector<Edge> vertexStar,
			PDScalar voronoiArea, PDScalar sweight, bool preventBendingFlips,
			bool flatBending);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		ProjDynConstraint *copy()
		{
			return new BendingConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		PDPositions zeroVec;
		unsigned int m_vInd;
		std::vector<Edge> m_vertexStar;
		PDTriangles m_triangles;
		PDScalar m_restMeanCurvature;
		PDVector m_cotanWeights;
		PDScalar m_dotProductWithNormal;
		bool m_preventBendingFlips;
	};


	class StrainConstraint : public ProjDynConstraint {
	public:
		StrainConstraint(unsigned int numVertices, unsigned int tInd, PDTriangles& triangles,
			PDPositions& positions, PDScalar minStrain, PDScalar maxStrain, PDScalar sweight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		ProjDynConstraint *copy()
		{
			return new StrainConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_tInd;
		PDScalar m_minStrain, m_maxStrain;
		Eigen::Matrix<PDScalar, 2, 2> m_restEdgesInv;
		unsigned int m_vInd1, m_vInd2, m_vInd3;
	};

	class TetStrainConstraint : public ProjDynConstraint {
	public:
		TetStrainConstraint(unsigned int numVertices, unsigned int tInd, PDTets& tets,
			PDPositions& positions, PDScalar minStrain, PDScalar maxStrain, PDScalar strainWeight,
			PDScalar incompressibilityWeight = 0.);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		PDScalar getDet(PDPositions& positions);
		bool isTetStrain() { return true; };
		ProjDynConstraint *copy()
		{
			return new TetStrainConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_tInd;
		PDScalar m_minStrain, m_maxStrain;
		PDScalar m_strainWeight, m_incWeight;
		Eigen::Matrix<PDScalar, 3, 3> m_restEdgesInv;
		unsigned int m_vInd1, m_vInd2, m_vInd3, m_vInd4;
	};


	class TetExampleBased : public ProjDynConstraint {
	public:
		TetExampleBased(unsigned int numVertices, unsigned int tInd, PDTets& tets,
			PDPositions& positions, std::vector<PDPositions>& examplePos,
			std::vector<PDScalar> initialExampleWeights, PDScalar sweight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setExampleWeights(std::vector<PDScalar> initialExampleWeights);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		ProjDynConstraint *copy()
		{
			return new TetExampleBased(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_tInd;
		Eigen::Matrix<PDScalar, 3, 3> m_restEdges;
		Eigen::Matrix<PDScalar, 3, 3> m_restEdgesInv;
		std::vector<Eigen::Matrix<PDScalar, 3, 3>> m_rotatedExampleEdges;
		Eigen::Matrix<PDScalar, 3, 3> m_curOptiDefGrad;
		unsigned int m_vInd1, m_vInd2, m_vInd3, m_vInd4;
	};


	class PositionConstraint : public ProjDynConstraint {
	public:
		PositionConstraint(unsigned int numVertices, unsigned int vInd, PDPositions pos, PDScalar sweight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		PDPositions getPos();
		void setPos(PDPositions newPos);
		ProjDynConstraint *copy()
		{
			return new PositionConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_vInd;
		PDPositions m_pos;
	};

	class PositionMultiConstraint : public ProjDynConstraint {
	public:
		PositionMultiConstraint(unsigned int numVertices, std::vector< unsigned int >& vInds, PDPositions& positions, PDScalar sweight);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		void setPos(PDPositions newPos);
		ProjDynConstraint *copy()
		{
			return new PositionMultiConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		std::vector< unsigned int > m_vInds;
		PDPositions m_pos;
	};

	class CenterConstraint : public ProjDynConstraint {
	public:
		/* The argument numVertices is the number of all vertices of the mesh,
		and the argument positions is supposed to be a vector containing all vertex positions,
		while the argument vInds contains a list of the vertices whose center of mass should be constrained to the
		center (which can be moved via setCenter or translate center. */
		CenterConstraint(unsigned int numVertices, std::vector< unsigned int >& vInds, PDPositions& positions, PDScalar sweight, bool initiallyActive = true);
		PDPositions getP(PDPositions& positions, int& didCollide);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		PDPositions getCenter();
		void setCenter(PDPositions newCenter);
		void translateCenter(PDPositions tCenter);
		/* Returns the ORIGINAL indices of the vertices in this group. Note that those might have been
		   remapped via setUsedVertices(), such that the list m_vInds will differ from the list returned
		   by this function! */
		std::vector<unsigned int>& getVertexInds();
		void setActive(bool);
		bool wasInitiallyActive();
		ProjDynConstraint *copy()
		{
			return new CenterConstraint(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		std::vector< unsigned int > m_vInds;
		std::vector< unsigned int > m_vInds_orig;
		PDPositions m_center;
		PDPositions m_origCenter;
		bool m_isActive;
		bool m_initiallyActive;
	};

	/* This constraint is simply an auxiliary class that always just returns the input position of
	   a chosen vertex as the projection. It is for example used to generate snapshots of the
	   mesh's vertices' positions. */
	class FreePosition : public ProjDynConstraint {
	public:
		FreePosition(unsigned int numVertices, unsigned int vInd);
		void setUsedVertices(std::vector<unsigned int>& usedVertices);
		PDPositions getP(PDPositions& positions, int& didCollide);
		ProjDynConstraint *copy()
		{
			return new FreePosition(*this);
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		unsigned int m_vInd;
	};

	class PositionConstraintGroup {
	private:
		std::vector< PositionConstraint* > m_constraints;
		std::vector< PDPositions > m_originalPositions;
	public:
		void addConstraint(PositionConstraint* con);
		void translateGroup(PDPositions trans);
		void setGroup(PDPositions trans);
		void reset();
	};
}