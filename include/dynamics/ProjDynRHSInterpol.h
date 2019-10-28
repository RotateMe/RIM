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

#include "ProjDynTypeDef.h"

#define PI_CONST 3.14159265358979323846

/* If the number of samples is not at least MIN_OVERSAMPLING more than
   the number of basis vectors in the subspace base for the rhs interpolation,
   the base will be cut such that this requirement is met. */
#define MIN_OVERSAMPLING 20

/* If loading a base, it is cropped (from the left) to fit this size.
   The constant vector will always be added*/
#define MAX_LOADED_BASE_SIZE 9999

/* Enable for the ability to evaluate the error that solely comes from
   projecting the rhs into the subspaces of the groups (and not the additional
   error that comes from evaluating a small subset of the constraints).*/
#define PROJ_DYN_ENABLE_RHS_PROJECTION false

/* If this is turned on, the basis from the skinning
   construction is created without any weights, the interpolation will
   also be done without the weights, and only after the subspace rhs
   vector has been found, the weights are applied. */
#define NO_WEIGHTS_IN_CONSTRUCTION true

/* When performing the PCA to get a basis from the skinning
   construction, the mass matrix will be set to identity if this is turned
   on, a mass matrix will be used that assigns the area of the element
   associated to the constraint as a mass for this constraint.*/
#define NO_MASSES_IN_PCA false

using namespace PD;

namespace PD {

	class ProjDynConstraint;

	/* RHS approximation from interpolation (or rather fitting) into a subspace of constraint projections */
	class RHSInterpolationGroup {

	private:
		/* List of constraints for this group (should be all bending, or all strain, or ... */
		std::vector<ProjDynConstraint*> m_constraints;
		/* Basis for vectors of stacked constraint projections */
		PDMatrix m_basis;
		/* Number of rows per auxiliary variable */
		unsigned int m_auxiliarySize;
		/* Rest positions of the mesh */
		PDPositions m_restPositionFull;
		/* List of the subset of constraints that should be evaluated when computing the interpolation
		   into the space spanned by the basis*/
		std::vector<ProjDynConstraint*> m_sampledConstraints;
		/* Lumped mass matrix which gives a weight associated to each constraint, proportional
		   to the associated area of each constraints element (vertex, triangle or tet)*/
		PDSparseMatrix m_massMatrix;
		PDVector m_massMatrixDiag;
		/* Diagonal matrix that contains the weight of each constraint in m_constraints,
		   unless NO_WEIGHTS_IN_CONSTRUCTION == false, in which case it will be the identity.*/
		PDSparseMatrix m_weightMatrix;
		/* Matrix which maps unassembled constraint auxiliaries to the weighted sum in the rhs */
		PDSparseMatrix m_assemblyMatrix;
		/* Matrix that maps a full vector of unassembled auxiliary vars to a smaller vector containing
		   only those vars that come from the sampled constraints */
		PDSparseMatrix m_selectionMatrix;
		/* Matrix that first maps subspace vertices of unassembled auxiliary variables to full
		   unassembled auxiliary varibles, then assembles unassembled auxiliaries (applies the selection matrices, sums
		   the results and applies the weights). */
		PDSparseMatrix m_finalizeMatrixBig;
		/* Matrix that first maps subspace vertices of unassembled auxiliary variables to full
		   unassembled auxiliary varibles, then assembles unassembled auxiliaries (applies the selection matrices, sums
		   the results and applies the weights) and then applies U^T, where U is the matrix used for
		   the positions in the simulation. */
		PDMatrix m_finalizeMatrix;
		/* Solver that computes the inteprolation of a subset of evaluated constraints into
		   the subspace spanned by m_basis. */
		Eigen::LLT<PDMatrix> m_interpolSolver;
		/* When solving the interpolation problem, this matrix needs to be multiplied to the
		vector containing the auxiliary variables of the sampled constraints and the result
		is the rhs of the problem. */
		PDMatrix m_interpolRHSMatrix;
		/* Solver that projects the vector of ALL unassembled constraints into the interpolation subspace. */
		Eigen::LLT<PDMatrix> m_projectionSolver;
		/* Needs to be aplied to the result of the projection solver before adding it to the rhs vector.*/
		PDSparseMatrix m_projectionFinalizeMatrix;
		/* Determines whether m_finalizeMatrix or m_finalizeMatrixBig is being used */
		bool m_usingPositionSubspace;
		/* Temporary vectors used to store the auxiliary variables and solution of the interpolation solver */
		PDPositions m_tempSolution;
		PDPositions m_tempAuxils;
		PDPositions m_tempRHS;
		/* Determines whether the interpolation matrices are initialized and up to date. */
		bool m_interpolReady;
		/* Only true if the basis for the constraint projections has been computed and is up to date. */
		bool m_basisReady;
		/* Name of the group, used for the file name of the save */
		std::string m_groupName;
		/* LHS term for the constraints in this group */
		PDMatrix m_lhsMatrixSubspace;
		bool m_lhsMatrixSubspaceInitialized = false;
		/* Factor by which the constraints in this group are multiplied */
		PDScalar m_weightFac = 1.;
		/* Regularization parameter: rhs from current frame should also be close to rhs of last frame
		   weighted by this factor against the interpolation problem. */
		PDScalar m_regularizationWeight;
		PDMatrix m_regularizationRHSMatrix;
		PDPositions m_solutionLastFrame;
		bool m_isTetExampleGroup = false;
		PDScalar m_blowUp = 1.;

	public:
		/* Full constructor that initializes the full set of constraints and constructs a mass matrix for them */
		RHSInterpolationGroup(std::string groupName, std::vector<ProjDynConstraint*>& constraints, PDPositions& initialPos,
			PDVector& vertexMasses, PDTriangles& tris, PDTets& tets, PDScalar regularizationWeight);

		/* Lightweight constructor that initializes an empty group with just a name, from which a basis might be
		   loaded, and samples may be suggested from that basis.
		   Use this, in case constraints have not yet been defined, but are anticipated, and a sampling is needed
		   already.
		   The auxiliary size (i.e. number of rows of the constraint projection variables)
		   will be required to determine the indices of the samples in the suggestSamples()
		   function.*/
		RHSInterpolationGroup(std::string groupName, unsigned int auxiliarySize);
		/* Creates a basis from the (PCA of) skinning construction of unassembled auxilary variables. */
		void createBasisViaSkinningWeights(unsigned int size, PDPositions& restPos, PDMatrix& skinningWeights,
			bool usePCA, PDTriangles& tris, PDTets &tets);
		/* Directly sets a basis. */
		void setBasis(PDMatrix& base);
		/* Performs a PCA on the vectors listed in columns of Y, weighted by the mass matrix M.
		   Uses the method of snapshots.
		   Returns a basis for the vectors in Y, whose number of columns is specified as [size]. */
		static PDMatrix snapshotPCA(PDMatrix& Y, PDVector& M, unsigned int size);
		/* Builds all matrices to solve the interpolation problem (i.e. to use interpolate() and approximateRHS() ).
		   Requires a list of sampled vertices/triangles/tets, depending on which type of elements the
		   constraints are based.
		   The finalize matrix will first map the subspace auxiliaries to full, unassembled auxiliaries,
		   then assemble them and then apply the transpose of the subspace matrix (if provided).
		   If a subspace for vertex positions is being used, the transposed subspace matrix  will be pre-multiplied
		   in order  to obtain small, dense finalization matrices. */
		void initInterpolation(unsigned int numVertices, std::vector<unsigned int>& sampledElements,
			PDSparseMatrix const& positionSubspaceT = PDSparseMatrix(0,0));
		/* Solves the interpolation problem for this group of constraints:
		   Evaluates the sampled constraints, solves the interpolation problem, 
		   and applies the finalization matrix.
		   The rhs should have the correct size (full dimension or dimension of subspace) and
		   either be all 0s or filled with the result of other interpolation groups. */
		void approximateRHS(PDPositions& pos, PDPositions& rhs, bool* collidedVertices);
		/* For testing the accuracy of the approximation in approximateRHS(), this function evaluates all
		   constraints in this group and a projection into the interpolation space and back is only
		   optional. */
		void evaluateRHS(PDPositions & pos, PDPositions & rhs, bool projectToInterpolationSpaceAndBack, bool forceFull = false);
		/* Only solves the interpolation problem by evaluating the sampled constraints and filling the vector
		   interpol with the unassembled auxiliarys. */
		void interpolate(PDPositions& pos, PDPositions& interpol, bool* collidedVertices);
		/* Computes and returns the matrix that needs to be added to the LHS matrix in the global system
		   to account for the constraints in this group. */
		PDMatrix getLHSMatrixSubspace(PDMatrix& posSubspaceMat, PDMatrix& posSubspaceMat_T);
		/* Computes and returns the matrix that needs to be added to the LHS matrix in the global system
		to account for the constraints in this group. */
		PDSparseMatrix getLHSMatrix();
		/* For the current constraints computes the mass matrix that assigns a weight to each 
		   constraint, associated to that constraint's element's area. */
		void computeMassMatrix(PDVector& vertexMasses, PDTriangles& tris, PDTets& tets);
		bool hasBasis();
		/* Returns a reference to the constraints used in the interpolation of this group of
		   constraints for the rhs.*/
		std::vector<ProjDynConstraint*>& getSampledConstraints();
		/* Returns a reference to all constraints used associated to this group */
		std::vector<ProjDynConstraint*>& getConstraints();

		/* IF this group is made up of TetExampleBased constraints, then this
		   will modify the weights of all the sampled constraints */
		void setExampleWeights(std::vector<PDScalar>& weights);

		void setWeightFactor(PDScalar w);
		void setBlowup(PDScalar s);

		std::string getName();
	};

}