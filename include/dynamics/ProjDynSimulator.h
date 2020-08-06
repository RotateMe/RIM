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

#include "helpers/OpenGLBufferObject.h"
#include "helpers/OpenGLTextureObject.h"
#include "helpers/OpenGLFramebufferObject.h"
#include "helpers/OpenGLProgramObject.h"
#include "helpers/CameraFPS.h"
#include "gui/BaseGUI.h"

// Use CUDA/CUBLAS for precomputation and/or for vertex position
// updates. Read the _README.txt for details.
// This needs to be defined if either using it in pre or rest
#define PROJ_DYN_USE_CUBLAS
#define PROJ_DYN_USE_CUBLAS_IN_PRE
#define PROJ_DYN_USE_CUBLAS_REST

#ifdef PROJ_DYN_USE_CUBLAS
#include "CUDAMatrixVectorMult.h"
#endif

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseCholesky"
#ifdef PROJ_DYN_USE_CHOLMOD
#include "Eigen/CholmodSupport"
#endif

#include "StopWatch.h"

#include "ProjDynTypeDef.h"
#include "ProjDynMeshSampler.h"
#include "ProjDynRHSInterpol.h"

#include "SPH.h"


class PDPICFLIP;

// If set to true, we sparsify the matrices used in the global step and for interpolation in the subspace
#define PROJ_DYN_SPARSIFY true
// Corrects mass of very small triangles to have a minimal mass
#define PROJ_DYN_MIN_MASS 1e-10f
// Entries below this value will be removed from matrices when sparsifying them
#define PROJ_DYN_SPARSITY_CUTOFF 1e-12
// Entries below this value will be removed from matrices when sparsifying high precision matrices
#define PROJ_DYN_SPARSITY_CUTOFF_HIGH_PREC 1e-20
// If using parallel blocks for updating vertex positions, this defines the size of those
#define PROJ_DYN_VPOS_BLOCK_SIZE 1000
// If defined, meshes with textures don't work, but faster vertex mappings
#define PROJ_DYN_SIMPLE_MESH
// Width of boundary walls being generated when doing grid steps
#define PROJ_DYN_WALL_WIDTH 0.05

#ifdef PROJ_DYN_DETERMINANT_LOG
	#define PROJ_DYN_NUM_THREADS 1
	#define PROJ_DYN_PARALLEL_FOR 
#else
	#define PROJ_DYN_NUM_THREADS 8
	#define PROJ_DYN_PARALLEL_FOR __pragma(omp parallel for num_threads(PROJ_DYN_NUM_THREADS)) 
#endif

#define PROJ_DYN_EIGEN_NUM_THREADS 1

namespace PD {

	typedef Eigen::SimplicialLDLT<PD::PDSparseMatrix> PDSparseSolver;

	struct Edge {
		unsigned int v1, v2;
		int t1, t2, vOtherT1, vOtherT2;
	};

	void errorExit(std::string msg);

	class ProjDynConstraint;
	class CenterConstraint;
	class TetExampleBased;
	class PositionMultiConstraint;
	class PositionConstraint;

	class PDSparseVector;

	enum CollisionType {
		sphere,
		block,
		floor
	};

	class CollisionObject {
	private:
		int m_type;
		PD3dVector m_pos;
		PDScalar m_s1, m_s2, m_s3;
	public:
		CollisionObject();
		CollisionObject(int type, PD3dVector& pos, PDScalar s1, PDScalar s2, PDScalar s3);
		bool resolveCollision(PD3dVector& desiredPos);
	};

	class ProjDynSimulator {

	public:
		ProjDynSimulator(PDTriangles& triangles,
			PDPositions& initialPositions,
			PDPositions& initialVelocities, PDScalar timeStep,
			int numSamples = -1,
			PDScalar baseFunctionRadius = 2.5,
			int interpolBaseSize = 120,
			PDScalar rhsInterpolWeightRadius = 2.,
			int numConstraintSamples = -1,
			PDScalar massPerUnitArea = 1,
			PDScalar dampingAlpha = 0,
			bool makeTets = false,
			std::string meshURL = "",
			PDScalar rhsRegularizationWeight = 0.,
			PDScalar yTranslation = 0,
			PDScalar tetMeshQ = 1.5,
			PDScalar gripWeight = 10,
			int initialNumIterations = 10);

		class GUI : public BaseGUI
		{
		public:
			GUI(ProjDynSimulator* pdsim = nullptr);
			virtual ~GUI();
			virtual void draw(GLFWwindow* window, int order = 0);
		private:
			ProjDynSimulator* pdsimptr;
		};

		void step(int numIterations = -1);
		PDPositions& getPositions();
		PDPositions& getVelocities();
		int getNumVertices();
		void addConstraint(ProjDynConstraint* c, bool alwaysAdd = false);
		void addConstraintDynamic(ProjDynConstraint* c);
		void addHangConstraint(PositionConstraint* c);
		void removeConstraintDynamic(ProjDynConstraint* c);
		void setup();
		void setExternalForces(PDPositions fExt);
		void addGravity(PDScalar g, int gravityCoord = 1);
		void addFloor(int floorCoordinate, PDScalar floorHeight, PDScalar floorCollisionWeight, PDScalar ceilingHeight = 1000);
		void addEdgeSprings(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax);
		void addTriangleStrain(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax);
		void addTetStrain(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax, PDScalar tetIncWeight = 0.);
		void addBendingConstraints(PDScalar weight, bool preventBendingFlips, bool flatBending);
		void setFrictionCoefficient(PDScalar coeff, PDScalar rCoeff = -1.);
		void printTimeMeasurements();
		void addHandleConstraint(CenterConstraint* cc);
		void changeTimeStep(PDScalar newTimeStep);
		void resetPositions();
		void resetVelocities();

		void setGrip(std::vector<unsigned int>& vInds, PDPositions gripPos);
		void releaseGrip();
		std::vector<unsigned int>& getUsedVertices();
		PDPositions& getUsedVertexPositions();
		void setGripType(bool centerGrip);

		void addConstraintSample(ProjDynConstraint* c);
		void setExamplePoses(std::vector<PDPositions> exPoses, PDScalar weight, bool forSprings = false);
		PDPositions extendSurfaceDeformationToTets(PDPositions& surfacePos);
		void setExampleWeights(std::vector<PDScalar>& exWeights);
		~ProjDynSimulator();

		void setParallelVUpdateBlockSize(int);
		void setEnableVUpdate(bool);

		bool m_useSparseMatricesForSubspace;
		void initializeGPUVPosMapping(GLuint bufferId);


		void addCollisionsFromFile(std::string fileName);
		void setEnforceCollisionFreeDraw(bool enable);

		void setStiffnessFactor(PDScalar w);

		PDScalar evaluateEnergy(PDPositions& q, PDPositions& s);

		void setInitialPos(PDPositions& startPos);
		void setBlowup(PDScalar enable);

		void addPICFLIP(int numParticlesLongestSide, bool preventSolidPenetration = false, bool addPool = false, bool addStream = false, double poolGap = 0,
			double poolHeight = 0.5, double gridExpac = 0, double solidSamplingFac = 1., bool useNormals = false, float forceFloorHeight = -9999, bool showMeshParticles = false,
			float streamSize = 1.);
		void setPICFLIPParams(float maxDensity, float densityCorrection, float flipness, float sphRegularizationWeight = 0.1, bool skipGridStep = false, float fluidDensity = 0.5,
			float velocityRestore = 0., int maxSSFluid = 20, int maxSSSolid = 20, int pfStepsPerFrame = 1, float frictionFac = 0, float solidInfluence = 0.5, int jacobiIts = 50,
			float colorDeviation = 0.1, float maxDCorrection = 0.5, bool skipIncompressibility = false);

		const PDPositions& getParticles();
		bool hasParticles();

		GLuint m_particleBufferID = 0;
		GLuint m_particleColorBufferID = 0;
		int getNumFreeParticles();
		int getNumParticles();

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		bool m_positionsUpdated;

		PDScalar m_initPosNorm;

		std::string m_meshURL;

		PDScalar m_blowupStrength = 0;
		PDPositions m_vertexNormals;

		bool m_collisionFreeDraw = false;
		/*  Only one vertex per sampled constraint is selected.
		Used for self-collision detection. */
		std::vector< unsigned int > m_usedVerticesSlim;
		std::vector< unsigned int > m_slimToUsedIndices;
		PDTets m_simpleTets;

		std::vector< CollisionObject > m_collisionObjects;
		void handleCollisionsUsedVs(PDPositions& s, bool update);

		void refreshLHS();

		/* Positions and velocities, describing the state of the system */
		PDPositions m_positions;
		PDPositions m_velocities;
		/* Mesh information */
		int m_numTriangles, m_numVertices, m_numOuterVertices, m_numTets;
		/* Indices of corners of the triangles that make up the surface connectivity */
		PDTriangles m_triangles;
		/* Edge list, only available if addEdgeSpring was called */
		std::vector< std::pair< unsigned int, unsigned int > > m_edges;
		/* If available, indices of the corners of the tetrahedrons that make up
		the volumetric connectivity */
		PDTets m_tetrahedrons;
		bool m_hasTetrahedrons;
		/* Normalization multiplier for areas and volumes associated to vertices
		and elements such that they are 1 on average. */
		PDScalar m_normalization;

		ProjDynMeshSampler m_sampler;

		void makeEdges();

		bool m_flatBending;
		PDScalar m_gripWeight;

		/* Example poses stuff */
		std::vector<PDPositions> m_exPoses;
		std::vector<ProjDynConstraint*> m_tetExConstraints;

		/* Surface deformation extension stuff */
		bool m_surfDefExtInit = false;
		PDSparseMatrix m_surfDefExtRHSMat;
		PDSparseMatrix m_surfDefExtFixedPartMat;
		PDSparseSolver m_surfDefExtSolver;
		std::vector<ProjDynConstraint*> m_surfDefExtConstraints;

		int m_parallelVUpdateBSize;
		bool m_parallelVUpdate;
		void updateParallelVUpdateBlocks();

#ifdef PROJ_DYN_USE_CUBLAS
		CUDAMatrixVectorMultiplier* m_vPosGPUUpdate = nullptr;
#endif

		int m_floorCoordinate;
		PDScalar m_floorCollisionWeight;
		PDScalar m_floorHeight;
		PDScalar m_ceilingHeight;
		std::vector< std::vector< Edge > > m_vertexStars;
		std::vector< std::vector< unsigned int > > m_tetsPerVertex;
		/* Additional external per-vertex forces like wind, interavtive "grabbing", ..., can be set
		with setExternalForces(fExt) */
		PDPositions m_fExt;
		/* Gravitational per-vertex forces, can be added with addGravity(massPerUnitArea) */
		PDPositions m_fGravity;
		/* Contains external forces, gravitational forces and friction forces, all weighted by inverse
		vertex masses and multiplied by m_timeStep squared, such that they can be directly added to s
		in the step() method */
		PDPositions m_fExtWeighted;
		PDPositions m_fGravWeighted;
		PDScalar m_frictionCoeff;
		PDScalar m_repulsionCoeff;

		// Lists of constraints
		/* ALL constraints in the system */
		std::vector<ProjDynConstraint*> m_constraints;
		/* All strain constraints */
		std::vector<ProjDynConstraint*> m_strainConstraints;
		/* All spring constraints */
		std::vector<ProjDynConstraint*> m_springConstraints;
		/* All bending constraints */
		std::vector<ProjDynConstraint*> m_bendingConstraints;
		/* All collision constraints */
		std::vector<ProjDynConstraint*> m_collisionConstraints;
		/* All tetrahedral strain constraints */
		std::vector<ProjDynConstraint*> m_tetStrainConstraints;
		/* All handle constraints */
		std::vector<CenterConstraint*> m_handleConstraints;
		/* All hang constraints, will later be filtered using only sample vertices */
		std::vector<PositionConstraint*> m_hangConstraints;

		PDSparseMatrix m_lhsMatrix;
		PDSparseMatrix m_lhsCompareMat;
		PDSparseSolver m_lhsCompareSolver;
		PDVector m_rhsMasses;
		PDSparseMatrix m_massMatrix;
		PDSparseMatrix m_reducedMassMatrix;
		PDVector m_vertexMasses;
		PDScalar m_totalMass;
		PDScalar m_timeStep;
		bool m_isSetup;
		PDSparseSolver m_linearSolver;
		Eigen::LLT<PDMatrix> m_denseSolver;
		void recomputeWeightedForces();
		StopWatch m_precomputationStopWatch;
		StopWatch m_localStepStopWatch;
		StopWatch m_globalStepStopWatch;
		StopWatch m_totalStopWatch;
		StopWatch m_localStepOnlyProjectStopWatch;
		StopWatch m_localStepRestStopWatch;
		StopWatch m_surroundingBlockStopWatch;
		StopWatch m_updatingVPosStopWatch;
		StopWatch m_constraintSummationStopWatch;
		StopWatch m_momentumStopWatch;
		StopWatch m_multiplicationForPosUpdate;
		StopWatch m_sortingForPosUpdate;
		StopWatch m_fullUpdateStopWatch;
		StopWatch m_picflipStopWatch;
		StopWatch m_particleTransferStopWatch;
		StopWatch m_elasticityStopWatch;
		int m_numIterations;
		int m_numRefactorizations;
		bool once;
		bool m_recomputeFactorization;

		PDPositions m_positionCorrectionsUsedVs;
		PDPositions m_positionCorrections;
		PDPositions m_velocitiesUsedVs;
		bool m_collisionCorrection;
		PDScalar m_rayleighDampingAlpha;

		PDPositions m_initialPos;
		PDPositions m_initialPosSub;

		std::vector<unsigned int> m_grippedVertices;
		PDPositions m_grippedVertexPos;
		int* m_usedVertexMap;
		bool m_useCenterConstraintGrip = true;

		CenterConstraint* m_gripCenterConstraint = nullptr;
		PositionMultiConstraint* m_gripPosConstraint = nullptr;

		std::vector< unsigned int > m_additionalUsedVertices;

		void createPositionSubspace(unsigned int numSamples, bool useSkinningSpace = true);
		PDMatrix createSkinningWeights(unsigned int numSamples, PDScalar r);
		bool m_usingSubspaces;
		std::vector< unsigned int > m_samples;
		PDMatrix m_baseFunctions;
		PDMatrix m_baseFunctionWeights;
		PDMatrix m_baseFunctionsTransposed;
		PDMatrix m_baseFunctionsSquared;
		PDMatrix m_rhsFirstTermMatrix;
		PDPositions m_fExtWeightedSubspace;
		PDPositions m_fGravWeightedSubspace;
		PDScalar m_baseFunctionRadius;
		PDScalar m_usedRadius;
		PDMatrix m_lhsMatrixSampled;
		PDMatrix m_lhsMatrixSampledStiff;
		PDMatrix m_rhsCollisionTerm;
		PDMatrix m_rhsStabTerm;
		PDPositions m_positionsSubspace;
		PDPositions m_positionsUsedVs;
		PDPositions m_velocitiesSubspace;
		Eigen::LLT<PDMatrix> m_subspaceSolver;
		Eigen::LLT<PDMatrix> m_usedVertexInterpolator;
		PDMatrix m_usedVertexInterpolatorRHSMatrix;
		void projectToSubspace(PDPositions& b, PDPositions& x);

		PDSparseMatrix m_rhsFirstTermMatrixSparse;
		PDSparseMatrix m_baseFunctionsTransposedSparse;
		PDSparseMatrixRM m_baseFunctionsSparse;
		std::vector< PDSparseMatrixRM > m_baseFunctionsSparseBlocks;
		PDSparseMatrix m_usedVertexInterpolatorRHSMatrixSparse;
		PDSparseSolver m_subspaceSystemSolverSparse;
		PDSparseSolver m_usedVertexInterpolatorSparse;

		PDMatrix m_subspaceLHS_mom;
		PDMatrix m_subspaceLHS_inner;
		PDMatrix m_rhsFirstTermMatrixPre;

		bool* m_collidedVerts;

		void finalizeBaseFunctions();

		PDPositions m_rhs;

		PDPositions rhs;
		PDPositions rhs2;

		float m_stiffnessFactor = 1.;

		int m_numSamplesPosSubspace;

		// RHS Interpolation stuff
		int m_numConstraintSamples;
		std::vector<unsigned int> m_allVerts;
		void createConstraintSampling(unsigned int numSamples);
		/* List of vertex indices at which constraints are being sampled. I.e. if a constraint contains
		this vertex, it is evaluated. Note that this means that there are more vertices involved in
		the sampled constraints than just the ones in this list, which is where the list m_usedVertices
		comes in. */
		std::vector< unsigned int > m_constraintVertexSamples;
		/* List of triangle indices at which constraints are being sampled. For each vertex, one adjacent
		triangle is being selected (basically arbitrarily). */
		std::vector< unsigned int > m_constraintTriSamples;
		/* List of tetrahedron indices at which constraints are being sampled. For each vertex, one adjacent
		tetrahedron is being selected (basically arbitrarily). */
		std::vector< unsigned int > m_constraintTetSamples;
		/* List of edge indices at which constraints are being sampled. For each vertex, one adjacent
		edge is being selected (basically arbitrarily). */
		std::vector< unsigned int > m_constraintEdgeSamples;
		/* When using constraint sampling, this list contains all vertices that are involved in
		the sampled constraints. This is different from the list m_constraintSamples,
		which contains only the vertices at which constraints were sampled.
		MOST IMPORTANTLY this list contains the non-zero rows of the sum on the r.h.s of the
		linear system.*/
		std::vector< unsigned int > m_usedVertices;
		/* The list of constraints that are being sampled.
		These get chosen by the rhs interpolation groups*/
		std::vector< ProjDynConstraint* > m_sampledConstraints;
		bool m_constraintSamplesChanged;

		/* Resolves collisions for the vertex with index v in the vector pos and writes the
		correction vector into row v of posCorrect*/
		void resolveCollision(unsigned int v, PDPositions& pos, PDPositions& posCorrect);

		/* Vertices at which collisions are being resolved, required to build list of "used vertices" */
		//std::vector< unsigned int > m_collisionSamples;

		std::vector< PDPositions > m_additionalConstraintsAuxTemp;

		/* Updates positions at all vertices used by sampled constraints */
		void updatePositionsSampling(PDPositions& fullPos, PDPositions& subPos, bool usedVerticesOnly);
		void evaluatePositionsAtUsedVertices(PDPositions& fullPos, PDPositions& subPos);
		std::vector< std::vector<PDScalar> > m_usedVerticesBase;
		std::vector< std::vector< unsigned int > > m_usedVerticesBaseNNZ;

		/* Updates the list of used vertices (NOT the positions at used vertices) */
		void updateUsedVertices();

		/* Whether we only evaluate a few samples of the constraints and interpolate
		the full rhs of the global step using some subspace for the rhs. */
		bool m_rhsInterpolation;
		void addAdditionalConstraints(PDPositions& pos, PDPositions& rhs, bool* collidedVertices);
		std::vector<ProjDynConstraint*> m_additionalConstraints;
		std::vector<RHSInterpolationGroup> m_snapshotGroups;

		void initRHSInterpolGroup(RHSInterpolationGroup& g, std::vector<unsigned int>& samples, PDMatrix& hessian);
		int m_rhsInterpolBaseSize;

		PDScalar m_rhsRegularizationWeight;
		PDScalar m_rhsInterpolWeightRadiusMultiplier;
		PDMatrix m_rhsInterpolReusableWeights;

		/* Count of frames (full simulation steps) since the start of the simulation */
		int m_frameCount;

		PDScalar m_lastRHSNorm = 0;

		// Fluid Variables
		PDPositions m_particles;
		PDPositions m_particlesV;
		PDPositions m_particlesFGrav;
		PDPositions m_particlesInitial;
		PDScalar m_sphConstraintWeight = 0;
		PDMatrix m_sphLHS;
		PDMatrix m_sphLHS_rest;
		bool m_sphAfterElasticity = false;
		int m_numSPHIterations = 10;
		int m_numMeshParticles = 0;
		PDScalar m_gravity = 0;
		float m_fluidDensity = 0.5;
		PDScalar m_velocityRestore = 0.;

    	//PICFLIP Stuff
		PDPICFLIP* m_picflipsimulator;
		float m_maxDensity;
		float m_densityCorrection;
		float m_flipness;
		bool m_skipGridStep = false;
		bool m_skipIncompressibility = false;
		bool m_didSkipGridStep = true;
		int m_maxSSFluid;
		int m_maxSSSolid;
		int m_pfStepsPerFrame = 1;
		float m_pfFriction = 0;
		float m_solidInfluence = 0.5;
		int m_jacobiIts = 50;
		int m_numFreeParticles = 0;
		float m_colorDeviation = 0.1;
		float m_maxDCorrection = 0.5;


#ifdef PROJ_DYN_USE_CUBLAS
		// CUDA stuff
		CUDAMatrixVectorMultiplier* m_usedVertexUpdater;
		CUDAMatrixVectorMultiplier* m_rhsEvaluator;
		PDMatrix m_projUsedVerts;
		PDMatrix m_rhsEvalMat;
		PDVector m_curTempVec;
#endif
	};

	struct SparseEntry {
		unsigned int i, j;
		double entry;
	};

}