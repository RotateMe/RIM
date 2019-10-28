#pragma once

//#define PF_FULL_PROJECTION

#define FLUID_COLOR_R 0
#define FLUID_COLOR_G 0.635
#define FLUID_COLOR_B 0.9098

#define SOLID_COLOR_R 0.5
#define SOLID_COLOR_G 0.25
#define SOLID_COLOR_B 0.25

#define COLOR_DEV 0.05

#define PF_BOUNDARY_EPS 1e-4
#define PF_BOUNDARY_LARGE_EPS 3.5

void updateParticleNormals_PF(float* particleNormalsOrig, int maxNNZ, int* nnzIndices, float* nnzWeights, int subCoordsRows, float* subspaceCoords, float* particleNormals);

void initGrid_PF(int numParticles, int numFreeParticles, float cellLength, float timeStepInitial, int cellsX, int cellsY, int cellsZ, float cornerX, float cornerY, float cornerZ, bool hasNormals, bool useNormalsForSolids);
void setParameters_PF(float timeStep, float gravity, float flipness, float maxDensity, float densityCorrection,
	float densityFluid, float densitySolid, float frictionFac, float solidInfluence = 0.5, int jacobiIts = 50,
	float colorDeviation = 0.1, float maxCorrection = 0.5, float velStab = 0);
void setTimeStep_PF(float timeStep);
void sparseCoordinateEvaluation_PF(int rows, int cols, int maxNumEntries, int* indices, float* entries, float* subspaceCoords, float* destParticles);

void particleVelocitiesToGrid_PF(float* particlePositions, float* particleVelocities, float* intermediateSolidVelocities, float* gridVelocities, float* gridVelocitiesOrig, float* gridWeights, float* gridDensities, int* particleList, int* cellList, int* gridListIDs, float* particleNormals, bool* gridSolid);
std::pair<float, float> correctGridVelocities_PF(float* gridVelocities, float* gridVelocitiesOrig, float* gridWeights, float* gridDensities, int* ppcData, float* gridDivergence, float* gridPressure1, float* gridPressure2);
void advectParticles_PF(float* gridVelocities, float* gridVelocitiesOrig, float* randomVelos, float* particlePositions, float* particleVelocities, bool dontAdvectFluids = false, float* particleNormals = nullptr, bool* gridSolid = nullptr, int* ppcData = nullptr, int* particleList = nullptr, float* intermediateSolidVelos = nullptr, float* particleColors = nullptr, float* origPositions = nullptr);
void setMeshVelocities_PF(float* particlePositions, int numMeshParticles, float* newMeshPositions, float* intermediateSolidVelocities);
void boundMeshPositions_PF(float* particlePositions, int numMeshParticles);

void makeParticleColors(float* particleVelocities, float* solidParticleNormals, float* particleColors);