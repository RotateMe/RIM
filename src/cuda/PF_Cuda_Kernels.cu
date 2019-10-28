#include "device_launch_parameters.h"
#include "math.h"
#include <cuda_runtime.h>
#include <iostream>
#include "dynamics/CudaSafeCall.h"
#include "cuda/PF_Cuda_Kernels.cuh"
#include "thrust/device_ptr.h"
#include "thrust/sort.h"
#include "curand_kernel.h"

typedef struct {
	float x, y, z;
} vec3_PF;

typedef struct {
	vec3_PF oldV, newV;
} twoVelos_PF;

#define PF_CUDA_KERNEL_DEBUG_OUTPUT
#define DebugOutputKernel_PF( txt, x ) __debugOutputKernel_PF( txt, x, i )
__device__
inline void __debugOutputKernel_PF(const char* txt, float x, int i)
{
#ifdef PF_CUDA_KERNEL_DEBUG_OUTPUT
	if (i == 0)
		printf(txt, x);
#endif
	return;
}


__device__ __constant__ int layerSize_PF;
__device__ __constant__ int layerSizeMAC_PF;
__device__ __constant__ float cellLength_PF;
__device__ __constant__ float boundXMin_PF;
__device__ __constant__ float boundXMax_PF;
__device__ __constant__ float boundYMin_PF;
__device__ __constant__ float boundYMax_PF;
__device__ __constant__ float boundZMin_PF;
__device__ __constant__ float boundZMax_PF;
__device__ __constant__ int numParticles_PF;
__device__ __constant__ int numFreeParticles_PF;
__device__ __constant__ float timeStep_PF;
__device__ __constant__ float gravity_PF;
__device__ __constant__ float flipness_PF;
__device__ __constant__ float maxDensity_PF;
__device__ __constant__ float densityCorrection_PF;
__device__ __constant__ float maxDensityCorrection_PF;
__device__ __constant__ float densityFluid_PF;
__device__ __constant__ float densitySolid_PF;
__device__ __constant__ float frictionFactor_PF;
__device__ __constant__ float colorDeviation_PF;
__device__ __constant__ int maxNumSubstepsFluid_PF;
__device__ __constant__ int maxNumSubstepsSolid_PF;
__device__ __constant__ float solidInfluence_PF;
__device__ __constant__ bool hasNormals_PF;
__device__ __constant__ bool useNormalsForSolids_PF;
__device__ __constant__ int gridSizeX_PF;
__device__ __constant__ int gridSizeY_PF;
__device__ __constant__ int gridSizeZ_PF;
__device__ __constant__ float boundaryEps_PF;
__device__ __constant__ float boundaryEpsLarge_PF;
__device__ __constant__ float boundaryEpsLargeFloor_PF;
__device__ __constant__ float velStab_PF;

int gridSizeHost_PF;
int jacobiItsHost_PF;
bool hasNormalsHost_PF;
int numParticlesHost_PF;
int numFreeParticlesHost_PF;
bool useNormalsForSolidsHost_PF = true;

dim3 gridKernelGridDim_PF, gridKernelBlockDim_PF;
dim3 macGridKernelGridDim_PF, macGridKernelBlockDim_PF;
dim3 particleKernelGridDim_PF, particleKernelBlockDim_PF;

__device__ __forceinline__
int minI_PF(int x, int y) {
	return ((x < y) ? x : y);
}

__device__ __forceinline__
int maxI_PF(int x, int y) {
	return ((x < y) ? y : x);
}

__device__ __forceinline__
int flattenIndex_PF(int xInd, int yInd, int zInd) {
	return xInd + yInd * gridSizeX_PF + zInd * layerSize_PF;
}

__device__ __forceinline__
int flattenIndexBound_PF(int xInd, int yInd, int zInd) {
	return maxI_PF(0, minI_PF(gridSizeX_PF - 1, xInd)) + maxI_PF(0, minI_PF(gridSizeY_PF - 1, yInd)) * gridSizeX_PF + maxI_PF(0, minI_PF(gridSizeZ_PF - 1, zInd)) * layerSize_PF;
}

__device__ __forceinline__
int getGridCellID_PF(int i, float* posParticles) {
	// Compute x, y and z indices
	float x = posParticles[3 * i + 0];
	float y = posParticles[3 * i + 1];
	float z = posParticles[3 * i + 2];
	x -= boundXMin_PF;
	x /= cellLength_PF;
	int xInd = (int)minI_PF(gridSizeX_PF - 1, maxI_PF(0, floor(x)));
	y -= boundYMin_PF;
	y /= cellLength_PF;
	int yInd = (int)minI_PF(gridSizeY_PF - 1, maxI_PF(0, floor(y)));
	z -= boundZMin_PF;
	z /= cellLength_PF;
	int zInd = (int)minI_PF(gridSizeZ_PF - 1, maxI_PF(0, floor(z)));

	return flattenIndex_PF(xInd, yInd, zInd);
}

__device__ __forceinline__
int getGridCellID_PF(float x, float y, float z) {
	// Compute x, y and z indices
	x -= boundXMin_PF;
	x /= cellLength_PF;
	int xInd = (int)minI_PF(gridSizeX_PF - 1, maxI_PF(0, floor(x)));
	y -= boundYMin_PF;
	y /= cellLength_PF;
	int yInd = (int)minI_PF(gridSizeY_PF - 1, maxI_PF(0, floor(y)));
	z -= boundZMin_PF;
	z /= cellLength_PF;
	int zInd = (int)minI_PF(gridSizeZ_PF - 1, maxI_PF(0, floor(z)));

	return flattenIndex_PF(xInd, yInd, zInd);
}


__device__ __forceinline__
int flattenIndexMAC_PF(int xInd, int yInd, int zInd) {
	return xInd + yInd * (gridSizeX_PF + 1) + zInd * layerSizeMAC_PF;
}

__device__ __forceinline__
float spike(float r) {
	if (r >= 0.0 && r <= 1.0) {
		return 1.0 - r;
	}
	else if (r >= -1.0 && r <= 0.0) {
		return 1.0 + r;
	}
	else {
		return 0.0;
	}
}

__device__ __forceinline__
float linWeight(float dx, float dy, float dz) {
	return spike(dx) * spike(dy) * spike(dz);
}

void initGrid_PF(int numParticles, int numFreeParticles, float cellLength, float timeStepInitial,
	int cellsX, int cellsY, int cellsZ, float cornerX, float cornerY, float cornerZ, bool hasNormals,
	bool useNormalsForSolids)
{
	float temp = 0;

	CudaSafeCall(cudaMemcpyToSymbol(cellLength_PF, &cellLength, sizeof(float)));

	// Copy grid dims to device
	CudaSafeCall(cudaMemcpyToSymbol(gridSizeX_PF, &cellsX, sizeof(int)));
	CudaSafeCall(cudaMemcpyToSymbol(gridSizeY_PF, &cellsY, sizeof(int)));
	CudaSafeCall(cudaMemcpyToSymbol(gridSizeZ_PF, &cellsZ, sizeof(int)));

	gridSizeHost_PF = cellsX * cellsY * cellsZ;

	int layerSize_PFT = cellsX * cellsY;
	CudaSafeCall(cudaMemcpyToSymbol(layerSize_PF, &layerSize_PFT, sizeof(int)));
	layerSize_PFT = (cellsX + 1) * (cellsY + 1);
	CudaSafeCall(cudaMemcpyToSymbol(layerSizeMAC_PF, &layerSize_PFT, sizeof(int)));


	// Compute and copy bounds to device
	CudaSafeCall(cudaMemcpyToSymbol(boundXMin_PF, &cornerX, sizeof(float)));
	temp = cornerX + cellsX * cellLength;
	CudaSafeCall(cudaMemcpyToSymbol(boundXMax_PF, &temp, sizeof(float)));

	CudaSafeCall(cudaMemcpyToSymbol(boundYMin_PF, &cornerY, sizeof(float)));
	temp = cornerY + cellsY * cellLength;
	CudaSafeCall(cudaMemcpyToSymbol(boundYMax_PF, &temp, sizeof(float)));

	CudaSafeCall(cudaMemcpyToSymbol(boundZMin_PF, &cornerZ, sizeof(float)));
	temp = cornerZ + cellsZ * cellLength;
	CudaSafeCall(cudaMemcpyToSymbol(boundZMax_PF, &temp, sizeof(float)));

	gridKernelBlockDim_PF = dim3(256);
	gridKernelGridDim_PF = dim3((cellsX * cellsY * cellsZ + gridKernelBlockDim_PF.x - 1) / gridKernelBlockDim_PF.x);

	macGridKernelBlockDim_PF = dim3(256);
	macGridKernelGridDim_PF = dim3(((cellsX + 1) * (cellsY + 1) * (cellsZ + 1) + gridKernelBlockDim_PF.x - 1) / gridKernelBlockDim_PF.x);

	particleKernelBlockDim_PF = dim3(256);
	particleKernelGridDim_PF = dim3((numParticles + particleKernelBlockDim_PF.x - 1) / particleKernelBlockDim_PF.x);

	numParticlesHost_PF = numParticles;
	CudaSafeCall(cudaMemcpyToSymbol(numParticles_PF, &numParticles, sizeof(int)));
	CudaSafeCall(cudaMemcpyToSymbol(numFreeParticles_PF, &numFreeParticles, sizeof(int)));
	numFreeParticlesHost_PF = numFreeParticles;

	CudaSafeCall(cudaMemcpyToSymbol(hasNormals_PF, &hasNormals, sizeof(bool)));
	hasNormalsHost_PF = hasNormals;

	CudaSafeCall(cudaMemcpyToSymbol(timeStep_PF, &timeStepInitial, sizeof(float)));

	useNormalsForSolidsHost_PF = useNormalsForSolids;
	CudaSafeCall(cudaMemcpyToSymbol(useNormalsForSolids_PF, &useNormalsForSolidsHost_PF, sizeof(bool)));

	float boundaryEps = cellLength * PF_BOUNDARY_EPS;
	CudaSafeCall(cudaMemcpyToSymbol(boundaryEps_PF, &boundaryEps, sizeof(float)));
	boundaryEps = cellLength * PF_BOUNDARY_LARGE_EPS;
	CudaSafeCall(cudaMemcpyToSymbol(boundaryEpsLarge_PF, &boundaryEps, sizeof(float)));
	boundaryEps = floor(boundaryEps);
	CudaSafeCall(cudaMemcpyToSymbol(boundaryEpsLargeFloor_PF, &boundaryEps, sizeof(float)));
	
}

void setParameters_PF(float timeStep, float gravity, float flipness, float maxDensity, float densityCorrection, float densityFluid, float densitySolid,
	float frictionFac, float solidInfluence, int jacobiIts, float colorDeviation, float maxCorrection, float velStab) {
	CudaSafeCall(cudaMemcpyToSymbol(timeStep_PF, &timeStep, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(gravity_PF, &gravity, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(flipness_PF, &flipness, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(maxDensity_PF, &maxDensity, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(densityCorrection_PF, &densityCorrection, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(densityFluid_PF, &densityFluid, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(densitySolid_PF, &densitySolid, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(frictionFactor_PF, &frictionFac, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(solidInfluence_PF, &solidInfluence, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(colorDeviation_PF, &colorDeviation, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(maxDensityCorrection_PF, &maxCorrection, sizeof(float)));
	CudaSafeCall(cudaMemcpyToSymbol(velStab_PF, &velStab, sizeof(float)));
	jacobiItsHost_PF = jacobiIts;
}

void setTimeStep_PF(float timeStep)
{
	CudaSafeCall(cudaMemcpyToSymbol(timeStep_PF, &timeStep, sizeof(float)));
}

__global__
void sparseCoordinateEvaluationKernel_PF(int rows, int cols, int maxNumEntries, int * indices, float * matEntries, float * xEntries, float * outEntries) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < rows) {
		float xVal = 0, yVal = 0, zVal = 0;
		int offSet = i * maxNumEntries;
		for (int j = 0; j < maxNumEntries; j++) {
			float coef = matEntries[offSet + j];
			int ind = indices[offSet + j];
			xVal += coef * xEntries[ind];
			yVal += coef * xEntries[cols + ind];
			zVal += coef * xEntries[2 * cols + ind];
		}
		outEntries[i * 3 + 0] = xVal;
		outEntries[i * 3 + 1] = yVal;
		outEntries[i * 3 + 2] = zVal;
	}
}

void sparseCoordinateEvaluation_PF(int rows, int cols, int maxNumEntries, int * indices, float * entries, float * subspaceCoords, float * destParticles)
{
	sparseCoordinateEvaluationKernel_PF <<< (rows + 255) / 256, 256 >>> (rows, cols, maxNumEntries, indices, entries, subspaceCoords, destParticles);
	CudaSafeSync();
	CudaCheckError();
}

__forceinline__ __device__
float maxAbs(float a, float b) {
	if (abs(a) > abs(b)) return a;
	return b;
}

__forceinline__ __device__
float minAbs(float a, float b) {
	if (abs(a) > abs(b)) return b;
	return a;
}

__global__
void computeGridVelocitiesKernel_PF(float* particlePositions, float* particleVelocities, float* intermediateSolidVelocities,
	float* gridVelocities, float* gridVelocitiesOrig, float* gridWeights, float* gridDensities,
	int* particleList, int* ppcData, bool* gridSolid) {
	// THIS IS A MAC KERNEL!
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	int ii = i;
	int z = ii / layerSizeMAC_PF;
	ii = ii % layerSizeMAC_PF;
	int y = ii / (gridSizeX_PF + 1);
	int x = ii % (gridSizeX_PF + 1);
	if (x <= gridSizeX_PF && y <= gridSizeY_PF && z <= gridSizeZ_PF) {
		// Gather weights and velocity sums from particles in all neighbour cells:
		float xWeight = 0, yWeight = 0, zWeight = 0;
		float cDensity = 0, cWeight = 0;
		float xVel = 0, yVel = 0, zVel = 0;
		float xVelO = 0, yVelO = 0, zVelO = 0;
		bool cellSolid = false;
		for (int xx = maxI_PF(x - 1, 0); xx < minI_PF(x + 1, gridSizeX_PF); xx++) {
			for (int yy = maxI_PF(y - 1, 0); yy < minI_PF(y + 1, gridSizeY_PF); yy++) {
				for (int zz = maxI_PF(z - 1, 0); zz < minI_PF(z + 1, gridSizeZ_PF); zz++) {
					int gridInd = flattenIndex_PF(xx, yy, zz);
					// For all particles in the current cell:
					int numParts = ppcData[gridInd * 2 + 1];
					for (int partLocInd = 0; partLocInd < numParts; partLocInd++) {
						int partId = particleList[ppcData[gridInd * 2 + 0] + partLocInd];
						bool isSolid = (partId >= numFreeParticles_PF);
						if (hasNormals_PF && isSolid && xx == x && yy == y && zz == z) {
							cellSolid = true;
						}
						float partXInGrid = (particlePositions[partId * 3 + 0] - boundXMin_PF) / cellLength_PF;
						float partYInGrid = (particlePositions[partId * 3 + 1] - boundYMin_PF) / cellLength_PF;
						float partZInGrid = (particlePositions[partId * 3 + 2] - boundZMin_PF) / cellLength_PF;
						// The weights are just tri-linear kernels, centered at the centers of each face, and at the cell center
						//float influence = (isSolid ? solidInfluence_PF : 1. - solidInfluence_PF);
						float curXW = linWeight((float)x - partXInGrid, (float)y + 0.5 - partYInGrid, (float)z + 0.5 - partZInGrid); // *influence;
						xWeight += curXW;
						float curYW = linWeight((float)x + 0.5 - partXInGrid, (float)y - partYInGrid, (float)z + 0.5 - partZInGrid); // *influence;
						yWeight += curYW;
						float curZW = linWeight((float)x + 0.5 - partXInGrid, (float)y + 0.5 - partYInGrid, (float)z - partZInGrid); // *influence;
						zWeight += curZW;
						float curCW = linWeight((float)x + 0.5 - partXInGrid, (float)y + 0.5 - partYInGrid, (float)z + 0.5 - partZInGrid);
						cWeight += curCW;
						cDensity += curCW * (isSolid ? densitySolid_PF : densityFluid_PF);
						// Sum weighted velocities in each component
						// For the original grid velocities we use the last known particles velocities, which
						// do not contain the current external forces or elastic forces
						xVelO += particleVelocities[partId * 3 + 0] * curXW;
						yVelO += particleVelocities[partId * 3 + 1] * curYW;
						zVelO += particleVelocities[partId * 3 + 2] * curZW;
						// For the new grid velocities we modify the particle velocities based on whether they
						// belong to an elastic solid (in which case we exchange them with the velocities where we
						// already applied external forces and elastic forces) 
						// or to a fluid (in which case we have to apply gravity)
						if (isSolid) {
							xVel += intermediateSolidVelocities[(partId - numFreeParticles_PF) * 3 + 0] * curXW;
							yVel += intermediateSolidVelocities[(partId - numFreeParticles_PF) * 3 + 1] * curYW;
							zVel += intermediateSolidVelocities[(partId - numFreeParticles_PF) * 3 + 2] * curZW;
						}
						else {
							xVel += particleVelocities[partId * 3 + 0] * curXW;
							yVel += (particleVelocities[partId * 3 + 1] - timeStep_PF*gravity_PF) * curYW;
							zVel += particleVelocities[partId * 3 + 2] * curZW;
						}

					}
				}
			}
		}

		if (hasNormals_PF) {
			if (x < gridSizeX_PF && y < gridSizeY_PF && z < gridSizeZ_PF) {
				int gridInd = flattenIndex_PF(x, y, z);
				if (cellSolid) {
					gridSolid[gridInd] = true;
				}
				else {
					gridSolid[gridInd] = false;
				}
			}
		}

		// Set grid values:
		if (xWeight > 1e-10) {
			gridVelocities[i * 3 + 0] = xVel / xWeight;
			gridVelocitiesOrig[i * 3 + 0] = xVelO / xWeight;
		}
		else {
			gridVelocities[i * 3 + 0] = 0;
			gridVelocitiesOrig[i * 3 + 0] = 0;
		}
		if (yWeight > 1e-10) {
			gridVelocities[i * 3 + 1] = yVel / yWeight;
			gridVelocitiesOrig[i * 3 + 1] = yVelO / yWeight;
		}
		else {
			gridVelocities[i * 3 + 1] = 0;
			gridVelocitiesOrig[i * 3 + 1] = 0;
		}
		if (zWeight > 1e-10) {
			gridVelocities[i * 3 + 2] = zVel / zWeight;
			gridVelocitiesOrig[i * 3 + 2] = zVelO / zWeight;
		}
		else {
			gridVelocities[i * 3 + 2] = 0;
			gridVelocitiesOrig[i * 3 + 2] = 0;
		}

		gridWeights[i] = cWeight;
		if (cWeight > 1e-10) {
			gridDensities[i] = cDensity / cWeight;
		}
		else {
			gridDensities[i] = densityFluid_PF;
		}

		// Boundary conditions
#ifdef PF_LARGE_BOUNDARIES
		if (x <= boundaryEpsLargeFloor_PF) gridVelocities[i * 3 + 0] = 0;
		if (x >= gridSizeX_PF - boundaryEpsLargeFloor_PF) gridVelocities[i * 3 + 0] = 0;
		if (y <= boundaryEpsLargeFloor_PF) gridVelocities[i * 3 + 1] = 0;
		if (y >= gridSizeY_PF - boundaryEpsLargeFloor_PF) gridVelocities[i * 3 + 1] = 0;
		if (z <= boundaryEpsLargeFloor_PF) gridVelocities[i * 3 + 2] = 0;
		if (z >= gridSizeZ_PF - boundaryEpsLargeFloor_PF) gridVelocities[i * 3 + 2] = 0;
#else
		if (x <= 0) gridVelocities[i * 3 + 0] = 0;
		if (x >= gridSizeX_PF) {
			gridVelocities[i * 3 + 0] = 0;
			//gridVelocities[i * 3 + 1] = 0;
			//gridVelocities[i * 3 + 2] = 0;
		}
		if (y <= 0) gridVelocities[i * 3 + 1] = 0;
		if (y >= gridSizeY_PF) {
			gridVelocities[i * 3 + 1] = 0;
			//gridVelocities[i * 3 + 0] = 0;
			//gridVelocities[i * 3 + 2] = 0;
		}
		if (z <= 0) gridVelocities[i * 3 + 2] = 0;
		if (z >= gridSizeZ_PF) {
			gridVelocities[i * 3 + 2] = 0;
			//gridVelocities[i * 3 + 0] = 0;
			//gridVelocities[i * 3 + 1] = 0;
		}
#endif
		/*
		if (x <= 0 && gridVelocities[i * 3 + 0] < 0) gridVelocities[i * 3 + 0] = 0;
		if (x >= gridSizeX_PF) {
			if (gridVelocities[i * 3 + 0] > 0) gridVelocities[i * 3 + 0] = 0;
			gridVelocities[i * 3 + 1] = 0;
			gridVelocities[i * 3 + 2] = 0;
		}
		if (y <= 0 && gridVelocities[i * 3 + 1] < 0) gridVelocities[i * 3 + 1] = 0;
		if (y >= gridSizeY_PF) {
			if (gridVelocities[i * 3 + 1] > 0) gridVelocities[i * 3 + 1] = 0;
			gridVelocities[i * 3 + 0] = 0;
			gridVelocities[i * 3 + 2] = 0;
		}
		if (z <= 0 && gridVelocities[i * 3 + 2] < 0) gridVelocities[i * 3 + 2] = 0;
		if (z >= gridSizeZ_PF) {
			if (gridVelocities[i * 3 + 2] > 0) gridVelocities[i * 3 + 2] = 0;
			gridVelocities[i * 3 + 0] = 0;
			gridVelocities[i * 3 + 1] = 0;
		}
		*/

		// Friction
		if (frictionFactor_PF > 0) {
			if (x <= 0 || x >= gridSizeX_PF) {
				float vNorm = (gridVelocities[i * 3 + 1] * gridVelocities[i * 3 + 1] + gridVelocities[i * 3 + 2] * gridVelocities[i * 3 + 2]);
				if (vNorm > 1e-10 && isfinite(vNorm)) {
					float fricAcc = (timeStep_PF * gravity_PF * frictionFactor_PF) / vNorm;
					gridVelocities[i * 3 + 1] -= minAbs(gridVelocities[i * 3 + 1], fricAcc * gridVelocities[i * 3 + 1]);
					gridVelocities[i * 3 + 2] -= minAbs(gridVelocities[i * 3 + 2], fricAcc * gridVelocities[i * 3 + 2]);
				}
			}
			if (y <= 0 || y >= gridSizeY_PF) {
				float vNorm = (gridVelocities[i * 3 + 0] * gridVelocities[i * 3 + 0] + gridVelocities[i * 3 + 2] * gridVelocities[i * 3 + 2]);
				if (vNorm > 1e-10 && isfinite(vNorm)) {
					float fricAcc = (timeStep_PF * gravity_PF * frictionFactor_PF) / vNorm;
					gridVelocities[i * 3 + 0] -= minAbs(gridVelocities[i * 3 + 0], fricAcc * gridVelocities[i * 3 + 0]);
					gridVelocities[i * 3 + 2] -= minAbs(gridVelocities[i * 3 + 2], fricAcc * gridVelocities[i * 3 + 2]);
				}
			}
			if (z <= 0 || z >= gridSizeZ_PF) {
				float vNorm = (gridVelocities[i * 3 + 1] * gridVelocities[i * 3 + 1] + gridVelocities[i * 3 + 0] * gridVelocities[i * 3 + 0]);
				if (vNorm > 1e-10 && isfinite(vNorm)) {
					float fricAcc = (timeStep_PF * gravity_PF * frictionFactor_PF) / vNorm;
					gridVelocities[i * 3 + 1] -= minAbs(gridVelocities[i * 3 + 1], fricAcc * gridVelocities[i * 3 + 1]);
					gridVelocities[i * 3 + 0] -= minAbs(gridVelocities[i * 3 + 0], fricAcc * gridVelocities[i * 3 + 0]);
				}
			}
		}
	}
}

__global__
void makeKeyValuePairsKernel_PF(float* particlePositions, int* particleList, int* cellList) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF) {
		particleList[i] = i;
		cellList[i] = getGridCellID_PF(i, particlePositions);
	}
}

__global__
void makePPCDataKernel_PF(int* cellList, int* pPerCellData) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF) {
		if (i == 0 || cellList[i - 1] != cellList[i]) {
			int numParts = 1;
			pPerCellData[cellList[i] * 2 + 0] = i;
			while (cellList[i] == cellList[i + 1]) {
				numParts++;
				i++;
			}
			pPerCellData[cellList[i] * 2 + 1] = numParts;
		}
	}
}


void makeParticlesPerGrid_PF(float * particlePositions, int * particleList, int * cellList, int * pPerCellData) {
	// Create key-value pairs from particles to grid indices
	makeKeyValuePairsKernel_PF <<< particleKernelGridDim_PF, particleKernelBlockDim_PF >>> (particlePositions, particleList, cellList);
	CudaSafeSync();
	CudaCheckError();
	// Sort key-value pairs by key
	thrust::device_ptr<int> dev_data_ptr(particleList);
	thrust::device_ptr<int> dev_keys_ptr(cellList);
	try
	{
		thrust::sort_by_key(dev_keys_ptr, dev_keys_ptr + numParticlesHost_PF, dev_data_ptr);
	}
	catch (std::bad_alloc &e)
	{
		std::cerr << "Ran out of memory while sorting" << std::endl;
		system("pause");
		exit(-1);
	}
	catch (thrust::system_error &e)
	{
		std::cerr << "Some other error happened during sort: " << e.what() << std::endl;
		system("pause");
		exit(-1);
	}

	CudaSafeSync();
	// For each grid cell store first index into gridKeys and number of particles
	CudaSafeCall(cudaMemset(pPerCellData, (int)0, sizeof(int)*gridSizeHost_PF * 2));
	CudaSafeSync();
	makePPCDataKernel_PF <<< particleKernelGridDim_PF, particleKernelBlockDim_PF >>> (cellList, pPerCellData);
	CudaSafeSync();
}

void particleVelocitiesToGrid_PF(float* particlePositions, float* particleVelocities, float* intermediateSolidVelocities, float* gridVelocities, float* gridVelocitiesOrig, float* gridWeights, float* gridDensities, int* particleList, int* cellList, int* ppcData, float* particleNormals, bool* gridSolid) {
	makeParticlesPerGrid_PF(particlePositions, particleList, cellList, ppcData);
	CudaSafeSync();
	computeGridVelocitiesKernel_PF <<< macGridKernelGridDim_PF, macGridKernelBlockDim_PF >>> (particlePositions, particleVelocities, intermediateSolidVelocities, gridVelocities, gridVelocitiesOrig, gridWeights, gridDensities, particleList, ppcData, gridSolid);
	CudaSafeSync();
}

__global__
void computeDivergenceKernel_PF(int* ppcData, float* gridVelocities, float* gridWeights, float* gridDensity, float* gridDivergence) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	int ii = i;
	int z = ii / layerSize_PF;
	ii = ii % layerSize_PF;
	int y = ii / gridSizeX_PF;
	int x = ii % gridSizeX_PF;
	if (x < gridSizeX_PF && y < gridSizeY_PF && z < gridSizeZ_PF) {
		int macInd = flattenIndexMAC_PF(x, y, z) * 3;

		// Index for the bottom, left and back velocities
		float xVelLeft = gridVelocities[macInd + 0];
		float yVelBottom = gridVelocities[macInd + 1];
		float zVelBack = gridVelocities[macInd + 2];
		// To get the top, right and front velocities, neighbouring MAC cells have to be checked
		float xVelRight = gridVelocities[flattenIndexMAC_PF(x + 1, y, z) * 3 + 0];
		float yVelTop = gridVelocities[flattenIndexMAC_PF(x, y + 1, z) * 3 + 1];
		float zVelFront = gridVelocities[flattenIndexMAC_PF(x, y, z + 1) * 3 + 2];

		// From those compute divergence via simple formula:
		gridDivergence[i] = (xVelRight - xVelLeft) + (yVelTop - yVelBottom) + (zVelFront - zVelBack);

		if (ppcData[i * 2 + 1] > 0) {
			// We decrease the divergence in case there are too many particles in this cell
			float curW = gridWeights[macInd / 3];
			if (densityCorrection_PF > 0 && curW > maxDensity_PF) {
				// The divergence decrease is scaled such that it is 0 at maximum density 
				// and equal to the difference after reaching twice the maximum density
				float x = (curW - maxDensity_PF) / maxDensity_PF;
				gridDivergence[i] -= x * maxDensity_PF * densityCorrection_PF;
			}
			gridDivergence[i] *= gridDensity[macInd / 3];
		}
		else {
			gridDivergence[i] = 0;
		}
	}
}

__global__
void jacobiIterationKernel_PF(int* ppcData, float* gridDivergence, float* gridPressureIn, float* gridPressureOut) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	int ii = i;
	int z = ii / layerSize_PF;
	ii = ii % layerSize_PF;
	int y = ii / gridSizeX_PF;
	int x = ii % gridSizeX_PF;
	if (x < gridSizeX_PF && y < gridSizeY_PF && z < gridSizeZ_PF) {
		// Discard empty cells
#ifndef PF_FULL_PROJECTION
		if (ppcData[i * 2 + 1] <= 0) {
			return;
		}
#endif
        // Surounding pressure values
        int blanks = 0;
		float left = 0;
		if (x > 0)
			left = gridPressureIn[flattenIndex_PF(x - 1, y, z)];
		else
			blanks++;
		float right = 0;
		if (x < gridSizeX_PF - 1)
			right = gridPressureIn[flattenIndex_PF(x + 1, y, z)];
		else
			blanks++;
		float bottom = 0;
		if (y > 0)
			bottom = gridPressureIn[flattenIndex_PF(x, y - 1, z)];
		else
			blanks++;
		float top = 0;
		if (y < gridSizeY_PF - 1)
			top = gridPressureIn[flattenIndex_PF(x, y + 1, z)];
		else
			blanks++;
		float back = 0;
		if (z > 0)
			back = gridPressureIn[flattenIndex_PF(x, y, z - 1)];
		else
			blanks++;
		float front = 0;
		if (z < gridSizeZ_PF - 1)
			front = gridPressureIn[flattenIndex_PF(x, y, z + 1)];
		else
			blanks++;
        // Center divergence
        float divergenceCenter = gridDivergence[i];
        // Jacobi update of current pressure entry (on a 1x1x1 grid this gives division by 0, ignored for efficiency)
        gridPressureOut[i] = (left + right + bottom + top + back + front - divergenceCenter) / (6.0 - blanks);
	}
}

__global__
void velocityCorrectionKernel_PF(int* ppcData, float* gridVelocities, float* gridPressure, float* gridDensity) {
	// THIS IS A MAC KERNEL!
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	int ii = i;
	int z = ii / layerSizeMAC_PF;
	ii = ii % layerSizeMAC_PF;
	int y = ii / (gridSizeX_PF + 1);
	int x = ii % (gridSizeX_PF + 1);
	if (x <= gridSizeX_PF && y <= gridSizeY_PF && z <= gridSizeZ_PF) {
		// As the velocity probes sit at the left, bottom and back face, the center of the pressure
		// cell with the same index is right/top/front of them
		float d = gridDensity[i];
		if (d > 0) {
			float fac = 1. / d;
			// Comput pressure gradient
			float center = gridPressure[flattenIndexBound_PF(x, y, z)];
			float left = gridPressure[flattenIndexBound_PF(x - 1, y, z)];
			float bottom = gridPressure[flattenIndexBound_PF(x, y - 1, z)];
			float back = gridPressure[flattenIndexBound_PF(x, y, z - 1)];
			if (x > 0 && x < gridSizeX_PF) gridVelocities[i * 3 + 0] -= (center - left) * fac;
			if (y > 0 && y < gridSizeY_PF) gridVelocities[i * 3 + 1] -= (center - bottom) * fac;
			if (z > 0 && z < gridSizeZ_PF) gridVelocities[i * 3 + 2] -= (center - back) * fac;
		}
	}

}

std::pair<float, float> correctGridVelocities_PF(float* gridVelocities, float* gridVelocitiesOrig, float* gridWeights, float* gridDensity, int* ppcData, float* gridDivergence, float* gridPressure1, float* gridPressure2) {
	// Compute divergence
	computeDivergenceKernel_PF <<< gridKernelGridDim_PF, gridKernelBlockDim_PF >>> (ppcData, gridVelocities, gridWeights, gridDensity, gridDivergence);

	CudaSafeSync();
	// Initialize pressure as 0
	CudaSafeCall(cudaMemset(gridPressure1, 0, sizeof(float)*gridSizeHost_PF));
	CudaSafeCall(cudaMemset(gridPressure2, 0, sizeof(float)*gridSizeHost_PF));
	// Do jacobi iteration to compute pseudo-pressure as Ap=d, where d is the divergence, p is the unknown pressure and A is the Laplace Matrix
	for (int i = 0; i < jacobiItsHost_PF; i++) {
		// Interchange between two pressure fields to prevent data races
		if (i % 2 == 0) {
			jacobiIterationKernel_PF <<< gridKernelGridDim_PF, gridKernelBlockDim_PF >>> (ppcData, gridDivergence, gridPressure1, gridPressure2);
		}
		else {
			jacobiIterationKernel_PF <<< gridKernelGridDim_PF, gridKernelBlockDim_PF >>> (ppcData, gridDivergence, gridPressure2, gridPressure1);
		}
		CudaSafeSync();
	}
	// Subtract pressure gradient from velocities
	velocityCorrectionKernel_PF <<< macGridKernelGridDim_PF, macGridKernelBlockDim_PF >>> (ppcData, gridVelocities, gridPressure1, gridDensity);

    CudaSafeSync();

    return std::pair<float, float>(0, 0);
}

__device__
twoVelos_PF evaluateVelocities_PF(float x, float y, float z, float* gridVelocities1, float* gridVelocities2) {
	x -= boundXMin_PF;
	x /= cellLength_PF;
	int xInd = (int)minI_PF(gridSizeX_PF - 1, maxI_PF(0, floor(x)));
	y -= boundYMin_PF;
	y /= cellLength_PF;
	int yInd = (int)minI_PF(gridSizeY_PF - 1, maxI_PF(0, floor(y)));
	z -= boundZMin_PF;
	z /= cellLength_PF;
	int zInd = (int)minI_PF(gridSizeZ_PF - 1, maxI_PF(0, floor(z)));
	float xWeight = 0, yWeight = 0, zWeight = 0;
	float xVel1 = 0, yVel1 = 0, zVel1 = 0;
	float xVel2 = 0, yVel2 = 0, zVel2 = 0;
	// Accumulate velocities and linear weights from nearby grid cells
	for (int xx = maxI_PF(xInd, 0); xx <= minI_PF(xInd + 1, gridSizeX_PF); xx++) {
		for (int yy = maxI_PF(yInd, 0); yy <= minI_PF(yInd + 1, gridSizeY_PF); yy++) {
			for (int zz = maxI_PF(zInd, 0); zz <= minI_PF(zInd + 1, gridSizeZ_PF); zz++) {
				int gridMACInd = flattenIndexMAC_PF(xx, yy, zz);
				// The weights are just tri-linear kernels, centered at the centers of each face, and at the cell center
				float curXW = linWeight((float)xx - x, (float)yy + 0.5 - y, (float)zz + 0.5 - z);
				curXW = (yy < gridSizeY_PF && zz < gridSizeZ_PF) ? curXW : 0;
				xWeight += curXW;
				float curYW = linWeight((float)xx + 0.5 - x, (float)yy - y, (float)zz + 0.5 - z);
				curYW = (xx < gridSizeX_PF && zz < gridSizeZ_PF) ? curYW : 0;
				yWeight += curYW;
				float curZW = linWeight((float)xx + 0.5 - x, (float)yy + 0.5 - y, (float)zz - z);
				curZW = (yy < gridSizeY_PF && xx < gridSizeX_PF) ? curZW : 0;
				zWeight += curZW;
				// Sum weighted velocities in each component
				xVel1 += gridVelocities1[gridMACInd * 3 + 0] * curXW;
				yVel1 += gridVelocities1[gridMACInd * 3 + 1] * curYW;
				zVel1 += gridVelocities1[gridMACInd * 3 + 2] * curZW;
				if (gridVelocities2) {
					// Sum weighted velocities in each component
					xVel2 += gridVelocities2[gridMACInd * 3 + 0] * curXW;
					yVel2 += gridVelocities2[gridMACInd * 3 + 1] * curYW;
					zVel2 += gridVelocities2[gridMACInd * 3 + 2] * curZW;
				}
			}
		}
	}

	twoVelos_PF vel;

	// Return velocities
	if (xWeight > 0) {
		vel.oldV.x = xVel2 / xWeight;
		vel.newV.x = xVel1 / xWeight;
	}
	else {
		vel.oldV.x = 0;
		vel.newV.x = 0;
	}

	if (yWeight > 0) {
		vel.oldV.y = yVel2 / yWeight;
		vel.newV.y = yVel1 / yWeight;
	}
	else {
		vel.oldV.y = 0;
		vel.newV.y = 0;
	}

	if (zWeight > 0) {
		vel.oldV.z = zVel2 / zWeight;
		vel.newV.z = zVel1 / zWeight;
	}
	else {
		vel.oldV.z = 0;
		vel.newV.z = 0;
	}

	/*
	if (isnan(vel.oldV.x) ||
		isnan(vel.oldV.y) ||
		isnan(vel.oldV.z) ||
		isnan(vel.newV.x) ||
		isnan(vel.newV.y) ||
		isnan(vel.newV.z) )
		printf("hasNAN!\n");
	*/


	return vel;
}

__forceinline__ __device__
float clampToGridX_PF(float x, bool largerBd = false) {
#ifdef PF_LARGE_BOUNDARIES
	float bEps = largerBd ? boundaryEpsLarge_PF : boundaryEps_PF;
#else
	float bEps = boundaryEps_PF;
#endif
	if (x < boundXMin_PF + bEps) x = boundXMin_PF + bEps;
	if (x > boundXMax_PF - bEps * 2) x = boundXMax_PF - bEps * 2;
	return x;
}

__forceinline__ __device__
float clampToGridY_PF(float y, bool largerBd = false) {
#ifdef PF_LARGE_BOUNDARIES
	float bEps = largerBd ? boundaryEpsLarge_PF : boundaryEps_PF;
#else
	float bEps = boundaryEps_PF;
#endif
	if (y < boundYMin_PF + bEps) y = boundYMin_PF + bEps;
	if (y > boundYMax_PF - bEps) y = boundYMax_PF - bEps;
	return y;
}

__forceinline__ __device__
float clampToGridZ_PF(float z, bool largerBd = false) {
#ifdef PF_LARGE_BOUNDARIES
	float bEps = largerBd ? boundaryEpsLarge_PF : boundaryEps_PF;
#else
	float bEps = boundaryEps_PF;
#endif
	if (z < boundZMin_PF + bEps) z = boundZMin_PF + bEps;
	if (z > boundZMax_PF - bEps * 2) z = boundZMax_PF - bEps * 2;
	return z;
}

__forceinline__ __device__
void preventPenetrationFluid_PF(float& x, float& y, float& z, float& vx, float& vy, float& vz, int* ppcData, int* particleList, float* particlePos, float* particleVelos, bool* gridSolid, float* particleNormals, int i, float* particleColors) {
	float closest = cellLength_PF * cellLength_PF * 1.;
	int sID = -1;
	// Find nearby closest solid particle:

	// Compute x, y and z indices
	float xi = x - boundXMin_PF;
	xi /= cellLength_PF;
	int xInd = (int)minI_PF(gridSizeX_PF - 1, maxI_PF(0, floor(xi)));
	float yi = y - boundYMin_PF;
	yi /= cellLength_PF;
	int yInd = (int)minI_PF(gridSizeY_PF - 1, maxI_PF(0, floor(yi)));
	float zi = z - boundZMin_PF;
	zi /= cellLength_PF;
	int zInd = (int)minI_PF(gridSizeZ_PF - 1, maxI_PF(0, floor(zi)));

	for (int xx = xi - 1; xx <= xi + 1; xx++) {
		for (int yy = yi - 1; yy <= yi + 1; yy++) {
			for (int zz = zi - 1; zz <= zi + 1; zz++) {
				if (xx < 0 || xx >= gridSizeX_PF || yy < 0 || yy >= gridSizeY_PF || zz < 0 || zz >= gridSizeZ_PF) continue;
				int gridInd = flattenIndex_PF(xx, yy, zz);
				if (gridSolid[gridInd]) {
					int numParts = ppcData[gridInd * 2 + 1];
					for (int partLocInd = 0; partLocInd < numParts; partLocInd++) {
						int partId = particleList[ppcData[gridInd * 2 + 0] + partLocInd];
						if (partId >= numFreeParticles_PF) {
							float dist = pow(x - particlePos[partId * 3 + 0], 2) + pow(y - particlePos[partId * 3 + 1], 2) + pow(z - particlePos[partId * 3 + 2], 2);
							if (dist < closest) {
								closest = dist;
								sID = partId;
							}
						}
					}
				}
			}
		}
	}
	if (sID >= numFreeParticles_PF) {
		// After it was found, we enforce v_solid * n_solid = v_part * n_solid (where * is the dot product)
		int ssID = sID - numFreeParticles_PF;
		float nx = particleNormals[ssID * 3 + 0], ny = particleNormals[ssID * 3 + 1], nz = particleNormals[ssID * 3 + 2];
		float a = fmaxf(0, nx * particleVelos[sID * 3 + 0] + ny * particleVelos[sID * 3 + 1] + nz * particleVelos[sID * 3 + 2]);
		float dot = nx * vx + ny * vy + nz * vz;
		if (dot < a) {
			float vnb = sqrtf(vx*vx + vy*vy + vz*vz);
			vx -= (dot - a) * nx;
			vy -= (dot - a) * ny;
			vz -= (dot - a) * nz;
			float fac = vnb/sqrtf(vx*vx + vy*vy + vz*vz);
			vx *= fac;
			vy *= fac;
			vz *= fac;
		}
	}
}

__forceinline__ __device__
void preventPenetrationSolid_PF(int pId, float& x, float& y, float& z, float& vx, float& vy, float& vz, int* ppcData, int* particleList, float* particlePos, float* particleVelos, float* particleNormals, float* particleColors, float* origPositions) {
	float nx = particleNormals[pId * 3 + 0], ny = particleNormals[pId * 3 + 1], nz = particleNormals[pId * 3 + 2];
	float nNorm = nx * nx + ny * ny + nz * nz;
	if (nNorm < 0.1) return;
	int i = pId + numFreeParticles_PF;

	// Find other solid particles
	// Compute x, y and z indices
	float xi = x - boundXMin_PF;
	xi /= cellLength_PF;
	int xInd = (int)minI_PF(gridSizeX_PF - 1, maxI_PF(0, floor(xi)));
	float yi = y - boundYMin_PF;
	yi /= cellLength_PF;
	int yInd = (int)minI_PF(gridSizeY_PF - 1, maxI_PF(0, floor(yi)));
	float zi = z - boundZMin_PF;
	zi /= cellLength_PF;
	int zInd = (int)minI_PF(gridSizeZ_PF - 1, maxI_PF(0, floor(zi)));

	for (int xx = xi - 1; xx <= xi + 1; xx++) {
		for (int yy = yi - 1; yy <= yi + 1; yy++) {
			for (int zz = zi - 1; zz <= zi + 1; zz++) {
				if (xx < 0 || xx >= gridSizeX_PF || yy < 0 || yy >= gridSizeY_PF || zz < 0 || zz >= gridSizeZ_PF) continue;
				int gridInd = flattenIndex_PF(xx, yy, zz);
				int numParts = ppcData[gridInd * 2 + 1];
				for (int partLocInd = 0; partLocInd < numParts; partLocInd++) {
					int opId = particleList[ppcData[gridInd * 2 + 0] + partLocInd];
					if (i != opId && opId >= numFreeParticles_PF) {
						int opIdN3 = (opId - numFreeParticles_PF) * 3;
						float onx = particleNormals[opIdN3 + 0], ony = particleNormals[opIdN3 + 1], onz = particleNormals[opIdN3 + 2];
						if (onx * onx + ony * ony + onz * onz < 0.1) continue;
						float ox = particlePos[opId * 3 + 0], oy = particlePos[opId * 3 + 1], oz = particlePos[opId * 3 + 2];
						float dx = (x - ox);
						float dy = (y - oy);
						float dz = (z - oz);
						// If they are close to each other
						if (dx * dx + dy * dy + dz * dz < cellLength_PF * cellLength_PF) {
							float odx = origPositions[opId * 3 + 0] - origPositions[i * 3 + 0], ody = origPositions[opId * 3 + 1] - origPositions[i * 3 + 1], odz = origPositions[opId * 3 + 2] - origPositions[i * 3 + 2];
							if (odx * odx + ody * ody + odz * odz < cellLength_PF * cellLength_PF * 4) continue;
							float ndot = nx * onx + ny * ony + nz * onz;
							// And if their normals point in opposite directions, and they are "facing" each other
							if (ndot < 0) { // && dx * nx + dy * ny + dz * nz > 0) {
								float ovx = particleVelos[opId * 3 + 0], ovy = particleVelos[opId * 3 + 1], ovz = particleVelos[opId * 3 + 2];
								// Make sure that velocities do not lead to intersections
								float a = fmaxf(0, onx * ovx + ony * ovy + onz * ovz);
								float dot = onx * vx + ony * vy + onz * vz;
								if (dot < a) {
									float vnb = sqrtf(vx*vx + vy*vy + vz*vz);
									vx -= (dot - a) * onx;
									vy -= (dot - a) * ony;
									vz -= (dot - a) * onz;
									float fac = vnb / sqrtf(vx*vx + vy*vy + vz*vz);
									vx *= fac;
									vy *= fac;
									vz *= fac;

								}
							}
						}
					}
				}
			}
		}
	}
}

__forceinline__ __device__
void preventPenetrationSolid2_PF(int pId, float& x, float& y, float& z, float& vx, float& vy, float& vz, int* ppcData, int* particleList, float* particlePos, float* particleVelos, float* particleNormals) {
	int gridInd = getGridCellID_PF(x, y, z);
	int numParts = ppcData[gridInd * 2 + 1];
	float nx = particleNormals[pId * 3 + 0], ny = particleNormals[pId * 3 + 1], nz = particleNormals[pId * 3 + 2];
	float nNorm = nx * nx + ny * ny + nz * nz;
	if (nNorm < 0.1) return;
	// Find other solid particles
	for (int partLocInd = 0; partLocInd < numParts; partLocInd++) {
		int opId = particleList[ppcData[gridInd * 2 + 0] + partLocInd];
		int opIdN3 = (opId - numFreeParticles_PF) * 3;
		float onx = particleNormals[opIdN3 + 0], ony = particleNormals[opIdN3 + 1], onz = particleNormals[opIdN3 + 2];
		// If their normals point in opposite directions, and they are "facing" each other
		float ox = particlePos[opId * 3 + 0], oy = particlePos[opId * 3 + 1], oz = particlePos[opId * 3 + 2];
		float dx = (x - ox);
		float dy = (y - oy);
		float dz = (z - oz);
		float ndot = nx * onx + ny * ony + nz * onz;
		if (ndot < 0 && dx * nx + dy * ny + dz * nz > 0) {
			float ovx = particleVelos[opId * 3 + 0], ovy = particleVelos[opId * 3 + 1], ovz = particleVelos[opId * 3 + 2];
			float ovNorm = ovx * ovx + ovy * ovy + ovz * ovz;
			if (vx * ovx + vy * ovy + vz * ovz < 0 || ovNorm < 1e-12) {
				// Make sure that velocities do not lead to intersections
				float a = fmaxf(0, nx * ovx + ny * ovy + nz * ovz);
				float dot = nx * vx + ny * vy + nz * vz;
				if (dot < a) {
					vx -= (dot - a) * nx;
					vy -= (dot - a) * ny;
					vz -= (dot - a) * nz;
				}
			}
		}
	}
}

__global__
void updateParticleNormalsKernel_PF(float* particleNormalsOrig, int maxNNZ, int* nnzIndices, float* nnzWeights, int cols, float* subspaceCoords, float* particleNormals) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF - numFreeParticles_PF) {
		int i3 = i * 3;
		float pnx = 0, pny = 0, pnz = 0;

		float pox = particleNormalsOrig[i3 + 0];
		float poy = particleNormalsOrig[i3 + 1];
		float poz = particleNormalsOrig[i3 + 2];

		for (int j = 0; j < maxNNZ; j++) {
			int off = maxNNZ * i;
			int curInd4 = nnzIndices[off + j] * 4;
			if (curInd4 >= 0 && curInd4 < cols) {
				float w = nnzWeights[off + j];
				pnx += w * subspaceCoords[curInd4 + 0] * pox;
				pnx += w * subspaceCoords[curInd4 + 1] * poy;
				pnx += w * subspaceCoords[curInd4 + 2] * poz;

				pny += w * subspaceCoords[cols + curInd4 + 0] * pox;
				pny += w * subspaceCoords[cols + curInd4 + 1] * poy;
				pny += w * subspaceCoords[cols + curInd4 + 2] * poz;

				pnz += w * subspaceCoords[2 * cols + curInd4 + 0] * pox;
				pnz += w * subspaceCoords[2 * cols + curInd4 + 1] * poy;
				pnz += w * subspaceCoords[2 * cols + curInd4 + 2] * poz;
			}
		}

		float l = 1. / sqrtf(pnx * pnx + pny * pny + pnz * pnz);

		l = (isfinite(l) ? l : 1.);

		particleNormals[i3 + 0] = pnx * l;
		particleNormals[i3 + 1] = pny * l;
		particleNormals[i3 + 2] = pnz * l;
	}
}

void updateParticleNormals_PF(float* particleNormalsOrig, int maxNNZ, int* nnzIndices, float* nnzWeights, int subCoordsRows, float* subspaceCoords, float* particleNormals) {
	updateParticleNormalsKernel_PF<<< (numParticlesHost_PF - numFreeParticlesHost_PF + 255) / 256, 256 >>>(particleNormalsOrig, maxNNZ, nnzIndices, nnzWeights, subCoordsRows, subspaceCoords, particleNormals);
	CudaSafeSync();
	CudaCheckError();
}

__global__
void transferToFluidParticlesKernel_PF(float* gridVelocities, float* gridVelocitiesOrig, float* particlePositions, float* particleVelocities, float* particleNormals, bool* gridSolid, int* ppcData, int* particleList, float* particleColors) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numFreeParticles_PF) {
		// Evaluate particle positions
		float x = particlePositions[3 * i + 0];
		float y = particlePositions[3 * i + 1];
		float z = particlePositions[3 * i + 2];

		// Set particle velocities as a mix of velocity change and new velocities
		twoVelos_PF v = evaluateVelocities_PF(x, y, z, gridVelocities, gridVelocitiesOrig);
		float pvx = (1 - flipness_PF) *	(particleVelocities[i * 3 + 0] + v.newV.x - v.oldV.x) + flipness_PF * v.newV.x;
		float pvy = (1 - flipness_PF) *	(particleVelocities[i * 3 + 1] + v.newV.y - v.oldV.y) + flipness_PF * v.newV.y;
		float pvz = (1 - flipness_PF) *	(particleVelocities[i * 3 + 2] + v.newV.z - v.oldV.z) + flipness_PF * v.newV.z;

		// Prevent fluid solid penetration
		preventPenetrationFluid_PF(x, y, z, pvx, pvy, pvz, ppcData, particleList, particlePositions, particleVelocities, gridSolid, particleNormals, i, particleColors);

		particleVelocities[i * 3 + 0] = pvx;
		particleVelocities[i * 3 + 1] = pvy;
		particleVelocities[i * 3 + 2] = pvz;
	}
}

__global__
void transferToFluidParticlesKernelNN_PF(float* gridVelocities, float* gridVelocitiesOrig, float* particlePositions, float* particleVelocities, float* particleNormals, bool* gridSolid, int* ppcData, int* particleList) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numFreeParticles_PF) {
		// Evaluate particle positions
		float x = particlePositions[3 * i + 0];
		float y = particlePositions[3 * i + 1];
		float z = particlePositions[3 * i + 2];

		// Set particle velocities as a mix of velocity change and new velocities
		twoVelos_PF v = evaluateVelocities_PF(x, y, z, gridVelocities, gridVelocitiesOrig);
		float pvx = (1 - flipness_PF) *	(particleVelocities[i * 3 + 0] + v.newV.x - v.oldV.x) + flipness_PF * v.newV.x;
		float pvy = (1 - flipness_PF) *	(particleVelocities[i * 3 + 1] + v.newV.y - v.oldV.y) + flipness_PF * v.newV.y;
		float pvz = (1 - flipness_PF) *	(particleVelocities[i * 3 + 2] + v.newV.z - v.oldV.z) + flipness_PF * v.newV.z;

		particleVelocities[i * 3 + 0] = pvx;
		particleVelocities[i * 3 + 1] = pvy;
		particleVelocities[i * 3 + 2] = pvz;
	}
}

__global__
void preventPenetrationsSolidKernel_PF(float* particlePositions, float* particleVelocities, float* particleNormals, int* ppcData, int* particleList, float* correctedVelocities, float* particleColors, float* origPositions) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF - numFreeParticles_PF) {
		int pId = i + numFreeParticles_PF;

		// Evaluate particle positions
		float x = particlePositions[3 * pId + 0];
		float y = particlePositions[3 * pId + 1];
		float z = particlePositions[3 * pId + 2];

		float pvx = particleVelocities[3 * pId + 0];
		float pvy = particleVelocities[3 * pId + 1];
		float pvz = particleVelocities[3 * pId + 2];

		// Prevent fluid solid penetration
		preventPenetrationSolid_PF(i, x, y, z, pvx, pvy, pvz, ppcData, particleList, particlePositions, particleVelocities, particleNormals, particleColors, origPositions);

		correctedVelocities[i * 3 + 0] = pvx;
		correctedVelocities[i * 3 + 1] = pvy;
		correctedVelocities[i * 3 + 2] = pvz;
	}
}

__global__
void transferToSolidParticlesKernel_PF(float* gridVelocities, float* gridVelocitiesOrig, float* particlePositions, float* particleVelocities, float* particleNormals, bool* gridSolid, int* ppcData, int* particleList) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF - numFreeParticles_PF) {
		i += numFreeParticles_PF;

		// Evaluate particle positions
		float x = particlePositions[3 * i + 0];
		float y = particlePositions[3 * i + 1];
		float z = particlePositions[3 * i + 2];

		// Set particle velocities as a mix of velocity change and new velocities
		twoVelos_PF v = evaluateVelocities_PF(x, y, z, gridVelocities, nullptr);

		particleVelocities[i * 3 + 0] = v.newV.x;
		particleVelocities[i * 3 + 1] = v.newV.y;
		particleVelocities[i * 3 + 2] = v.newV.z;
	}
}

__global__
void advectKernel_PF (float* gridVelocities, float* randomVelos, int frameNo, float* particlePositions, float* particleVelocities,  float* particleNormals, bool* gridSolid, int* ppcData, int* particleList, float* particleColors, float* origPositions) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF) {
		bool isFluid = (i < numFreeParticles_PF);

		// Evaluate particle positions
		float x = particlePositions[3 * i + 0];
		float y = particlePositions[3 * i + 1];
		float z = particlePositions[3 * i + 2];

		float pvx = particleVelocities[i * 3 + 0];
		float pvy = particleVelocities[i * 3 + 1];
		float pvz = particleVelocities[i * 3 + 2];

		// Evaluate halway position for RK2
		float hwx = clampToGridX_PF(x + 0.5 * timeStep_PF * pvx, isFluid);
		float hwy = clampToGridY_PF(y + 0.5 * timeStep_PF * pvy, isFluid);
		float hwz = clampToGridZ_PF(z + 0.5 * timeStep_PF * pvz, isFluid);

		vec3_PF hwv = evaluateVelocities_PF(hwx, hwy, hwz, gridVelocities, nullptr).newV;

		// If normals are provided...
		if (hasNormals_PF && isFluid) {
			preventPenetrationFluid_PF(hwx, hwy, hwz, hwv.x, hwv.y, hwv.z, ppcData, particleList, particlePositions, particleVelocities, gridSolid, particleNormals, i, particleColors);
		}
		if (hasNormals_PF && (!isFluid) && useNormalsForSolids_PF) {
			preventPenetrationSolid_PF(i - numFreeParticles_PF, hwx, hwy, hwz, hwv.x, hwv.y, hwv.z, ppcData, particleList, particlePositions, particleVelocities, particleNormals, particleColors, origPositions);
		}

		// Advance particles using halfway velocities
		x = clampToGridX_PF(x + timeStep_PF * hwv.x, isFluid);
		y = clampToGridY_PF(y + timeStep_PF * hwv.y, isFluid);
		z = clampToGridZ_PF(z + timeStep_PF * hwv.z, isFluid);
		particlePositions[3 * i + 0] = x;
		particlePositions[3 * i + 1] = y;
		particlePositions[3 * i + 2] = z;
	}
}


__global__
void advectKernelNN_PF(float* gridVelocities, float* randomVelos, int frameNo, float* particlePositions, float* particleVelocities, float* particleNormals, bool* gridSolid, int* ppcData, int* particleList) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF) {
		bool isFluid = (i < numFreeParticles_PF);

		// Evaluate particle positions
		float x = particlePositions[3 * i + 0];
		float y = particlePositions[3 * i + 1];
		float z = particlePositions[3 * i + 2];

		float pvx = particleVelocities[i * 3 + 0];
		float pvy = particleVelocities[i * 3 + 1];
		float pvz = particleVelocities[i * 3 + 2];

		// Evaluate halway position for RK2
		float hwx = clampToGridX_PF(x + 0.5 * timeStep_PF * pvx, isFluid);
		float hwy = clampToGridY_PF(y + 0.5 * timeStep_PF * pvy, isFluid);
		float hwz = clampToGridZ_PF(z + 0.5 * timeStep_PF * pvz, isFluid);

		// Evaluate halway velocity
		vec3_PF hwv = evaluateVelocities_PF(hwx, hwy, hwz, gridVelocities, nullptr).newV;

#ifdef PF_RAND_DIR_FAC
		if (isFluid) {
			float l = sqrtf(hwv.x * hwv.x + hwv.y * hwv.y + hwv.z * hwv.z);
			int randInd = (frameNo * 3 + i) % numFreeParticles_PF;
			hwv.x += l * randomVelos[randInd + 0] * PF_RAND_DIR_FAC;
			hwv.y += l * randomVelos[randInd + 1] * PF_RAND_DIR_FAC;
			hwv.z += l * randomVelos[randInd + 2] * PF_RAND_DIR_FAC;
		}
#endif

		// Advance particles using halfway velocities
		x = clampToGridX_PF(x + timeStep_PF * hwv.x, isFluid);
		y = clampToGridY_PF(y + timeStep_PF * hwv.y, isFluid);
		z = clampToGridZ_PF(z + timeStep_PF * hwv.z, isFluid);
		particlePositions[3 * i + 0] = x;
		particlePositions[3 * i + 1] = y;
		particlePositions[3 * i + 2] = z;
	}
}

int curFrame = 0;

void advectParticles_PF(float* gridVelocities, float* gridVelocitiesOrig, float* randomVelos, float* particlePositions, float* particleVelocities, bool dontAdvectFluids, float* particleNormals, bool* gridSolid, int* ppcData, int* particleList, float* intermediateSolidVelos, float* particleColors, float* origPositions) {
	// Transfer grid velocities to solid particles first
	if (numFreeParticlesHost_PF < numParticlesHost_PF) {
		transferToSolidParticlesKernel_PF <<< (numParticlesHost_PF - numFreeParticlesHost_PF + 255) / 256, 256 >> > (gridVelocities, gridVelocitiesOrig, particlePositions, particleVelocities, particleNormals, gridSolid, ppcData, particleList);
		CudaSafeSync();
		CudaCheckError();
		if (useNormalsForSolidsHost_PF && hasNormalsHost_PF) {
			preventPenetrationsSolidKernel_PF <<< (numParticlesHost_PF - numFreeParticlesHost_PF + 255) / 256, 256 >> > (particlePositions, particleVelocities, particleNormals, ppcData, particleList, intermediateSolidVelos, particleColors, origPositions);
			CudaSafeSync();
			CudaCheckError();
			CudaSafeCall(cudaMemcpy(particleVelocities + (numFreeParticlesHost_PF * 3), intermediateSolidVelos, (numParticlesHost_PF - numFreeParticlesHost_PF) * 3 * sizeof(float), cudaMemcpyDeviceToDevice));
		}
	}
	CudaCheckError();

	if (hasNormalsHost_PF) {
		// Then transfer fluid velocities to particles
		if ((!dontAdvectFluids) && numFreeParticlesHost_PF > 0) {
			transferToFluidParticlesKernel_PF <<< (numFreeParticlesHost_PF + 255) / 256, 256 >>> (gridVelocities, gridVelocitiesOrig, particlePositions, particleVelocities, particleNormals, gridSolid, ppcData, particleList, particleColors);
			CudaSafeSync();
		}
		// Advect them all
		advectKernel_PF <<< (numParticlesHost_PF + 255) / 256, 256 >>> (gridVelocities, randomVelos, curFrame, particlePositions, particleVelocities, particleNormals, gridSolid, ppcData, particleList, particleColors, origPositions);
		CudaSafeSync();
	}
	else {
		// Then transfer fluid velocities to particles
		if ((!dontAdvectFluids) && numFreeParticlesHost_PF > 0) {
			transferToFluidParticlesKernelNN_PF <<< (numFreeParticlesHost_PF + 255) / 256, 256 >>> (gridVelocities, gridVelocitiesOrig, particlePositions, particleVelocities, particleNormals, gridSolid, ppcData, particleList);
			CudaSafeSync();
		}
		// Advect them all
		advectKernelNN_PF << < (numParticlesHost_PF + 255) / 256, 256 >> > (gridVelocities, randomVelos, curFrame, particlePositions, particleVelocities, particleNormals, gridSolid, ppcData, particleList);
		CudaSafeSync();

	}
	curFrame++;
	CudaSafeSync();
	CudaCheckError();
}

__global__
void setMeshVelocitiesKernel_PF(float * particlePositions, int numMeshParticles, float * newMeshPositions, float * intermediateSolidVelocities)
{
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numMeshParticles) {
		int off = (numParticles_PF - numMeshParticles) * 3;
		intermediateSolidVelocities[i * 3 + 0] = (newMeshPositions[i * 3 + 0] - particlePositions[off + i * 3 + 0]) / timeStep_PF;
		intermediateSolidVelocities[i * 3 + 1] = (newMeshPositions[i * 3 + 1] - particlePositions[off + i * 3 + 1]) / timeStep_PF;
		intermediateSolidVelocities[i * 3 + 2] = (newMeshPositions[i * 3 + 2] - particlePositions[off + i * 3 + 2]) / timeStep_PF;
	}
}

void setMeshVelocities_PF(float * particlePositions, int numMeshParticles, float * newMeshPositions, float * intermediateSolidVelocities)
{
	setMeshVelocitiesKernel_PF <<< (numMeshParticles + 255) / 256, 256 >>> (particlePositions, numMeshParticles, newMeshPositions, intermediateSolidVelocities);
	CudaSafeSync();
	CudaCheckError();
}

__global__
void boundMeshPositionsKernel_PF(float* particlePositions, int numMeshParticles)
{
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numMeshParticles) {
		int off = (i + numFreeParticles_PF) * 3;
		particlePositions[off + 0] = clampToGridX_PF(particlePositions[off + 0]);
		particlePositions[off + 1] = clampToGridY_PF(particlePositions[off + 1]);
		particlePositions[off + 2] = clampToGridZ_PF(particlePositions[off + 2]);
	}
}


void boundMeshPositions_PF(float* particlePositions, int numMeshParticles)
{
	boundMeshPositionsKernel_PF <<< (numMeshParticles + 255) / 256, 256 >>> (particlePositions, numMeshParticles);
	CudaSafeSync();
	CudaCheckError();
}


__global__ 
void makeParticleColorsKernel_PF(float * particleVelocities, float * solidParticleNormals, float * particleColors) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < numParticles_PF) {
		bool isFluid = (i < numFreeParticles_PF);
		int i3 = i * 3;
		if (isFluid || (!hasNormals_PF)) {
			float r = (isFluid ? FLUID_COLOR_R : SOLID_COLOR_R);
			float g = (isFluid ? FLUID_COLOR_G : SOLID_COLOR_G);
			float b = (isFluid ? FLUID_COLOR_B : SOLID_COLOR_B);
			float vn = sqrtf(particleVelocities[i3 + 0] * particleVelocities[i3 + 0] + particleVelocities[i3 + 1] * particleVelocities[i3 + 1] + particleVelocities[i3 + 2] * particleVelocities[i3 + 2]);
			float d = fmaxf(-1., fminf(1., (vn * timeStep_PF) / cellLength_PF)) * colorDeviation_PF;
			particleColors[i3 + 0] = r + d;
			particleColors[i3 + 1] = g + d;
			particleColors[i3 + 2] = b + d;
			//printf("color g: %f, g+d: %f, value: %f\n", particleColors[i3 + 1], g + d, fminf(1., fmaxf(0., g + d)));
		}
		else {
			int in3 = (i - numFreeParticles_PF) * 3;
			particleColors[i3 + 0] = solidParticleNormals[in3 + 0];
			particleColors[i3 + 1] = solidParticleNormals[in3 + 1];
			particleColors[i3 + 2] = solidParticleNormals[in3 + 2];
		}
	}
}

void makeParticleColors(float * particleVelocities, float * solidParticleNormals, float * particleColors)
{
	makeParticleColorsKernel_PF <<< (numParticlesHost_PF + 255) / 256, 256 >>> (particleVelocities, solidParticleNormals, particleColors);
}
