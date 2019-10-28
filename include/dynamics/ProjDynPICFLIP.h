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
#include "ProjDynFluidSim.h"
#include "Eigen/Dense"
#include "ProjDynUtil.h"
#include "ProjDynTypeDef.h"
#include "cublas_v2.h"
#include <cuda_runtime.h>
#include "CudaSafeCall.h"
#include "StopWatch.h"
#ifndef __gl_h_
#ifndef __GL_H_
#include <GL\glew.h>
#endif
#endif
#include "cuda_gl_interop.h"


#define PF_PI           3.14159265358979323846

using namespace FluidSim;

typedef Eigen::Matrix<double, -1, -1> PFDenseMat;
typedef Eigen::LLT<PFDenseMat> PFDenseSolver;
typedef Eigen::Matrix<fluid_scalar, -1, 3> PFPositionsF;
typedef Eigen::Matrix<double, -1, 3> PFPositionsD;

#define PF_NUM_THREADS 8
#define PF_PARALLEL_FOR __pragma(omp parallel for num_threads(PF_NUM_THREADS)) 
#define PF_PPC 10
#define PF_SPARSE_SUBSPACE_MAPPING

class PDPICFLIP {
protected:
    bool m_firstSimFrame = true;

    /* Basic grid data */
    FluidGrid m_grid;
    fluid_index m_numParticles;
    fluid_index m_numFreeParticles;
    fluid_index m_numMeshParticles;

    /* Particle positions array on host, required for temporary conversion */
    fluid_scalar* m_particlePositionsH;
    fluid_scalar* m_allParticlePositionsH;

    /* Device particle data */
    fluid_scalar* m_particlePositionsD;
    fluid_scalar* m_origParticlePositionsD;
    fluid_scalar* m_particleVelocitiesD;
    fluid_scalar* m_intermediateSolidParticleVelocitiesD;
    fluid_scalar* m_meshParticlesD;
    fluid_scalar* m_randomVelosD;

    /* Device grid data */
    fluid_scalar* m_gridVelocitiesD;
    fluid_scalar* m_gridVelocitiesOrigD;
    fluid_scalar* m_gridDWeightsD;
    fluid_scalar* m_gridDivergenceD;
    fluid_scalar* m_gridPressure1D;
    fluid_scalar* m_gridPressure2D;
    fluid_scalar* m_gridDensityD;

    /* Auxilary device arrays for particles per cell information */
    fluid_int* m_particleListD;
    fluid_int* m_cellListD;
    fluid_int* m_pPerCellDataD;

    /* Subspace and mesh varaibles for device and host */
    fluid_int m_subSize;
    fluid_scalar* m_subspaceCoordsD;
    fluid_scalar* m_subspaceCoords2D;
    PFDenseMat m_subspaceMat;
    fluid_scalar* m_subspaceMatD;
    fluid_int* m_subspaceMatIndicesD;
    fluid_scalar* m_subspaceMatEntriesD;
    fluid_int m_subspacMaxNumEntries;
    bool* m_gridSolidD;

    /* Subspace projection variables */
    PFPositionsF m_meshSubCoordsFloatH;
    fluid_scalar m_subspaceProjectionRegularize = 0;
    fluid_scalar* m_projectionRhsD;
    PFDenseSolver m_subspaceProjectionSolver;
    PFPositionsD m_subspaceProjectionRHS;
    PFPositionsD m_subspaceProjectionResult;
    PFPositionsF m_subspaceProjectionRHSFloat;

    /* GL buffer ID and data pointer */
    GLuint m_glbufferId;
    float* m_glArrayPtr;
    bool m_showMeshParticles = false;
    bool m_particleColorBuffer = false;
    GLuint m_glColorBufferId;
    float* m_glColorArrayPtr;
    float* m_particleColorsD;

    /* Normal data */
    bool m_hasNormals = false;
    Eigen::Matrix<fluid_scalar, -1, -1>  m_solidNormals;
    float* m_solidNormalsD;
    float* m_solidNormalsOrigD;
    float* m_gridSolidNormalsD;
    int m_maxNNZ;
    float* m_nnzWeightsD;
    int* m_nnzIndsD;
    PFDenseMat m_bounds;

    /* Tracked divergence before and after incompressibility step*/
    std::vector<float> m_divPre;
    std::vector<float> m_divResidual;

public:
    PDPICFLIP(fluid_int gridWidth, fluid_int gridHeight, fluid_int gridDepth,
        fluid_scalar gridBaseX, fluid_scalar gridBaseY, fluid_scalar gridBaseZ,
        fluid_scalar cellLength, fluid_scalar timeStepInitial,
        fluid_index numFreeParticles, fluid_index numMeshParticless,
        PFDenseMat& subspaceMatrix, fluid_scalar subspaceProjectionRegularizer,
        const PFDenseMat& normals = PFDenseMat(0, 3), bool useNormalsForSolids = true);

    void setFreeParticlePositions(PFPositionsD& partPos);
    void setMeshPositions(PFPositionsD& meshSubCoords);
    void setMeshVelocities(PFPositionsD& meshSubCoords);
    void setTimeStep(fluid_scalar h);

    void setupSubspaceProjection(double regularizer);
    void getProjectionRHS(PFPositionsD& destRHS);
    void projectParticlesToSubspace(PFPositionsD& prevSubCoords);
    void prepareSubspaceMapping();

    void setGLBuffer(GLuint glBufferID, bool showMeshParticles, GLuint glColorBufferID = 0);
    void updateGLBuffer();
    void resetVelocities();

    int getNumParticles();
    int getNumFreeParticles();
    void getFreeParticlePositions(double* data);
    int getGridSize();
    PFDenseMat& getGridBounds();

    void step(fluid_scalar stepSize, fluid_scalar gravity, fluid_scalar flipness, fluid_scalar maxDensity = -1, fluid_scalar densityCorrection = 0, fluid_scalar fluidDensity = .5,
        int maxNumSubstepsFluid = 20, int maxNumSubstepsSolid = 1, float frictionFac = 0, float solidInfluence = 0.5, bool dontAdvectFluids = false, int jacobiIts = 50,
        float colorDeviation = 0.1, float maxDCorrection = 0.5, bool skipIncompressiblity = false, float velStab = 0.);
};


PDPICFLIP* createPicFlip(int gridXZ, int gridY,
    fluid_scalar cellLength, fluid_scalar gap, fluid_scalar mass,
    fluid_scalar gravity, GLuint& glBufferID,
    GLuint& glColorBufferID,
    const Eigen::Matrix<double, -1, -1> & meshParticles,
    const std::vector<Eigen::Matrix<double, 3, 1>> & subspaceSamples,
    fluid_scalar timeStepInitial,
    fluid_scalar subspaceRadius,
    fluid_scalar expansionFactor = 0,
    bool showMeshParticles = false,
    bool addStream = false,
    const PFDenseMat & particleNormals = PFDenseMat(0, 0),
    bool preventSolidPenetration = false,
    float forceFloorHeight = -9999,
    float streamSize = 1.,
    bool forceUnitSquare = false,
    int seedPPC = PF_PPC);
