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

#include "dynamics/ProjDynPICFLIP.h"
#include "dynamics/SPH.h"
#include "cuda/doubleToFloatDeviceCpy.cuh"
#include "cuda/PF_Cuda_Kernels.cuh"
#include "dynamics/ProjDynUtil.h"

bool cublasLibInitialized_PF = false;
cublasHandle_t cublasLibHandle_PF = 0;

using namespace SPH;

PDPICFLIP::PDPICFLIP(
    fluid_int gridWidth, fluid_int gridHeight, fluid_int gridDepth,
    fluid_scalar gridBaseX, fluid_scalar gridBaseY, fluid_scalar gridBaseZ,
    fluid_scalar cellLength, fluid_scalar timeStepInitial,
    fluid_index numFreeParticles, fluid_index numMeshParticles,
    PFDenseMat& subspaceMatrix, fluid_scalar subspaceProjectionRegularizer,
    const PFDenseMat& normals, bool useNormalsForSolids
)
    :
    m_grid(
        fluid_ivec3(gridWidth, gridHeight, gridDepth),
        fluid_vec3(gridBaseX, gridBaseY, gridBaseZ),
        cellLength
    ),
    m_subspaceMat(subspaceMatrix),
    m_particlePositionsH(nullptr)
{
    int v = -1;
    cudaRuntimeGetVersion(&v);
    printf("Cuda version: %d \n", v);

    // Initialize Cublas
    if (!cublasLibInitialized_PF) {
        cublasCreate(&cublasLibHandle_PF);
        cublasLibInitialized_PF = true;
    }

    m_bounds.setZero(2, 3);
    m_bounds(0, 0) = gridBaseX;
    m_bounds(0, 1) = gridBaseY;
    m_bounds(0, 2) = gridBaseZ;
    m_bounds(1, 0) = gridBaseX + gridWidth * cellLength;
    m_bounds(1, 1) = gridBaseY + gridHeight * cellLength;
    m_bounds(1, 2) = gridBaseZ + gridDepth * cellLength;

    // Initilialize particle counts
    m_numFreeParticles = numFreeParticles;
    m_numMeshParticles = numMeshParticles;
    m_numParticles = m_numFreeParticles + m_numMeshParticles;

    printf("Initializing PICFLIP with %i fluid and %i solid particles...\n", m_numFreeParticles, m_numMeshParticles);
    printf("	Grid dimensions: %i, %i, %i  (amounts to %i cells)\n", gridWidth, gridHeight, gridDepth, m_grid.getSize());
    printf("	Subspace dimension: %i\n", subspaceMatrix.cols() * 3);


    m_subspaceProjectionRegularize = -1;

    // Initialize CUDA data structures
    // Reserve space for particle positions and velocities
    CudaSafeCall(cudaMalloc((void**)& m_particlePositionsD, sizeof(fluid_scalar) * m_numParticles * 3));
    CudaSafeCall(cudaMalloc((void**)& m_origParticlePositionsD, sizeof(fluid_scalar) * m_numParticles * 3));
    CudaSafeCall(cudaMalloc((void**)& m_particleVelocitiesD, sizeof(fluid_scalar) * m_numParticles * 3));
    CudaSafeCall(cudaMalloc((void**)& m_intermediateSolidParticleVelocitiesD, sizeof(fluid_scalar) * m_numMeshParticles * 3));
    CudaSafeCall(cudaMalloc((void**)& m_meshParticlesD, sizeof(fluid_scalar) * 3 * m_numMeshParticles));
    m_particlePositionsH = new fluid_scalar[m_numParticles * 3];
    CudaCheckError();

    // Reserve space for grid velocities
    CudaSafeCall(cudaMalloc((void**)& m_gridVelocitiesD, sizeof(fluid_scalar) * m_grid.getMACSize() * 3));
    CudaSafeCall(cudaMalloc((void**)& m_gridVelocitiesOrigD, sizeof(fluid_scalar) * m_grid.getMACSize() * 3));
    CudaSafeCall(cudaMalloc((void**)& m_gridDWeightsD, sizeof(fluid_scalar) * m_grid.getMACSize()));
    CudaSafeCall(cudaMalloc((void**)& m_gridDivergenceD, sizeof(fluid_scalar) * m_grid.getSize()));
    CudaSafeCall(cudaMalloc((void**)& m_gridPressure1D, sizeof(fluid_scalar) * m_grid.getSize()));
    CudaSafeCall(cudaMalloc((void**)& m_gridPressure2D, sizeof(fluid_scalar) * m_grid.getSize()));
    CudaSafeCall(cudaMalloc((void**)& m_gridDensityD, sizeof(fluid_scalar) * m_grid.getMACSize()));
    CudaSafeCall(cudaMemset(m_gridDensityD, 0, sizeof(fluid_scalar) * m_grid.getMACSize()));
    CudaCheckError();


    // Reserve space for auxilary particle-per-cell info
    CudaSafeCall(cudaMalloc((void**)& m_particleListD, sizeof(fluid_int) * m_numParticles));
    CudaSafeCall(cudaMalloc((void**)& m_cellListD, sizeof(fluid_int) * m_numParticles));
    CudaSafeCall(cudaMalloc((void**)& m_pPerCellDataD, sizeof(fluid_int) * m_grid.getSize() * 2));
    CudaSafeSync();
    CudaCheckError();

    resetVelocities();
    CudaSafeSync();
    CudaCheckError();

    if (m_numFreeParticles > 0) {
        // Create random velocities for fluid particles
        CudaSafeCall(cudaMalloc((void**)& m_randomVelosD, sizeof(fluid_scalar) * 3 * m_numFreeParticles));
        float* randVelos = new float[m_numFreeParticles * 3];
        for (int i = 0; i < m_numFreeParticles; i++) {
            float u = 1. - 2. * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
            float phi = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 2. * PF_PI;
            randVelos[i * 3 + 0] = std::sqrtf(1 - u * u) * std::cos(phi);
            randVelos[i * 3 + 1] = std::sqrtf(1 - u * u) * std::sin(phi);
            randVelos[i * 3 + 2] = u;
        }
        CudaSafeCall(cudaMemcpy(m_randomVelosD, randVelos, sizeof(fluid_scalar) * 3 * m_numFreeParticles, cudaMemcpyKind::cudaMemcpyHostToDevice));
        CudaCheckError();
    }

    // If normals are provided, upload them
    if (normals.rows() > 0) {
        m_hasNormals = true;
        for (int i = 0; i < normals.rows(); i++) {
            if (std::abs(1. - normals.row(i).norm()) > 1e-08) {
                printf("WARNING: There are degenerated input normals!\n");
                break;
            }
        }
        m_solidNormals = normals.transpose().cast<fluid_scalar>();
        CudaSafeCall(cudaMalloc((void**)& m_solidNormalsD, sizeof(fluid_scalar) * m_numMeshParticles * 3));
        CudaSafeCall(cudaMemcpy(m_solidNormalsD, m_solidNormals.data(), sizeof(fluid_scalar) * m_numMeshParticles * 3, cudaMemcpyHostToDevice));

        CudaSafeCall(cudaMalloc((void**)& m_solidNormalsOrigD, sizeof(fluid_scalar) * m_numMeshParticles * 3));
        CudaSafeCall(cudaMemcpy(m_solidNormalsOrigD, m_solidNormals.data(), sizeof(fluid_scalar) * m_numMeshParticles * 3, cudaMemcpyHostToDevice));

        CudaSafeCall(cudaMalloc((void**)& m_gridSolidNormalsD, sizeof(fluid_scalar) * m_grid.getSize() * 3));

        // Prepare nnz skinning weights and handle indices
        std::vector<std::vector<std::pair< fluid_int, float >>> indexEntryPairs;
        indexEntryPairs.resize(m_numMeshParticles);
        int maxNumEntries = 0;
        for (int i = 0; i < m_numMeshParticles; i++) {
            std::vector<std::pair< fluid_int, float >> curPairs;
            for (int j = 0; j < m_subspaceMat.cols() / 4; j++) {
                if (std::abs(m_subspaceMat(i, j * 4 + 3)) > 1e-12) {
                    curPairs.push_back(std::pair< fluid_int, float >(j, m_subspaceMat(i, j * 4 + 3)));
                }
            }
            if (curPairs.size() > maxNumEntries) maxNumEntries = curPairs.size();
            indexEntryPairs[i] = curPairs;
        }

        fluid_int* indices = new fluid_int[m_numMeshParticles * maxNumEntries];
        fluid_scalar* entries = new fluid_scalar[m_numMeshParticles * maxNumEntries];
        for (int i = 0; i < m_numMeshParticles; i++) {
            std::vector<std::pair< fluid_int, float >>& curPairs = indexEntryPairs[i];
            for (int j = 0; j < maxNumEntries; j++) {
                if (j < curPairs.size()) {
                    indices[i * maxNumEntries + j] = curPairs[j].first;
                    entries[i * maxNumEntries + j] = curPairs[j].second;
                }
                else {
                    indices[i * maxNumEntries + j] = -1;
                    entries[i * maxNumEntries + j] = 0;
                }
            }
        }

        CudaSafeCall(cudaMalloc((void**)& m_nnzIndsD, sizeof(fluid_int) * m_numMeshParticles * maxNumEntries));
        CudaSafeCall(cudaMemcpy(m_nnzIndsD, indices, sizeof(fluid_int) * m_numMeshParticles * maxNumEntries, cudaMemcpyHostToDevice));
        CudaSafeCall(cudaMalloc((void**)& m_nnzWeightsD, sizeof(fluid_scalar) * m_numMeshParticles * maxNumEntries));
        CudaSafeCall(cudaMemcpy(m_nnzWeightsD, entries, sizeof(fluid_scalar) * m_numMeshParticles * maxNumEntries, cudaMemcpyHostToDevice));
        CudaCheckError();

        m_maxNNZ = maxNumEntries;

        CudaSafeCall(cudaMalloc((void**)& m_gridSolidD, sizeof(bool) * m_grid.getSize()));
        CudaCheckError();
    }
    else {
        m_hasNormals = false;
        useNormalsForSolids = false;
        m_solidNormalsD = nullptr;
        m_solidNormalsOrigD = nullptr;
        m_gridSolidNormalsD = nullptr;
    }

    // Load grid dimensions to device memory
    initGrid_PF(m_numParticles, m_numFreeParticles, cellLength, timeStepInitial, gridWidth,
        gridHeight, gridDepth, gridBaseX, gridBaseY, gridBaseZ, m_hasNormals, useNormalsForSolids);
    CudaSafeSync();
    CudaCheckError();

    if (m_numMeshParticles > 0 && m_subspaceMat.cols() > 0) {
        m_subSize = m_subspaceMat.cols();
        // Reserve space for the mesh particle's subspace coordinates and the projection's rhs
        CudaSafeCall(cudaMalloc((void**)& m_subspaceCoordsD, sizeof(fluid_scalar) * m_subSize * 3));
        CudaSafeCall(cudaMalloc((void**)& m_subspaceCoords2D, sizeof(fluid_scalar) * m_subSize * 3));
        CudaCheckError();
        // Allocate space for and upload the subspace matrix
        Eigen::Matrix<fluid_scalar, -1, -1> subMatFloat = subspaceMatrix.cast<float>();
        const void* matDataPointer = (const void*)subMatFloat.data();
        CudaSafeCall(cudaMalloc((void**)& m_subspaceMatD, sizeof(fluid_scalar) * m_numMeshParticles * m_subSize));
        CudaCheckError();
        CublasSafeCall(cublasSetMatrix(m_numMeshParticles, m_subSize, sizeof(fluid_scalar), matDataPointer, m_numMeshParticles, (void*)m_subspaceMatD, m_numMeshParticles));
        CudaCheckError();
        setupSubspaceProjection(subspaceProjectionRegularizer);
#ifdef PF_SPARSE_SUBSPACE_MAPPING
        prepareSubspaceMapping();
#endif
    }
    else {
        m_numMeshParticles = 0;
        m_subSize = 0;
    }
    CudaSafeSync();
    CudaCheckError();

}

void PDPICFLIP::setFreeParticlePositions(PFPositionsD& partPos)
{
    if (m_numFreeParticles == 0) return;

    // Convert positions to float and correct ordering
    for (int i = 0; i < m_numFreeParticles; i++) {
        for (int d = 0; d < 3; d++) {
            m_particlePositionsH[i * 3 + d] = (fluid_scalar)partPos(i, d);
        }
    }
    // Copy from host to device
    CudaSafeCall(cudaMemcpy(m_particlePositionsD, m_particlePositionsH, m_numFreeParticles * 3 * sizeof(fluid_scalar), cudaMemcpyHostToDevice));
}

void PDPICFLIP::prepareSubspaceMapping()
{
    std::vector<std::vector<std::pair< fluid_int, float >>> indexEntryPairs;
    indexEntryPairs.resize(m_subspaceMat.rows());
    int maxNumEntries = 0;
    for (int i = 0; i < m_subspaceMat.rows(); i++) {
        std::vector<std::pair< fluid_int, float >> curPairs;
        for (int j = 0; j < m_subspaceMat.cols(); j++) {
            if (std::abs(m_subspaceMat(i, j)) > 1e-12) {
                curPairs.push_back(std::pair< fluid_int, float >(j, m_subspaceMat(i, j)));
            }
        }
        if (curPairs.size() > maxNumEntries) maxNumEntries = curPairs.size();
        indexEntryPairs[i] = curPairs;
    }

    fluid_int* indices = new fluid_int[m_subspaceMat.rows() * maxNumEntries];
    fluid_scalar* entries = new fluid_scalar[m_subspaceMat.rows() * maxNumEntries];
    for (int i = 0; i < m_subspaceMat.rows(); i++) {
        std::vector<std::pair< fluid_int, float >>& curPairs = indexEntryPairs[i];
        for (int j = 0; j < maxNumEntries; j++) {
            if (j < curPairs.size()) {
                indices[i * maxNumEntries + j] = curPairs[j].first;
                entries[i * maxNumEntries + j] = curPairs[j].second;
            }
            else {
                indices[i * maxNumEntries + j] = 0;
                entries[i * maxNumEntries + j] = 0;
            }
        }
    }

    CudaSafeCall(cudaMalloc((void**)& m_subspaceMatIndicesD, sizeof(fluid_int) * m_subspaceMat.rows() * maxNumEntries));
    CudaSafeCall(cudaMemcpy(m_subspaceMatIndicesD, indices, sizeof(fluid_int) * m_subspaceMat.rows() * maxNumEntries, cudaMemcpyHostToDevice));
    CudaSafeCall(cudaMalloc((void**)& m_subspaceMatEntriesD, sizeof(fluid_scalar) * m_subspaceMat.rows() * maxNumEntries));
    CudaSafeCall(cudaMemcpy(m_subspaceMatEntriesD, entries, sizeof(fluid_scalar) * m_subspaceMat.rows() * maxNumEntries, cudaMemcpyHostToDevice));
    CudaCheckError();

    m_subspacMaxNumEntries = maxNumEntries;
}

void PDPICFLIP::setMeshPositions(PFPositionsD& meshSubCoords)
{
    if (m_subSize > 0 && m_numMeshParticles > 0) {
        // Copy subspace coordinates to the GPU and map them to the mesh particle positions via
        // multiplication with the subspace matrix
        m_meshSubCoordsFloatH = meshSubCoords.cast<float>();
        CudaCheckError();
        CudaSafeCall(cudaMemcpy(m_subspaceCoordsD, m_meshSubCoordsFloatH.data(), m_subSize * 3 * sizeof(float), cudaMemcpyHostToDevice));
        CudaCheckError();
#ifdef PF_SPARSE_SUBSPACE_MAPPING
        sparseCoordinateEvaluation_PF(m_subspaceMat.rows(), m_subspaceMat.cols(), m_subspacMaxNumEntries, m_subspaceMatIndicesD, m_subspaceMatEntriesD, m_subspaceCoordsD, m_particlePositionsD + (3 * m_numFreeParticles));
        CudaSafeSync();
#else
        fluid_scalar alpha = 1;
        fluid_scalar beta = 0;
        for (int d = 0; d < 3; d++) {
            CublasSafeCall(cublasSgemv(cublasLibHandle_PF, CUBLAS_OP_N, m_numMeshParticles, m_subSize, &alpha, m_subspaceMatD, m_numMeshParticles, m_subspaceCoordsD + (d * m_subSize), 1, &beta, m_particlePositionsD + (3 * m_numFreeParticles) + d, 3));
            CudaCheckError();
        }
#endif
        boundMeshPositions_PF(m_particlePositionsD, m_numMeshParticles);
        CudaSafeSync();
        // If normals are provided, they need to be updated
        if (m_hasNormals) {
            m_meshSubCoordsFloatH = meshSubCoords.cast<float>();
            CudaSafeCall(cudaMemcpy(m_subspaceCoords2D, m_meshSubCoordsFloatH.data(), m_subSize * 3 * sizeof(float), cudaMemcpyHostToDevice));
            CudaCheckError();
            updateParticleNormals_PF(m_solidNormalsOrigD, m_maxNNZ, m_nnzIndsD, m_nnzWeightsD, m_subSize, m_subspaceCoords2D, m_solidNormalsD);
            CudaSafeSync();
        }
    }
}

// The velocities are set via specifying the new subspace coordinates of the mesh.
// The velocities of the particles are then induced from the difference of the new particle and
// old particle positions divided by the time step.
void PDPICFLIP::setMeshVelocities(PFPositionsD& meshSubCoords)
{
    if (m_subSize > 0 && m_numMeshParticles > 0) {
        // Copy subspace coordinates to the GPU and map them to the mesh particle positions via
        // multiplication with the subspace matrix
        m_meshSubCoordsFloatH = meshSubCoords.cast<float>();
        CublasSafeCall(cublasSetVector(m_subSize * 3, sizeof(float), m_meshSubCoordsFloatH.data(), 1, m_subspaceCoordsD, 1));
#ifdef PF_SPARSE_SUBSPACE_MAPPING
        sparseCoordinateEvaluation_PF(m_subspaceMat.rows(), m_subspaceMat.cols(), m_subspacMaxNumEntries, m_subspaceMatIndicesD, m_subspaceMatEntriesD, m_subspaceCoordsD, m_meshParticlesD);
#else
        fluid_scalar alpha = 1.;
        fluid_scalar beta = 0.;
        for (int d = 0; d < 3; d++) {
            CublasSafeCall(cublasSgemv(cublasLibHandle_PF, CUBLAS_OP_N, m_numMeshParticles, m_subSize, &alpha, m_subspaceMatD, m_numMeshParticles, m_subspaceCoordsD + (d * m_subSize), 1, &beta, m_meshParticlesD + d, 3));
            CudaCheckError();
        }
#endif
        // Take difference with current positions and use as velocities
        setMeshVelocities_PF(m_particlePositionsD, m_numMeshParticles, m_meshParticlesD, m_intermediateSolidParticleVelocitiesD);
        CudaCheckError();
    }
}

void PDPICFLIP::setTimeStep(fluid_scalar h)
{
    setTimeStep_PF(h);
}


void PDPICFLIP::setupSubspaceProjection(double regularizer) {
    if (m_subspaceProjectionRegularize != regularizer) {
        // Set up subspace projection:
        m_subspaceProjectionRegularize = (float)regularizer;
        m_subspaceProjectionRHS.setZero(m_subspaceMat.cols(), 3);
        m_subspaceProjectionResult.setZero(m_subspaceMat.cols(), 3);
        PFDenseMat subProjLHS = m_subspaceMat.transpose() * m_subspaceMat;
        if (m_subspaceProjectionRegularize > 0) {
            for (int j = 0; j < m_subspaceMat.cols(); j++) {
                subProjLHS(j, j) += m_subspaceProjectionRegularize;
            }
        }
        m_subspaceProjectionSolver.compute(subProjLHS);
        CudaSafeCall(cudaMalloc((void**)& m_projectionRhsD, sizeof(fluid_scalar) * m_subSize * 3));
        m_subspaceProjectionRHSFloat.setZero(m_subSize, 3);
        CudaCheckError();
    }
}

void PDPICFLIP::getProjectionRHS(PFPositionsD& destRHS)
{
    // Multiply the mesh particles positions with subspaceMat^T and copy that vector to the host
    fluid_scalar alpha = 1;
    fluid_scalar beta = 0;
    // On device perform rhs = subspaceMat^T * positions_x/y/z
    // Note: positions on device are sorted x1,y1,z1,x2,y2,z2,...
    for (int d = 0; d < 3; d++) {
        CublasSafeCall(cublasSgemv(cublasLibHandle_PF, CUBLAS_OP_T, m_numMeshParticles, m_subSize, &alpha, m_subspaceMatD, m_numMeshParticles, m_particlePositionsD + (3 * m_numFreeParticles) + d, 3, &beta, m_projectionRhsD + (d * m_subSize), 1));
    }
    // Copy rhs from device to host (still floats)
    CudaSafeCall(cudaMemcpy(m_subspaceProjectionRHSFloat.data(), m_projectionRhsD, m_subSize * 3 * sizeof(fluid_scalar), cudaMemcpyDeviceToHost));
    // Turn floats into doubles
    destRHS = m_subspaceProjectionRHSFloat.cast<double>();
}

void PDPICFLIP::projectParticlesToSubspace(PFPositionsD& prevSubCoords)
{
    // Compute RHS on GPU
    if (m_subspaceProjectionRHS.rows() != m_subSize) {
        m_subspaceProjectionRHS.resize(m_subSize, 3);
    }
    getProjectionRHS(m_subspaceProjectionRHS);
    // Regularize RHS
    if (m_subspaceProjectionRegularize > 0 && prevSubCoords.rows() == m_subSize) {
        m_subspaceProjectionRHS += prevSubCoords * m_subspaceProjectionRegularize;
    }
    // Solve system for all x, y and z in parallel
    PF_PARALLEL_FOR
        for (int d = 0; d < 3; d++) {
            prevSubCoords.col(d) = m_subspaceProjectionSolver.solve(m_subspaceProjectionRHS.col(d));
        }
}

void PDPICFLIP::setGLBuffer(GLuint glBufferID, bool showMeshParticles, GLuint glColorBufferID)
{
    m_showMeshParticles = showMeshParticles;

    m_glbufferId = glBufferID;
    glBindBuffer(GL_ARRAY_BUFFER, m_glbufferId);
    cudaGraphicsResource_t res;
    cudaGraphicsGLRegisterBuffer(&res, m_glbufferId, cudaGraphicsRegisterFlagsNone);
    cudaGraphicsMapResources(1, &res, 0);
    size_t size;
    cudaGraphicsResourceGetMappedPointer((void**)& m_glArrayPtr, &size, res);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    m_glColorBufferId = glColorBufferID;
    if (m_glColorBufferId > 0) {
        CudaSafeCall(cudaMalloc((void**)& m_particleColorsD, sizeof(fluid_scalar) * m_numParticles * 3));
        m_particleColorBuffer = true;
        glBindBuffer(GL_ARRAY_BUFFER, m_glColorBufferId);
        cudaGraphicsResource_t cres;
        cudaGraphicsGLRegisterBuffer(&cres, m_glColorBufferId, cudaGraphicsRegisterFlagsNone);
        cudaGraphicsMapResources(1, &cres, 0);
        size_t csize;
        cudaGraphicsResourceGetMappedPointer((void**)& m_glColorArrayPtr, &csize, cres);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    else {
        m_particleColorBuffer = false;
    }
}

void PDPICFLIP::updateGLBuffer()
{
    if (m_glbufferId > 0 && m_glArrayPtr) {
        //Copy particle positions into the OpenGL buffer
        if (m_showMeshParticles) {
            CudaSafeCall(cudaMemcpy(m_glArrayPtr, m_particlePositionsD, sizeof(fluid_scalar) * m_numParticles * 3, cudaMemcpyDeviceToDevice));
        }
        else {
            CudaSafeCall(cudaMemcpy(m_glArrayPtr, m_particlePositionsD, sizeof(fluid_scalar) * m_numFreeParticles * 3, cudaMemcpyDeviceToDevice));
        }
    }
    if (m_glColorBufferId > 0 && m_glColorArrayPtr) {
#ifndef PF_DEBUG_COLORS
        makeParticleColors(m_particleVelocitiesD, m_solidNormalsD, m_glColorArrayPtr);
#endif
    }
}

void PDPICFLIP::resetVelocities()
{
    // Set velocities to 0
    CudaSafeCall(cudaMemset(m_particleVelocitiesD, 0, sizeof(fluid_scalar) * m_numParticles * 3));
    CudaSafeCall(cudaMemset(m_intermediateSolidParticleVelocitiesD, 0, sizeof(fluid_scalar) * m_numMeshParticles * 3));
    CudaSafeCall(cudaMemset(m_gridVelocitiesD, 0, sizeof(fluid_scalar) * m_grid.getMACSize() * 3));
    CudaSafeCall(cudaMemset(m_gridVelocitiesOrigD, 0, sizeof(fluid_scalar) * m_grid.getMACSize() * 3));
}

int PDPICFLIP::getNumParticles()
{
    return m_numParticles;
}

int PDPICFLIP::getNumFreeParticles()
{
    return m_numFreeParticles;
}

void PDPICFLIP::getFreeParticlePositions(double* data)
{
    if (m_numFreeParticles == 0) return;

    // Copy from device to host
    CudaSafeCall(cudaMemcpy(m_particlePositionsH, m_particlePositionsD, m_numFreeParticles * 3 * sizeof(fluid_scalar), cudaMemcpyDeviceToHost));

    // Convert positions to double and correct ordering
    for (int i = 0; i < m_numFreeParticles; i++) {
        for (int d = 0; d < 3; d++) {
            data[d * m_numFreeParticles + i] = m_particlePositionsH[i * 3 + d];
        }
    }
    CudaCheckError();
}

int PDPICFLIP::getGridSize()
{
    return m_grid.getSize();
}

PFDenseMat& PDPICFLIP::getGridBounds()
{
    return m_bounds;
}

void PDPICFLIP::step(fluid_scalar stepSize, fluid_scalar gravity, fluid_scalar flipness, fluid_scalar maxDensity, fluid_scalar densityCorrection, fluid_scalar fluidDensity,
    int maxNumSubstepsFluid, int maxNumSubstepsSolid, float frictionFac, float solidInfluence, bool dontAdvectFluids, int jacobiIts, float colorDeviation, float maxDCorrection,
    bool skipIncompressibility, float velStab)
{
    if (m_firstSimFrame) {
        m_firstSimFrame = false;
        CudaSafeCall(cudaMemcpy(m_origParticlePositionsD, m_particlePositionsD, m_numParticles * 3 * sizeof(float), cudaMemcpyKind::cudaMemcpyDeviceToDevice));
    }

    if (maxDensity < 0) {
        maxDensity = PF_PPC;
    }
    fluidDensity = std::min(1., std::max(0., (double)fluidDensity));
    setParameters_PF(stepSize, gravity, flipness, maxDensity, densityCorrection, fluidDensity, 1. - fluidDensity, frictionFac, solidInfluence, jacobiIts, colorDeviation, maxDCorrection, velStab);
    CudaCheckError();
    CudaSafeSync();
    // Particle velocities to grid velocities (also adds gravitational acceleration and boundary conditions)
    particleVelocitiesToGrid_PF(m_particlePositionsD, m_particleVelocitiesD, m_intermediateSolidParticleVelocitiesD, m_gridVelocitiesD, m_gridVelocitiesOrigD, m_gridDWeightsD, m_gridDensityD, m_particleListD, m_cellListD, m_pPerCellDataD, m_solidNormalsD, m_gridSolidD);
    CudaCheckError();
    CudaSafeSync();
    // Make grid velocities divergence free
    if (!skipIncompressibility) {
        std::pair<float, float> div = correctGridVelocities_PF(m_gridVelocitiesD, m_gridVelocitiesOrigD, m_gridDWeightsD, m_gridDensityD, m_pPerCellDataD, m_gridDivergenceD, m_gridPressure1D, m_gridPressure2D);
    }
    CudaCheckError();
    CudaSafeSync();
    // Transfer velocities from grid back to particles and advect particles
    advectParticles_PF(m_gridVelocitiesD, m_gridVelocitiesOrigD, m_randomVelosD, m_particlePositionsD, m_particleVelocitiesD, dontAdvectFluids, m_solidNormalsD, m_gridSolidD, m_pPerCellDataD, m_particleListD, m_intermediateSolidParticleVelocitiesD, m_glColorArrayPtr, m_origParticlePositionsD);
    CudaCheckError();
    CudaSafeSync();

    // Testing: Check particle velocities
}

struct vec3f {
    float x, y, z;
};

PDPICFLIP* createPicFlip(int gridXZ, int gridY,
    fluid_scalar cellLength, fluid_scalar gap, fluid_scalar mass,
    fluid_scalar gravity, GLuint& glBufferID,
    GLuint& glColorBufferID,
    const Eigen::Matrix<double, -1, -1> & meshParticles,
    const std::vector<Eigen::Matrix<double, 3, 1>> & subspaceSamples,
    fluid_scalar timeStepInitial,
    fluid_scalar subspaceRadius,
    fluid_scalar expansionFactor,
    bool showMeshParticles,
    bool addStream,
    const PFDenseMat& particleNormals,
    bool preventSolidPenetration,
    float forceFloorHeight,
    float streamSize,
    bool forceUnitSquare,
    int seedPPC)
{

    sph_denseMat3 particles(0, 3);
    int numFreeParticles = 0;
    bool alreadyExpanded = false;

    std::cout << "Initial particle spacing: " << cellLength / std::pow(PF_PPC, 1. / 3.) << "m" << std::endl;

    if (meshParticles.rows() > 0) {
        auto mbb = meshBoundingBox(meshParticles);
        float largestSide = std::max(std::max(mbb.second(0) - mbb.first(0), mbb.second(1) - mbb.first(1)), mbb.second(2) - mbb.first(2));
        std::cout << "Longest side of mesh bounding box: " << largestSide << "m" << std::endl;
        if (gridY > 0) {
            float poolFloor = mbb.first(1) - gridY * cellLength;
            if (forceFloorHeight > -1000) {
                poolFloor = forceFloorHeight;
            }
            particles = particlesBox(cellLength,
                mbb.first(0) - (mbb.second(0) - mbb.first(0)) * expansionFactor, std::max(mbb.first(0) - (mbb.second(0) - mbb.first(0)) * expansionFactor + cellLength * 2, mbb.second(0) + (mbb.second(0) - mbb.first(0)) * expansionFactor - gap),
                poolFloor, poolFloor + gridY * cellLength,
                mbb.first(2) - (mbb.second(2) - mbb.first(2)) * expansionFactor, mbb.second(2) + (mbb.second(2) - mbb.first(2)) * expansionFactor,
                seedPPC);
            alreadyExpanded = true;
        }
        if (addStream) {
            SPH::sph_denseMat3 streamParticles = particlesStream(cellLength,
                (mbb.second(0) + mbb.first(0)) / 2.,
                mbb.second(1) + (mbb.second(1) - mbb.first(1)) * 0.5,
                (mbb.second(2) + mbb.first(2)) / 2.,
                largestSide * 0.1 * streamSize, largestSide * 0.6 * streamSize, seedPPC);
            int oldRows = particles.rows();
            particles.conservativeResize(oldRows + streamParticles.rows(), 3);
            particles.block(oldRows, 0, streamParticles.rows(), 3) = streamParticles;
        }
        numFreeParticles = particles.rows();

        sph_denseMat3 joinedParts(meshParticles.rows() + particles.rows(), 3);
        for (int i = 0; i < particles.rows(); i++) {
            joinedParts.row(i) = particles.row(i);
        }
        for (int i = 0; i < meshParticles.rows(); i++) {
            joinedParts.row(i + particles.rows()) = meshParticles.row(i);
        }
        particles = joinedParts;
    }
    else {
        particles = particlesBox(cellLength,
            -0.5 * gridXZ * cellLength, 0.5 * gridXZ * cellLength,
            0, gridY * cellLength,
            -0.5 * gridXZ * cellLength, 0.5 * gridXZ * cellLength, seedPPC);
        numFreeParticles = particles.rows();
    }

    auto bb = meshBoundingBox(particles);

    // Set up bounding box and rest density
    sph_denseMat3 rectBounds(2, 3);
    float lSide = std::max(bb.second(0) - bb.first(0), (bb.second(2) - bb.first(2)));
    rectBounds(0, 0) = bb.first(0) - (alreadyExpanded ? 0 : lSide * expansionFactor);
    rectBounds(1, 0) = bb.second(0) + (alreadyExpanded ? 0 : lSide * expansionFactor) + gap;
    rectBounds(0, 1) = bb.first(1) - ((meshParticles.rows() > 0 && gridY == 0) ? (bb.second(1) - bb.first(1)) : 0);
    rectBounds(1, 1) = bb.second(1) + (bb.second(1) - bb.first(1)) * (0.5 + 0.5 * expansionFactor);
    rectBounds(0, 2) = bb.first(2) - (alreadyExpanded ? 0 : lSide * expansionFactor);
    rectBounds(1, 2) = bb.second(2) + (alreadyExpanded ? 0 : lSide * expansionFactor);
    if (forceFloorHeight > -1000) {
        rectBounds(0, 1) = forceFloorHeight;
    }

    if (forceUnitSquare) {
        rectBounds(0, 0) = rectBounds(0, 1) = rectBounds(0, 2) = 0;
        rectBounds(1, 0) = rectBounds(1, 1) = rectBounds(1, 2) = 1;
    }

    int grid_w = std::ceil((rectBounds(1, 0) - rectBounds(0, 0)) / cellLength);
    int grid_h = std::ceil((rectBounds(1, 1) - rectBounds(0, 1)) / cellLength);
    int grid_d = std::ceil((rectBounds(1, 2) - rectBounds(0, 2)) / cellLength);

    // Initialize PICFLIP
    PDPICFLIP* picflip = new PDPICFLIP(
        grid_w, grid_h, grid_d,
        rectBounds(0, 0), rectBounds(0, 1), rectBounds(0, 2),
        cellLength, timeStepInitial,
        numFreeParticles, meshParticles.rows(),
        SPH::createParticleSubspace(meshParticles.rows(), meshParticles, subspaceSamples, subspaceRadius), 0.1,
        particleNormals, preventSolidPenetration
    );

    picflip->setFreeParticlePositions(particles);

    // OpenGL Buffer stuff
    if (glBufferID != 0) {
        glInvalidateBufferData(glBufferID);
    }
    // Create and fill particle positions array
    //assert(numFreeParticles <= 0 || numFreeParticles > particles.rows());
    std::vector<vec3f> currentParticlePositions;
    currentParticlePositions.resize(showMeshParticles ? particles.rows() : numFreeParticles);
    for (int i = 0; i < currentParticlePositions.size(); i++) {
        vec3f v;
        v.x = (float)particles(i, 0);
        v.y = (float)particles(i, 1);
        v.z = (float)particles(i, 2);
        currentParticlePositions[i] = v;
    }
    if (currentParticlePositions.size() == 0) {
        currentParticlePositions.resize(1);
        vec3f v;
        v.x = 0;
        v.y = 0;
        v.z = 0;
        currentParticlePositions[0] = v;
    }
    // Generate and fill buffer for particle positions
    glGenBuffers(1, &glBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, glBufferID);
    glBufferData(GL_ARRAY_BUFFER, currentParticlePositions.size() * sizeof(float) * 3, currentParticlePositions.data(), GL_STATIC_DRAW);
    GLuint particlesAttributeLoc = 0;
    glVertexAttribPointer(particlesAttributeLoc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(particlesAttributeLoc);

    // Particle Color Buffer
    if (glColorBufferID != 0) {
        glInvalidateBufferData(glColorBufferID);
    }
    glGenBuffers(1, &glColorBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, glColorBufferID);
    std::vector<vec3f> currentParticleColors;
    currentParticleColors.resize(particles.rows());
    for (int i = 0; i < currentParticleColors.size(); i++) {
        vec3f v;
        v.x = 1;
        v.y = 1;
        v.z = 1;
        currentParticleColors[i] = v;
    }
    glBufferData(GL_ARRAY_BUFFER, currentParticleColors.size() * sizeof(float) * 3, currentParticleColors.data(), GL_STATIC_DRAW);
    GLuint particlesColorLoc = 0;
    glVertexAttribPointer(particlesColorLoc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(particlesColorLoc);

    // Initialize direct mapping with CUDA SPH
    picflip->setGLBuffer(glBufferID, showMeshParticles, glColorBufferID);

    CudaCheckError();

    return picflip;
}