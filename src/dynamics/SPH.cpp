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

#include "dynamics/SPH.h"
#include "dynamics/ProjDynUtil.h"
#include <iostream>

using namespace SPH;

#define USE_LINEAR_WEIGHTS false

/*
General Kernel functions
*/

SPH::Kernel3d::Kernel3d(const sph_scalar& h)
{
	set_radius(h);
}

void SPH::Kernel3d::set_radius(sph_scalar h)
{
	m_h = h;
	m_normalization = 1. / pow(h, 3);
}

sph_scalar SPH::Kernel3d::evaluate(const sph_scalar& x)
{
	if (x > getSupportRadius()) return 0;
	sph_scalar xc = x / m_h;

	return eval(xc);
}

sph_scalar SPH::Kernel3d::evaluate(const sph_vec3& v)
{
	sph_scalar x = v.norm();
	if (x > getSupportRadius()) return 0;
	x = x / m_h;

	return eval(x);
}

sph_scalar SPH::Kernel3d::evaluate(const sph_vec3 & v1, const sph_vec3 & v2)
{
	return evaluate(v1 - v2);
}

sph_vec3 SPH::Kernel3d::grad(const sph_vec3& v)
{
	sph_scalar x = v.norm();
	sph_vec3 grad;
	grad.setZero();
	if (x > getSupportRadius() || x < 1e-15) return grad;
	sph_scalar d = deriv(x / m_h);
	if (x > 1e-15) {
		grad = d * (v / (x * m_h));
	}
	return grad;
}

/*
Monaghan's cubic spline Kernel
*/

SPH::CubicSplineKernel3d::CubicSplineKernel3d(const sph_scalar& h)
	:
	Kernel3d(h)
{
	m_k = 8.0 / (PI_D*m_normalization);
	m_l = 48.0 / (PI_D*m_normalization);
}

sph_scalar SPH::CubicSplineKernel3d::eval(const sph_scalar & x)
{
	sph_scalar res = 0.0;
	if (x <= 0.5) {
		const sph_scalar x2 = x*x;
		const sph_scalar x3 = x2*x;
		res = m_k * (6.0*x3 - 6.0*x2 + 1.0);
	}
	else {
		res = m_k * (2.0*pow(1.0 - x, 3));
	}
	return res;
}

sph_scalar SPH::CubicSplineKernel3d::deriv(const sph_scalar& x)
{
	sph_scalar res = 0;
	if (x <= 0.5) {
		res = m_l * x * (3. * x - 2.);
	}
	else {
		const sph_scalar factor = 1.0 - x;
		res = m_l * (-factor * factor);
	}

	return res;
}

sph_scalar SPH::Kernel3d::getSupportRadius()
{
	return m_h;
}




/*
SPH Field methods.
SPH fields are collections of particles that allow for the efficient
evaluation of neighbourhoods, quantities and gradients of quantities.
*/

SPH::SPHField::SPHField(const sph_denseMat3& positions,
	sph_scalar h,
	sph_scalar grid_size_multiplier,
	sph_scalar mass,
	const std::vector<sph_vec3>& velocities,
	const std::vector<sph_scalar>& rest_densities,
	const std::vector<bool>& onBd
)
	:
	m_gridSize(sph_ivec3(-1, -1, -1))
{
	std::cout << "Creating SPH field..." << std::endl;
	// Create particles, using the provided positions, velocities and
	// masses, or using default values if not provided.
	std::cout << "    Creating particles" << std::endl;
	int numParticles = positions.rows();
	m_particles.resize(numParticles);
	m_densities.resize(numParticles);
	m_currentNeighbours.setZero(SPH_NUM_MAX_NEIGHBOURS, numParticles);
	m_currentNumNeighbours.resize(numParticles, 0);
	m_mass = mass;
	for (int i = 0; i < numParticles; i++) {
		Particle* p = new Particle();
		m_particles[i] = p;
		m_densities[i] = -1;
		p->x = positions.row(i);
		if (velocities.size() > i) {
			p->v = velocities[i];
		}
		else {
			p->v = sph_vec3(0, 0, 0);
		}
		p->onBoundaryInitial = false;
		if (onBd.size() > i) {
			p->onBoundaryInitial = onBd[i];
		}
	}

	// Create the Kernel: 
	// If h is not specified (<0), choose h of the kernel as the minimal distance
	// of a particle to its closest neighbour times a multiplier.
	std::cout << "    Creating kernel..." << std::endl;
	if (h < 0) {
		m_minMinDist = std::numeric_limits<sph_scalar>::max();
		for (int i = 0; i < numParticles; i++) {
			Particle* p_i = m_particles[i];
			sph_scalar closest = std::numeric_limits<sph_scalar>::max();
			for (int j = 0; j < numParticles; j++) {
				if (j == i) continue;
				Particle* p_j = m_particles[j];
				sph_scalar dist = (p_i->x - p_j->x).norm();
				if (dist < closest) closest = dist;
			}
			if (closest < m_minMinDist) m_minMinDist = closest;
		}
		m_h = m_minMinDist;
	}
	else {
		m_minMinDist = h;
		m_h = h;
	}
	m_kernel = new CubicSplineKernel3d(m_h);
	m_supportRadius = m_kernel->getSupportRadius();
	m_supportRadius_squared = m_supportRadius * m_supportRadius;
	m_neighboursUpToDate = false;

	// Create the grid
	m_gridSizeMult = grid_size_multiplier;
	std::cout << "    Creating grid..." << std::endl;
	createGrid();
	updateField();
	updateNeighbours();

	// Update particles' rest densities, which is done after grid generation etc.,
	// because if they are not provided, they are chosen as the current densities
	// of the particles (i.e. such that pressure == 0 in the initial configuration).
	std::cout << "    Setting rest densities..." << std::endl;
	sph_scalar w0 = m_kernel->evaluate(0);
	sph_scalar largestDensity = 0;
	for (int i = 0; i < numParticles; i++) {
		sph_scalar d = getDensity(i);
		if (d > largestDensity) {
			largestDensity = d;
		}
	}
	for (int i = 0; i < numParticles; i++) {
		Particle* p = m_particles[i];
		if (rest_densities.size() > i) {
			p->rest_density = rest_densities[i];
		}
		else {
			p->rest_density = largestDensity;
		}
	}
}

sph_scalar SPH::SPHField::getParticleMass()
{
	return m_mass;
}

void SPH::SPHField::createGrid()
{
	sph_vec3 center;
	sph_vec3 diameter = getDiameter(&center);

	m_gridCellLength = m_supportRadius;

	int gridSize = (int)std::floor((diameter.maxCoeff() / m_gridCellLength) * (double)m_gridSizeMult);

	m_gridSize = sph_ivec3(gridSize, gridSize, gridSize);
	m_layerSize = m_gridSize[0] * m_gridSize[1];
	m_grid = new std::forward_list<int>[gridSize * gridSize * gridSize];

	m_gridCornerPos = center;
	sph_scalar offSet = ((double)gridSize / 2.) * m_gridCellLength;
	m_gridCornerPos(0) -= offSet;
	m_gridCornerPos(1) -= offSet;
	m_gridCornerPos(2) -= offSet;

	m_neighboursUpToDate = false;
	m_gridInit = false;

	m_lastGridIndices.resize(m_particles.size());
	for (int i = 0; i < m_particles.size(); i++) {
		m_lastGridIndices[i] = -1;
	}
}

void SPH::SPHField::sortGrid()
{
	int numParticles = m_particles.size();
	int gridMaxIndex = m_layerSize * m_gridSize[2] - 1;
	// If the grid was uninitialized, clear all grid cell lists
	if (!m_gridInit) {
		for (int ind = 0; ind < gridMaxIndex; ind++) {
			m_grid[ind].clear();
		}
	}

	for (int i = 0; i < numParticles; i++) {
		// Get new index for current particle
		sph_ivec3 curInd = getParticleIndex(i);
		// Compute flat index
		int newIndex = flattenIndex(curInd);
		// If old grid indices exist, check where the particle was before in the grid...
		if (m_gridInit && m_lastGridIndices.size() > i) {
			// ... if it didn't leave the old cell, do nothing.
			if (newIndex == m_lastGridIndices[i]) {
				continue;
			}
			// Otherwise, remove it from the old cell.
			else {
				m_grid[m_lastGridIndices[i]].remove(i);
			}
		}
		// Put the particle in the new cell
		m_grid[newIndex].push_front(i);
		// And remember its index
		m_lastGridIndices[i] = newIndex;
	}

	m_gridInit = true;
	m_neighboursUpToDate = false;
}

sph_scalar SPH::SPHField::getKernelValue(const sph_vec3 & v)
{
	return m_kernel->evaluate(v);
}

sph_vec3 SPH::SPHField::getKernelGradient(const sph_vec3 & v)
{
	return m_kernel->grad(v);
}

sph_vec3 SPH::SPHField::getDiameter(sph_vec3 * center)
{
	sph_vec3 xMax = sph_vec3(std::numeric_limits<sph_scalar>::min(), std::numeric_limits<sph_scalar>::min(), std::numeric_limits<sph_scalar>::min());
	sph_vec3 xMin = sph_vec3(std::numeric_limits<sph_scalar>::max(), std::numeric_limits<sph_scalar>::max(), std::numeric_limits<sph_scalar>::max());

	int numParticles = m_particles.size();
	for (int i = 0; i < numParticles; i++) {
		sph_vec3& curPos = m_particles[i]->x;
		for (int dim = 0; dim < 3; dim++) {
			if (curPos(dim) < xMin(dim)) {
				xMin(dim) = curPos(dim);
			}
			if (curPos(dim) > xMax(dim)) {
				xMax(dim) = curPos(dim);
			}
		}
	}

	m_boundingBox = std::pair<sph_vec3, sph_vec3>(xMin, xMax);

	if (center) {
		*center = (xMin + xMax) / 2;
	}
	return xMax - xMin;
}

void SPH::SPHField::makeNeighbours(const int& i)
{
	// If neighbours are up to date, return a copy from the internal list.
	if (m_neighboursUpToDate) {
		return;
	}

	// Otherwise, compute new local neighbourhood
	// Always add yourself
	m_currentNeighbours(0, i) = i;
	m_currentNumNeighbours[i] = 1;
	// Check all 28 nearby cells surrounding the current particle
	sph_ivec3 curInd = getLocationIndex(m_particles[i]->x);
	for (int xOff = -1; xOff <= 1; xOff++) {
		for (int yOff = -1; yOff <= 1; yOff++) {
			for (int zOff = -1; zOff <= 1; zOff++) {
				sph_ivec3 neighborInd = sph_ivec3(curInd(0) + xOff, curInd(1) + yOff, curInd(2) + zOff);
				// Skip out-of-bounds cells
				bool skip = false;
				for (int dim = 0; dim < 3; dim++) {
					if (neighborInd(dim) < 0) skip = true;
					if (neighborInd(dim) >= m_gridSize(dim)) skip = true;
				}
				if (skip) continue;
				int flatIndex = flattenIndex(neighborInd);
				auto& cell = m_grid[flatIndex];
				for (auto& it = cell.begin(); it != cell.end(); it++)
				{
					sph_vec3& x_j = m_particles[(*it)]->x;
					if (*it != i && (x_j - m_particles[i]->x).squaredNorm() < m_supportRadius_squared) {
						m_currentNeighbours(m_currentNumNeighbours[i], i) = *it;
						m_currentNumNeighbours[i]++;
						if (m_currentNumNeighbours[i] >= SPH_NUM_MAX_NEIGHBOURS) {
							return;
						}
					}
				}
			}
		}
	}
}

void SPH::SPHField::getLocationNeighbours(const sph_vec3 & x, std::vector<int>& neighbours)
{
	// Check all 28 nearby cells surrounding the current particle
	neighbours.clear();
	sph_ivec3 curInd = getLocationIndex(x);
	for (int xOff = -1; xOff <= 1; xOff++) {
		for (int yOff = -1; yOff <= 1; yOff++) {
			for (int zOff = -1; zOff <= 1; zOff++) {
				sph_ivec3 neighborInd = sph_ivec3(curInd(0) + xOff, curInd(1) + yOff, curInd(2) + zOff);
				// Skip out-of-bounds cells
				bool skip = false;
				for (int dim = 0; dim < 3; dim++) {
					if (neighborInd(dim) < 0) skip = true;
					if (neighborInd(dim) >= m_gridSize(dim)) skip = true;
				}
				if (skip) continue;
				int flatIndex = flattenIndex(neighborInd);
				auto& cell = m_grid[flatIndex];
				for (auto& it = cell.begin(); it != cell.end(); it++)
				{
					sph_vec3& x_j = m_particles[(*it)]->x;
					if ((x_j - x).squaredNorm() < m_supportRadius_squared) {
						neighbours.push_back(*it);
					}
				}
			}
		}
	}
}

int SPH::SPHField::getNumNeighbours(int i)
{
	// If neighbours are up to date, return a copy from the internal list.
	if (m_neighboursUpToDate) {
		return m_currentNumNeighbours[i];
	}
	else {
		makeNeighbours(i);
		return m_currentNumNeighbours[i];
	}
}

sph_denseMati& SPH::SPHField::getNeighbours()
{
	if (!m_neighboursUpToDate) {
		updateNeighbours();
	}
	return m_currentNeighbours;
}

const std::vector<int>& SPH::SPHField::getNumNeighbours()
{
	if (!m_neighboursUpToDate) {
		updateNeighbours();
	}
	return m_currentNumNeighbours;
}

void SPH::SPHField::updateNeighbours()
{
	m_neighboursUpToDate = false;
	int numParticles = m_particles.size();

	// In parallel, compute neighbourhoods
	SPH_PARALLEL_FOR
		for (int i = 0; i < numParticles; i++) {
			makeNeighbours(i);
		}

	m_neighboursUpToDate = true;
}

void SPH::SPHField::updateField()
{
	sortGrid();
	int numParticles = m_particles.size();
	SPH_PARALLEL_FOR
		for (int i = 0; i < numParticles; i++) {
			m_densities[i] = -1;
		}
	m_neighboursUpToDate = false;
}

sph_scalar SPH::SPHField::evaluateKernel(sph_scalar x)
{
	return m_kernel->evaluate(x);
}

sph_scalar SPH::SPHField::getDensity(const int& i)
{
	if (i < 0 || i > m_particles.size()) return 0;
	if (m_densities[i] < 0) {
		// Evaluate density and store the result
		const sph_ivec& neighbours = getNeighbours().col(i);
		int numNeighbours = m_currentNumNeighbours[i];
		const sph_vec3& x = m_particles[i]->x;
		sph_scalar density = 0;
		for (int jInd = 0; jInd < numNeighbours; jInd++) {
			int j = neighbours(jInd);
			density += m_kernel->evaluate(x, m_particles[j]->x) * m_mass;
		}
		m_densities[i] = density;
	}

	return m_densities[i];
}

sph_scalar SPH::SPHField::getDensity(const sph_vec3 & x)
{
	std::vector<int> neighbours;
	getLocationNeighbours(x, neighbours);
	sph_scalar density = 0;
	for (int jInd = 0; jInd < neighbours.size(); jInd++) {
		int j = neighbours[jInd];
		density += m_kernel->evaluate(x, m_particles[j]->x) * m_mass;
	}
	return density;
}

void SPH::SPHField::updateDensities()
{
	int numParticles = m_particles.size();
	SPH_PARALLEL_FOR
		for (int i = 0; i < numParticles; i++) {
			getDensity(i);
		}
}

Particle * SPH::SPHField::getParticle(int i)
{
	return m_particles[i];
}

void SPH::SPHField::setParticlePos(int i, const sph_vec3 & p)
{
	m_particles[i]->x = p;
}

const std::pair<sph_vec3, sph_vec3>& SPH::SPHField::getBoundingBox()
{
	return m_boundingBox;
}

sph_ivec3 SPH::SPHField::getParticleIndex(int i)
{
	return getLocationIndex(m_particles[i]->x);
}

sph_ivec3 SPH::SPHField::getLocationIndex(const sph_vec3& x)
{
	// Compute x, y and z indices
	sph_vec3 curP = x;
	curP -= m_gridCornerPos;
	curP /= m_gridCellLength;
	// Rounding
	sph_ivec3 ind = sph_ivec3((int)std::floor(curP(0)), (int)std::floor(curP(1)), (int)std::floor(curP(2)));
	// Trimming
	trimIndex(ind);
	return ind;
}

int SPH::SPHField::flattenIndex(sph_ivec3 ind)
{
	return ind(0) + ind(1) * m_gridSize[0] + ind(2) * m_layerSize;
}

void SPH::SPHField::trimIndex(sph_ivec3 & ind)
{
	// Trim x, y, z indices to fit the particle into the grid if it is outside
	for (int dim = 0; dim < 3; dim++) {
		if (ind(dim) < 0) ind(dim) = 0;
		if (ind(dim) >= m_gridSize(dim)) ind(dim) = m_gridSize(dim) - 1;
	}
}







/*
Implementation of Projective Fluids constraints.
*/

SPH::PressureConstraints::PressureConstraints(const sph_denseMat3& particlePos, sph_scalar h,
	int numMeshParticles,
	const std::vector<sph_vec3>& subspaceSamples, sph_scalar subspaceRadius,
	sph_scalar particleMass, const std::vector<bool>& onBd)
	:
	m_sph(particlePos, h, 3, particleMass, std::vector<sph_vec3>(), std::vector<sph_scalar>(), onBd)
{
	m_h = h;
	m_particleSpacing = m_h / 2.;
	m_numParticles = particlePos.rows();
	m_numMeshParticles = numMeshParticles;
	m_maxStepSquared = std::pow(m_h, 2);
	if (m_numMeshParticles < 0) {
		m_numMeshParticles = m_numParticles;
	}
	m_projections.resize(m_numParticles);
	m_deltaC.resize(m_numParticles);
	m_inverseNeighbours.resize(m_numParticles);
	for (int i = 0; i < m_numParticles; i++) {
		m_projections[i].resize(SPH_NUM_MAX_NEIGHBOURS);
		m_deltaC[i].resize(SPH_NUM_MAX_NEIGHBOURS);
		m_inverseNeighbours[i].reserve(SPH_NUM_MAX_NEIGHBOURS);
	}
	m_neighbourCount.resize(m_numParticles);
	m_fullProjection.setZero(m_numParticles, 3);
	m_particlePositions.setZero(m_numParticles, 3);
	m_weights.setZero(m_numParticles);


	if (subspaceSamples.size() > 0) {
		std::cout << "Creating particle subspace..." << std::endl;
		m_reduced = true;
		m_subspace = createParticleSubspace(m_numMeshParticles, particlePos, subspaceSamples, subspaceRadius);
		setupSubspaceProjection();
		m_subProjection.setZero(m_subspace.cols(), 3);
		m_tempLHSMat.setZero(m_subspace.cols(), m_numParticles);
		m_globalStepSubLHS.setZero(m_subspace.cols(), m_subspace.cols());
		// Manual sparsification of subspace matrices (using Eigen's sparse
		// matrices is extremely slow)
		m_subColNNZ.resize(m_subspace.cols());
		m_subColEntries.resize(m_subspace.cols(), std::vector<sph_scalar>());
		m_subColRowInds.resize(m_subspace.cols(), std::vector<int>());
		int cols = m_subspace.cols();
		int rows = m_subspace.rows();
		SPH_PARALLEL_FOR
			for (int j = 0; j < cols; j++) {
				m_subColNNZ[j] = 0;
				for (int i = 0; i < rows; i++) {
					if (std::abs(m_subspace(i, j)) > SPH_SPARSITY_EPS) {
						m_subColNNZ[j] += 1;
						m_subColEntries[j].push_back(m_subspace(i, j));
						m_subColRowInds[j].push_back(i);
					}
				}
			}
		m_subRowNNZ.resize(m_numParticles);
		m_subRowColInds.resize(m_numParticles, std::vector<int>());
		SPH_PARALLEL_FOR
			for (int i = 0; i < rows; i++) {
				m_subRowNNZ[i] = 0;
				for (int j = 0; j < cols; j++) {
					if (std::abs(m_subspace(i, j)) > SPH_SPARSITY_EPS) {
						m_subRowNNZ[i] += 1;
						m_subRowColInds[i].push_back(j);
					}
				}
			}
		// Analyze sparsity pattern of U^T D U for a diagonal matrix D and the subspace U
		m_lhsNNZ.reserve(m_subspace.cols() * m_subspace.cols() / 2);
		m_globalStepSubLHS = m_subspace.transpose() * m_subspace;
		for (int i = 0; i < m_subspace.cols(); i++) {
			for (int j = 0; j <= i; j++) {
				if (std::abs(m_globalStepSubLHS(i, j)) > SPH_SPARSITY_EPS) {
					m_lhsNNZ.push_back(std::pair<int, int>(i, j));
				}
			}
		}
	}
	else {
		m_subspace = sph_denseMat(0, 0);
		m_globalStepSubLHS = sph_denseMat(0, 0);
		m_reduced = false;
	}
}

void SPH::PressureConstraints::updateProjections(bool computeSubRHS, bool updateField)
{
	if (updateField) {
		m_sph.updateField();
		m_sph.updateNeighbours();
		m_sph.updateDensities();
	}

	m_weights.setZero();

	sph_denseMati& neighbours = m_sph.getNeighbours();
	const std::vector<int>& numNeighbours = m_sph.getNumNeighbours();

	// Compute projections
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numParticles; i++) {
			computeProjection(i, m_projections[i]);
		}

	// Build "inverted neighbourhoods", i.e. for each particle i determine
	// the local index where the k-th neighbour has particle i in his list
	// of neighbours
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numParticles; i++) {
			m_inverseNeighbours[i].clear();
			const sph_ivec& curNeighbours = neighbours.col(i);
			for (int jInd = 0; jInd < numNeighbours[i]; jInd++) {
				int j = curNeighbours(jInd);
				const sph_ivec&  neighbourNeighbours = neighbours.col(j);
				bool found = false;
				for (int kInd = 0; kInd < numNeighbours[j]; kInd++) {
					if (neighbourNeighbours(kInd) == i) {
						m_inverseNeighbours[i].push_back(kInd);
						found = true;
						break;
					}
				}
				if (!found) {
					m_inverseNeighbours[i].push_back(-1);
				}
			}
		}

	// Accumulate all projections to build RHS term p_full
	// This can be done in parallel by collecting, for each particle, all
	// contributions via projcections of its neighbours
	// (this works since a a particle is always its neighbours neighbour).
	m_fullProjection.setZero();
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numParticles; i++) {
			const sph_ivec& curN = neighbours.col(i);
			auto& curIN = m_inverseNeighbours[i];
			for (int j = 0; j < numNeighbours[i]; j++) {
				if (curIN[j] >= 0) m_fullProjection.row(i) += m_projections[curN(j)][curIN[j]] * m_weights(curN[j]);
			}
		}

	if (m_reduced && computeSubRHS) {
		// Multiply U^T * p_full
		m_subProjection.setZero();
		int sizeSub = m_subspace.cols();
		SPH_PARALLEL_FOR
			for (int j = 0; j < sizeSub; j++) {
				for (int iInd = 0; iInd < m_subColRowInds[j].size(); iInd++) {
					int i = m_subColRowInds[j][iInd];
					sph_scalar entry = m_subColEntries[j][iInd];
					m_subProjection.row(j) += m_fullProjection.row(i) * entry;
				}
			}
	}

	updateWeights();
}

void SPH::PressureConstraints::computeProjection(int i, std::vector<sph_vec3>& projection)
{
	int numNeighbours = m_sph.getNumNeighbours()[i];
	int indexInNeighbours = -1;
	const sph_ivec& neighbours = m_sph.getNeighbours().col(i);

	// The initial guess is the positions of the particles.
	// Also collect the neighbour's masses:
	for (int j = 0; j < numNeighbours; j++) {
		if (neighbours(j) == i) indexInNeighbours = j;
		projection[j] = m_sph.getParticle(neighbours(j))->x;
	}
	if (indexInNeighbours < 0) {
		return;
	}

	// Compute initial density, can reuse particle density values, since
	// initial guess for projection are the particle positions
	sph_scalar invRestD = 1. / m_sph.getParticle(i)->rest_density;
	sph_scalar C = 0;
	for (int j = 0; j < numNeighbours; j++) {
		C += getParticleMass() * m_sph.getKernelValue(projection[indexInNeighbours] - projection[j]);
	}
	C *= invRestD;
	C -= 1;

	if (C > SPH_PROJECTION_EPS1) {
		m_weights(i) = C;
	}
	else {
		m_weights(i) = 0;
	}

#ifdef SPH_SIMPLE_PROJECTION
	// This projection only works if the particle spacing is 0.5 * h.
	if (C > SPH_PROJECTION_EPS1 && numNeighbours > 1) {
		for (int j = 0; j < numNeighbours; j++) {
			if (j == indexInNeighbours) continue;
			sph_vec3 v = projection[j] - projection[indexInNeighbours];
			sph_scalar vn = v.norm();
			// Particles closer than the average spacing are moved to the average spacing
			if (vn < m_particleSpacing) {
				v *= (m_particleSpacing + 1e-10) / vn;
				projection[j] = projection[indexInNeighbours] + v;
			}
			// Don't push other particles out of bounds
			if (m_hasRectBounds) {
				for (int d = 0; d < 3; d++) {
					projection[j](d) = std::max(std::min(projection[j](d), m_rectBounds(1, d)), m_rectBounds(0, d));
				}
			}
		}
	}
#else
	std::vector<sph_vec3>& deltaC = m_deltaC[i];
	deltaC.resize(numNeighbours);
	int iter = 0;
	while (C > SPH_PROJECTION_EPS1 && iter < SPH_PROJECTION_MAX_ITER) {
		iter++;
		// Reset gradient
		for (int j = 0; j < numNeighbours; j++) {
			deltaC[j].setZero();
		}
		// Compute gradient
		for (int j = 0; j < numNeighbours; j++) {
			if (j == indexInNeighbours) continue;
			sph_vec3 grad = getParticleMass() * m_sph.getKernelGradient(projection[j] - projection[indexInNeighbours]);
			deltaC[indexInNeighbours] += grad;
			deltaC[j] -= grad;
		}
		// Compute gradient squared norm
		sph_scalar deltaCSqNorm = 0;
		for (int j = 0; j < numNeighbours; j++) {
			deltaC[j] *= invRestD;
			deltaCSqNorm += deltaC[j].squaredNorm();
		}
		// Check for convergence
		if (deltaCSqNorm < SPH_PROJECTION_EPS2) {
			break;
		}
		// Update projection and recompute C
		sph_scalar factor = C / (deltaCSqNorm + 1e-9);
		for (int j = 0; j < numNeighbours; j++) {
			deltaC[j] *= factor;
			projection[j] += deltaC[j];
			// Don't push other particles out of bounds
			if (m_hasRectBounds) {
				for (int d = 0; d < 3; d++) {
					projection[j](d) = std::max(std::min(projection[j](d), m_rectBounds(1, d)), m_rectBounds(0, d));
				}
			}
		}
		sph_scalar C_before = C;
		C = 0;
		for (int j = 0; j < numNeighbours; j++) {
			C += getParticleMass() * m_sph.getKernelValue(projection[indexInNeighbours] - projection[j]);
		}
		C *= invRestD;
		C -= 1;
		if (C > C_before) {
			//std::cout << "ERROR: pressure projection raises functional value!" << std::endl;
			break;
		}
	}
#endif

	// Check for rect bounds for this particle and set weight to 1 if out of bounds
	if (m_hasRectBounds) {
		int j = indexInNeighbours;
		for (int d = 0; d < 3; d++) {
			if (projection[j](d) < m_rectBounds(0, d)) {
				projection[j](d) = m_rectBounds(0, d);
				m_weights(i) = 1;
			}
			if (projection[j](d) > m_rectBounds(1, d)) {
				projection[j](d) = m_rectBounds(1, d);
				m_weights(i) = 1;
			}
		}
	}
}

sph_denseMat SPH::createParticleSubspace(int numMeshParticles, const sph_denseMat3& particlePos, const std::vector<sph_vec3>& subspaceSamples, sph_scalar r)
{
	// Create a subspace for particle positions in exactly the same 
	// fashion than the one that is created for
	// the mesh.
	sph_denseMat subspace;
	subspace.setZero(numMeshParticles, subspaceSamples.size() * 4);
	sph_scalar a = (1. / std::pow(r, 4.));
	sph_scalar b = -2. * (1. / (r * r));
	SPH_PARALLEL_FOR
		for (int i = 0; i < numMeshParticles; i++) {
			sph_vec weights;
			weights.setZero(subspaceSamples.size());
			// For each sample...
			for (int j = 0; j < subspaceSamples.size(); j++) {
				// Evaluate radiual basis function
				double curDist = (particlePos.row(i) - subspaceSamples[j].transpose()).norm();
				double val = 0;
				if (curDist < 0 || curDist >= r) {
					val = 0;
				}
				else if (USE_LINEAR_WEIGHTS) {
					val = 1. - (curDist / r);
				}
				else {
					val = a * std::pow(curDist, 4.) + b * (curDist * curDist) + 1;
				}
				weights(j) = val;
			}
			// Partition of unity
			sph_scalar sum = weights.sum();
			if (sum < 1e-10) {
				std::cout << "Warning: a vertex isn't properly covered by any of the radial basis functions!" << std::endl;
				weights.setConstant(1 / (sph_scalar)subspaceSamples.size());
			}
			else {
				weights /= sum;
			}
			// Put entries in subspace matrix (4 per weight)
			for (int j = 0; j < subspaceSamples.size(); j++) {
				for (int d = 0; d < 3; d++) {
					subspace(i, j * 4 + d) = weights(j) * particlePos(i, d);
				}
				subspace(i, j * 4 + 3) = weights(j);
			}
		}

	return subspace;
}

void SPH::PressureConstraints::setSubspaceProjectionRegularizer(sph_scalar reg) {
	m_subspaceProjectionRegularize = reg;
	setupSubspaceProjection();
}

void SPH::PressureConstraints::setRectBounds(sph_scalar xMin, sph_scalar xMax, sph_scalar yMin, sph_scalar yMax, sph_scalar zMin, sph_scalar zMax)
{
	m_rectBounds.setZero(2, 3);
	m_rectBounds(0, 0) = xMin;
	m_rectBounds(0, 1) = yMin;
	m_rectBounds(0, 2) = zMin;
	m_rectBounds(1, 0) = xMax;
	m_rectBounds(1, 1) = yMax;
	m_rectBounds(1, 2) = zMax;

	m_hasRectBounds = true;
}



const sph_denseMati& SPH::PressureConstraints::getNeighbours()
{
	return m_sph.getNeighbours();
}

const std::vector<int>& SPH::PressureConstraints::getNumNeighbours()
{
	return m_sph.getNumNeighbours();
}

Particle * SPH::PressureConstraints::getParticle(int i)
{
	return m_sph.getParticle(i);
}

void SPH::PressureConstraints::updateNeighbours()
{
	m_sph.updateNeighbours();
}

void SPH::PressureConstraints::sortGrid()
{
	m_sph.sortGrid();
}

sph_scalar SPH::PressureConstraints::evaluateKernel(sph_scalar x)
{
	return m_sph.evaluateKernel(x);
}

sph_scalar SPH::PressureConstraints::computeBoundaryPressure(int numBounds)
{
	switch (numBounds) {
	case 1:
		return getParticleMass() * (evaluateKernel(m_particleSpacing) + 4. * evaluateKernel(sqrt(2) * m_particleSpacing) + 4. * evaluateKernel(sqrt(3) * m_particleSpacing));
	case 2:
		return getParticleMass() * (2. * evaluateKernel(m_particleSpacing) + 7. * evaluateKernel(sqrt(2) * m_particleSpacing) + 6. * evaluateKernel(sqrt(3) * m_particleSpacing));
	case 3:
		return getParticleMass() * (3. * evaluateKernel(m_particleSpacing) + 9. * evaluateKernel(sqrt(2) * m_particleSpacing) + 7. * evaluateKernel(sqrt(3) * m_particleSpacing));
	}
	return 0;
}

void SPH::PressureConstraints::setupSubspaceProjection() {
	// Set up subspace projection:
	m_subspaceProjectionRHS.setZero(m_subspace.cols(), 3);
	m_subspaceProjectionResult.setZero(m_subspace.cols(), 3);
	sph_denseMat subProjLHS = m_subspace.transpose() * m_subspace;
	if (m_subspaceProjectionRegularize > 0) {
		for (int j = 0; j < m_subspace.cols(); j++) {
			subProjLHS(j, j) += m_subspaceProjectionRegularize;
		}
	}
	m_subspaceProjectionSolver.compute(subProjLHS);
}

const sph_denseMat3 & SPH::PressureConstraints::getRectBounds()
{
	return m_rectBounds;
}

const sph_denseMat3& SPH::PressureConstraints::getParticlePositions() {
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numParticles; i++) {
			m_particlePositions.row(i) = m_sph.getParticle(i)->x;
		}
	return m_particlePositions;
}

void SPH::PressureConstraints::updateWeights()
{
	const sph_denseMati& neighbours = m_sph.getNeighbours();

	// Reset neighbour counts 
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numParticles; i++) {
			m_neighbourCount[i] = 0;
		}

	// Collect new neighbour counts
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numParticles; i++) {
			const std::vector<int>& curNI = m_inverseNeighbours[i];
			int numNeighbours = m_sph.getNumNeighbours()[i];
			for (int j = 0; j < numNeighbours; j++) {
				if (curNI[j] >= 0) m_neighbourCount[i] += m_weights(neighbours(j, i));
				;
			}
		}
}

void SPH::PressureConstraints::setSubspaceCoords(const sph_denseMat3 & subspaceCoords)
{
	SPH_PARALLEL_FOR
		for (int i = 0; i < m_numMeshParticles; i++) {
			sph_vec3 p(0., 0., 0.);
			for (int jInd = 0; jInd < m_subRowNNZ[i]; jInd++) {
				int j = m_subRowColInds[i][jInd];
				p += subspaceCoords.row(j) * m_subspace(i, j);
			}
			m_sph.setParticlePos(i, p);
		}
}

void SPH::PressureConstraints::setParticlePositions(const sph_denseMat3 & pos)
{
	int nRows = std::min((int)pos.rows(), m_numParticles);
	SPH_PARALLEL_FOR
		for (int i = 0; i < nRows; i++) {
			m_sph.setParticlePos(i, pos.row(i));
		}
}

const sph_denseMat& SPH::PressureConstraints::globalStepSubMatrix()
{
	if (m_reduced) {
		// Build and return U^T sum_i S_i^T S_i,
		// where the matrix S_i^T S_i has already been computed
		// as m_neighbourCount
		int nnz = m_lhsNNZ.size();
		SPH_PARALLEL_FOR
			for (int ind = 0; ind < nnz; ind++) {
				int i = m_lhsNNZ[ind].first;
				int j = m_lhsNNZ[ind].second;
				sph_scalar entry = 0;
				// We abuse the sparsity pattern of U^T, looping only
				// non-zeroes of the i-th column of U^T (i.e. the i-th
				// column of U).
				// TODO: further reduce operations by precomputing, 
				// for each pair i,j, the indices of the entries which
				// are non-zero in the sum for the computation of (U^T U)_ij
				for (int jInd = 0; jInd < m_subColNNZ[i]; jInd++) {
					int k = m_subColRowInds[i][jInd];
					entry += m_subspace(k, i) * m_subspace(k, j) * m_neighbourCount[k];
				}
				m_globalStepSubLHS(i, j) = entry;
				m_globalStepSubLHS(j, i) = entry;
			}
	}

	return m_globalStepSubLHS;
}

const std::vector<sph_scalar>& SPH::PressureConstraints::getWeights()
{
	return m_neighbourCount;
}

const sph_denseMat& SPH::PressureConstraints::getRHS(bool alwaysFull)
{
	if (m_reduced && !alwaysFull) {
		return m_subProjection;
	}

	return m_fullProjection;
}

const sph_denseMat& SPH::PressureConstraints::getParticleSubspace()
{
	return m_subspace;
}

sph_scalar SPH::PressureConstraints::getParticleMass()
{
	return m_sph.getParticleMass();
}

sph_denseMat3 & SPH::PressureConstraints::projectParticlesToSubspace(const sph_denseMat3& prevSubCoords, const sph_denseMat3& precomputedRHS)
{
	if (precomputedRHS.rows() == m_subspace.rows()) {
		m_subspaceProjectionRHS = precomputedRHS + prevSubCoords * m_subspaceProjectionRegularize;
	}
	else {
		getParticlePositions();
		// TODO speed-up product by sparsity and parallelization
		m_subspaceProjectionRHS = m_subspace.transpose() * m_particlePositions.block(0, 0, m_numMeshParticles, 3);
		if (m_subspaceProjectionRegularize > 0 && prevSubCoords.rows() == m_subspace.cols()) {
			m_subspaceProjectionRHS += prevSubCoords * m_subspaceProjectionRegularize;
		}
	}
	SPH_PARALLEL_FOR
		for (int d = 0; d < 3; d++) {
			m_subspaceProjectionResult.col(d) = m_subspaceProjectionSolver.solve(m_subspaceProjectionRHS.col(d));
		}
	return m_subspaceProjectionResult;
}





/*
 Auxilary functions
*/

sph_denseMat3 SPH::particlesFromTetMesh(const sph_denseMat3 & verts, const sph_denseMati4 & tets, 
	sph_scalar gridSize, const std::pair<sph_vec3, sph_vec3>& boundingBox, std::vector<bool>& onBoundary,
	sph_denseMati3& tris, sph_denseMat3& particleNormals)
{
	std::cout << "Creating particles from tet-mesh..." << std::endl;

	// Create a boolean grid whose points will be true if they are within a tet of the mesh.
	sph_ivec3 gridSizes(0, 0, 0);
	for (int dim = 0; dim < 3; dim++) {
		gridSizes(dim) = std::round((boundingBox.second(dim) - boundingBox.first(dim)) / gridSize);
	}
	gridSizes += sph_ivec3(2, 2, 2);
	sph_vec3 topLeft = boundingBox.first - sph_vec3(gridSize / 2., gridSize / 2., gridSize / 2.);
	bool*** particles = new bool**[gridSizes(0)];
	SPH_PARALLEL_FOR
		for (int x = 0; x < gridSizes(0); x++) {
			particles[x] = new bool*[gridSizes(1)];
			for (int y = 0; y < gridSizes(1); y++) {
				particles[x][y] = new bool[gridSizes(2)];
				for (int z = 0; z < gridSizes(2); z++) {
					particles[x][y][z] = false;
				}
			}
		}

	// For each tet, check for nearby grid points, if they are within this tet
	int numTets = tets.rows();
	SPH_PARALLEL_FOR
		for (int tet = 0; tet < numTets; tet++) {
			const sph_vec3& v1 = verts.row(tets(tet, 0));
			const sph_vec3& v2 = verts.row(tets(tet, 1));
			const sph_vec3& v3 = verts.row(tets(tet, 2));
			const sph_vec3& v4 = verts.row(tets(tet, 3));
			std::pair<sph_vec3, sph_vec3> bb = tetBoundingBox(v1, v2, v3, v4);

			// Iterate over grid points that lie within the bounding box
			// Start by finding the top-, left-, backmost point that lies on the grid,
			// and within the bounding box of the tet.
			sph_vec3 curGridPoint = (bb.first - topLeft) / gridSize;
			sph_ivec3 curIndex(0, 0, 0);
			for (int d = 0; d < 3; d++) {
				curIndex(d) = std::ceil(curGridPoint(d));
				curGridPoint(d) = curIndex(d) * gridSize + topLeft(d);
			}
			sph_vec3 startGridPoint = curGridPoint;
			sph_ivec3 startIndex = curIndex;
			while (curGridPoint(0) <= bb.second(0) && curIndex(0) >= 0 && curIndex(0) < gridSizes(0)) {
				while (curGridPoint(1) <= bb.second(1) && curIndex(1) >= 0 && curIndex(1) < gridSizes(1)) {
					while (curGridPoint(2) <= bb.second(2) && curIndex(2) >= 0 && curIndex(2) < gridSizes(2)) {
						if (pointInTetrahedron(v1, v2, v3, v4, curGridPoint)) {
							particles[curIndex(0)][curIndex(1)][curIndex(2)] = true;
						}
						curGridPoint(2) += gridSize;
						curIndex(2)++;
					}
					curGridPoint(2) = startGridPoint(2);
					curIndex(2) = startIndex(2);
					curGridPoint(1) += gridSize;
					curIndex(1)++;
				}
				curGridPoint(1) = startGridPoint(1);
				curIndex(1) = startIndex(1);
				curGridPoint(0) += gridSize;
				curIndex(0)++;
			}
		}

	sph_vec3* gridNormals = nullptr;
	if (tris.rows() > 0) {
		std::cout << "	Making normals..." << std::endl;
		gridNormals = new sph_vec3[gridSizes(0) * gridSizes(1) * gridSizes(2)];
		transferTriangleNormalsToParticles(verts, tris, gridSizes, boundingBox.first, gridSize, particles, gridNormals);
	}

	std::cout << "	Finalizing..." << std::endl;
	// Now collect all "true"s in the grid and turn them into particles, also collect boundary particles and make normals
	sph_denseMat3 particleMat(gridSizes(0) * gridSizes(1) * gridSizes(2), 3);
	if (tris.rows() > 0) {
		particleNormals.setZero(gridSizes(0) * gridSizes(1) * gridSizes(2), 3);
	}
	int ind = 0;
	for (int x = 0; x < gridSizes(0); x++) {
		for (int y = 0; y < gridSizes(1); y++) {
			for (int z = 0; z < gridSizes(2); z++) {
				if (particles[x][y][z]) {

					// Check if this is a boundary vertex (less than 8 neighbours)
					bool onBd = false;
					for (int xx = x - 1; xx <= x + 1; xx++) {
						for (int yy = y - 1; yy <= y + 1; yy++) {
							for (int zz = z - 1; zz <= z + 1; zz++) {
								if (xx >= 0 && xx < gridSizes(0) && yy >= 0 && yy < gridSizes(1) && zz >= 0 && zz < gridSizes(2)) {
									// Neighbour index valid, if no particle at this position, set to boundary particle
									if (!particles[xx][yy][zz]) onBd = true;
								}
								else {
									// Neighbour index invalid, so it must be a boundary particle
									onBd = true;
								}
							}
						}
					}
					onBoundary.push_back(onBd);

					// Add particle
					particleMat.row(ind) = sph_vec3(
						topLeft(0) + gridSize * x,
						topLeft(1) + gridSize * y,
						topLeft(2) + gridSize * z);

					// Add normals if triangles are available
					if (tris.rows() > 0) { // && onBd) {
						sph_vec3 closestNormal = gridNormals[x * gridSizes(1) * gridSizes(2) + y * gridSizes(2) + z];
						particleNormals.row(ind) = closestNormal;
					}

					ind++;
				}
			}
		}
	}
	particleMat.conservativeResize(ind, 3);
	if (tris.rows() > 0) {
		particleNormals.conservativeResize(ind, 3);
	}

	std::cout << "Storing particle mesh..." << std::endl;
	PD::storeMesh(particleMat, sph_denseMati3(0, 3), "D:\\particleMesh.obj", particleNormals);
	if (gridNormals) delete gridNormals;
	if (particles) delete particles;
	return particleMat;
}

void SPH::transferTriangleNormalsToParticles(const sph_denseMat3 & verts, const sph_denseMati3 & tris, const sph_ivec3 & box_size, const sph_vec3 & box_corner, double cellLength, bool *** particles, sph_vec3 * gridNormals)
{
	double* distsToTris = new double[box_size(0) * box_size(1) * box_size(2)];
	for (int j = 0; j < box_size(0) * box_size(1) * box_size(2); j++) {
		distsToTris[j] = 0;
		gridNormals[j].setZero();
	}
	// For each outer triangle of the mesh
	for (int t = 0; t < tris.rows(); t++) {
		// Compute the triangle normal
		sph_vec3* tvs = new sph_vec3[3];
		for (int j = 0; j < 3; j++) tvs[j] = verts.row(tris(t, j));
		sph_vec3 normal = (tvs[1] - tvs[0]).cross(tvs[2] - tvs[0]);
		normal.normalize();
		// Get the triangle bounding box
		sph_vec3 lBd = sph_vec3(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
			uBd = sph_vec3(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min());
		for (int j = 0; j < 3; j++) {
			for (int d = 0; d < 3; d++) {
				if (tvs[j](d) < lBd(d)) lBd(d) = tvs[j](d);
				if (tvs[j](d) > uBd(d)) uBd(d) = tvs[j](d);
			}
		}
		// Convert bounding box to grid indices (and extend a bit)
		int extend = 2;
		for (int d = 0; d < 3; d++) {
			lBd(d) = std::max(0., std::floor((lBd(d) - box_corner(d)) / cellLength) - extend);
			uBd(d) = std::min((double)box_size(d), std::ceil((uBd(d) - box_corner(d)) / cellLength) + extend);
		}
		// For each grid point in that triangle's bounding box:
		for (int xx = lBd(0); xx < uBd(0); xx++) {
			for (int yy = lBd(1); yy < uBd(1); yy++) {
				for (int zz = lBd(2); zz < uBd(2); zz++) {
					if (particles[xx][yy][zz]) {
						// Compute the distance of the corresponding grid point to the current triangle
						sph_vec3 p(xx * cellLength + box_corner(0), yy * cellLength + box_corner(1), zz * cellLength + box_corner(2));
						double dist = PD::distPointToTriangle(tvs, p);
						if (1. / dist > distsToTris[xx * box_size(1) * box_size(2) + yy * box_size(2) + zz]) {
							gridNormals[xx * box_size(1) * box_size(2) + yy * box_size(2) + zz] = normal;
							distsToTris[xx * box_size(1) * box_size(2) + yy * box_size(2) + zz] = 1. / dist;
						}
					}
				}
			}
		}

		delete tvs;
	}

	/*
	for (int j = 0; j < box_size(0) * box_size(1) * box_size(2); j++) {
		if (distsToTris[j] > 0) {
			gridNormals[j] /= distsToTris[j];
			if (std::abs(gridNormals[j].norm() - 1) > 1e-3) {
				gridNormals[j].normalize();
			}
		}
	}
	*/

	delete distsToTris;
}

 

sph_denseMat3 SPH::particlesBoxBoundary(sph_denseMat3& particles, sph_scalar gridSize, sph_scalar xMin, sph_scalar xMax, sph_scalar yMin, sph_scalar yMax, sph_scalar zMin, sph_scalar zMax)
{
	sph_ivec3 sizes;
	sizes(0) = std::round((xMax - xMin) / gridSize);
	sizes(1) = std::round((yMax - yMin) / gridSize);
	sizes(2) = std::round((zMax - zMin) / gridSize);

	std::vector<sph_vec3> boundaryParticles;
	for (int xInd = -2; xInd < sizes(0) + 2; xInd++) {
		for (int yInd = -2; yInd < sizes(1) * 3; yInd++) {
			for (int zInd = -2; zInd < sizes(2) + 2; zInd++) {
				if (((xInd < 0 || xInd >= sizes(0)) || (zInd < 0 || zInd >= sizes(2))) || yInd < 0) {
					sph_vec3 p;
					p(0) = (double)xInd * gridSize + xMin;
					p(1) = (double)yInd * gridSize + yMin;
					p(2) = (double)zInd * gridSize + zMin;
					boundaryParticles.push_back(p);
				}
			}
		}
	}
	int oldSize = particles.rows();
	particles.conservativeResize(oldSize + boundaryParticles.size(), 3);
	for (int i = 0; i < boundaryParticles.size(); i++) {
		particles.row(oldSize + i) = boundaryParticles[i];
	}

	return particles;
}

sph_denseMat3 SPH::particlesStream(sph_scalar gridSize,
	sph_scalar xMid, sph_scalar yMid, sph_scalar zMid,
	sph_scalar radius, sph_scalar height,
	int ppc) {

	int ySize = height / gridSize;
	int xzSize = 2. * radius / gridSize;
	sph_scalar xMin = xMid - radius;
	sph_scalar yMin = yMid;
	sph_scalar zMin = zMid - radius;

	std::vector<sph_vec3> parts;

	for (int xInd = 0; xInd < xzSize; xInd++) {
		for (int yInd = 0; yInd < ySize; yInd++) {
			for (int zInd = 0; zInd < xzSize; zInd++) {
				for (int i = 0; i < ppc; i++) {
					double px = (double)xInd * gridSize + xMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (gridSize)));
					double py = (double)yInd * gridSize + yMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (gridSize)));
					double pz = (double)zInd * gridSize + zMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (gridSize)));
					if ((px - xMid) * (px - xMid) + (pz - zMid) * (pz - zMid) < radius * radius) {
						parts.push_back(sph_vec3(px, py, pz));
					}
				}
			}
		}
	}

	sph_denseMat3 particles;
	particles.setZero(parts.size(), 3);
	for (int i = 0; i < parts.size(); i++) {
		particles(i, 0) = parts[i](0);
		particles(i, 1) = parts[i](1);
		particles(i, 2) = parts[i](2);
	}

	return particles;
}

sph_denseMat3 SPH::particlesBox(sph_scalar gridSize, 
	sph_scalar xMin, sph_scalar xMax, sph_scalar yMin, 
	sph_scalar yMax, sph_scalar zMin, sph_scalar zMax,
	int ppc)
{
	sph_ivec3 sizes;
	sizes(0) = std::round((xMax - xMin) / gridSize);
	sizes(1) = std::round((yMax - yMin) / gridSize);
	sizes(2) = std::round((zMax - zMin) / gridSize);

	sph_denseMat3 particles;
	//std::cout << "Number of additional free particles: " << (sizes(0) * sizes(1) * sizes(2) * ppc) << std::endl;
	particles.setZero(sizes(0) * sizes(1) * sizes(2) * ppc, 3);

	int ind = 0;
	for (int xInd = 0; xInd < sizes(0); xInd++) {
		for (int yInd = 0; yInd < sizes(1); yInd++) {
			for (int zInd = 0; zInd < sizes(2); zInd++) {
				for (int i = 0; i < ppc; i++) {
					particles(ind, 0) = (double)xInd * gridSize + xMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (gridSize)));
					particles(ind, 1) = (double)yInd * gridSize + yMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (gridSize)));
					particles(ind, 2) = (double)zInd * gridSize + zMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (gridSize)));
					ind++;
				}
			}
		}
	}

	return particles;
}

std::pair<sph_vec3, sph_vec3> SPH::meshBoundingBox(const sph_denseMat3 & verts)
{
	sph_vec3 xMax = sph_vec3(std::numeric_limits<sph_scalar>::min(), std::numeric_limits<sph_scalar>::min(), std::numeric_limits<sph_scalar>::min());
	sph_vec3 xMin = sph_vec3(std::numeric_limits<sph_scalar>::max(), std::numeric_limits<sph_scalar>::max(), std::numeric_limits<sph_scalar>::max());

	int numVerts = verts.rows();
	for (int i = 0; i < numVerts; i++) {
		sph_vec3 curPos = verts.row(i);
		for (int dim = 0; dim < 3; dim++) {
			if (curPos(dim) < xMin(dim)) {
				xMin(dim) = curPos(dim);
			}
			if (curPos(dim) > xMax(dim)) {
				xMax(dim) = curPos(dim);
			}
		}
	}

	return std::pair<sph_vec3, sph_vec3>(xMin, xMax);
}

bool SPH::sameSide(const sph_vec3& v1, const sph_vec3& v2, const sph_vec3& v3, const sph_vec3& v4, const sph_vec3& p)
{
	sph_vec3 normal = (v2 - v1).cross(v3 - v1);
	sph_scalar dotV4 = normal.dot(v4 - v1);
	sph_scalar dotP = normal.dot(p - v1);
	return (dotV4 <= 0 && dotP <= 0) || (dotV4 >= 0 && dotP >= 0);
}

bool SPH::pointInTetrahedron(const sph_vec3& v1, const sph_vec3& v2, const sph_vec3& v3, const sph_vec3& v4, const sph_vec3& p)
{
	return (sameSide(v1, v2, v3, v4, p) &&
		sameSide(v2, v3, v4, v1, p) &&
		sameSide(v3, v4, v1, v2, p) &&
		sameSide(v4, v1, v2, v3, p));
}

std::pair<sph_vec3, sph_vec3> SPH::tetBoundingBox(const sph_vec3& v1, const sph_vec3& v2, const sph_vec3& v3, const sph_vec3& v4) {
	sph_vec3 xMax = v1;
	sph_vec3 xMin = v1;

	for (int d = 0; d < 3; d++) {
		if (v2(d) < xMin(d)) xMin(d) = v2(d);
		if (v3(d) < xMin(d)) xMin(d) = v3(d);
		if (v4(d) < xMin(d)) xMin(d) = v4(d);

		if (v2(d) > xMax(d)) xMax(d) = v2(d);
		if (v3(d) > xMax(d)) xMax(d) = v3(d);
		if (v4(d) > xMax(d)) xMax(d) = v4(d);
	}

	return std::pair<sph_vec3, sph_vec3>(xMin, xMax);
}