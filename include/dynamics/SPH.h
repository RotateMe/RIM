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
#include <vector>
#include <forward_list>
#include <set>
#include "SPH_defines.h"

namespace SPH {

	typedef double sph_scalar;
	typedef Eigen::Matrix<sph_scalar, -1, 1> sph_vec;
	typedef Eigen::Matrix<int, -1, 1> sph_ivec;
	typedef Eigen::Matrix<sph_scalar, 3, 1> sph_vec3;
	typedef Eigen::Matrix<int, 3, 1> sph_ivec3;
	typedef Eigen::Matrix<sph_scalar, -1, -1, Eigen::ColMajor> sph_denseMat;
	typedef Eigen::Matrix<int, -1, -1, Eigen::ColMajor> sph_denseMati;
	typedef Eigen::Matrix<sph_scalar, -1, 3, Eigen::ColMajor> sph_denseMat3;
	typedef Eigen::SparseMatrix<sph_scalar, Eigen::ColMajor> sph_sparseMat;
	typedef Eigen::Matrix<int, -1, 3, Eigen::ColMajor> sph_denseMati3;
	typedef Eigen::Matrix<int, -1, 4, Eigen::ColMajor> sph_denseMati4;
	typedef Eigen::LLT<sph_denseMat> sph_denseSolve;

	class Kernel3d {
	protected:
		sph_scalar m_h;
		sph_scalar m_normalization;
	public:
		Kernel3d(const sph_scalar& h);
		void set_radius(sph_scalar h);
		sph_scalar evaluate(const sph_scalar&);
		sph_scalar evaluate(const sph_vec3&);
		sph_scalar evaluate(const sph_vec3& v1, const sph_vec3& v2);
		virtual sph_scalar eval(const sph_scalar& x) = 0;
		sph_vec3 grad(const sph_vec3&);
		virtual sph_scalar deriv(const sph_scalar&) = 0;
		sph_scalar getSupportRadius();
	};

	class CubicSplineKernel3d : public Kernel3d {
	private:
		sph_scalar m_k, m_l;
	public:
		CubicSplineKernel3d(const sph_scalar& h);
		sph_scalar eval(const sph_scalar& x);
		sph_scalar deriv(const sph_scalar&);
	};

	class Particle {
	public:
		sph_vec3 x, v;
		sph_scalar rest_density;
		bool onBoundaryInitial = false;
	};

	class SPHField {
	private:
		/* The fixed mass of each particle (which is the same, always) */
		sph_scalar m_mass;
		/* The list of particles. Their index in this array will remain constant. */
		std::vector<Particle*> m_particles;
		/*
		A grid that is used for faster evaluation of neighborhoods, since each cell's diameter is the
		support radius of the kernel.
		The grid is 3-dimensional, the size of each side is given in m_gridSize, the indices are
		1-dimensional, however, and can be obtained using flattenIndex().
		Each cell contains a list of indices into the m_particles list.
		*/
		std::forward_list<int>* m_grid;
		/*
		The minimum of the distances to the closest neighbour in the initial configuration.
		Used to specify h of the kernel.
		*/
		sph_scalar m_minMinDist;
		/* The grid will be m_gridSizeMult times as large as is required to fit the initial particles. */
		sph_scalar m_gridSizeMult = 10.;
		/* x, y and z size (in terms of cells) of the current grid. */
		sph_ivec3 m_gridSize;
		/* x-size * y-size, precomputed in createGrid() */
		int m_layerSize;
		/*
		The lengths of the sides of the cells of the grid. Should always be the support radius of
		he current kernel.
		*/
		sph_scalar m_gridCellLength;
		/* The coordinates corresponding to the top, left, back corner of the cell with index (0,0,0).
		Used to compute the index of a particle in the grid. */
		sph_vec3 m_gridCornerPos;
		/* List of the indices of each particle in the grid since the last call of sortGrid() */
		std::vector<int> m_lastGridIndices;
		/*
		Uses the current positions of the particles, the kernel's support size and the grid size
		multiplier, to initialize a grid that can fit all particles and sufficient surrounding
		space.
		*/
		void createGrid();
		/*
		Upper, back, left point and lower, front, right point
		*/
		std::pair<sph_vec3, sph_vec3> m_boundingBox;
		/*
		Whenever a new grid has been created (grid size changed, etc.) this should be set to false
		to let the sort function do the extra work for the first time it puts the particles into
		the grid.
		*/
		bool m_gridInit = false;
		/*
		Restricts an x, y, z index to the current grid
		*/
		void trimIndex(sph_ivec3& ind);
		/*
		Converts an x, y, z index on grid into an index into the actual 1-dimensional array representing
		the grid.
		*/
		int flattenIndex(sph_ivec3 ind);
		/*
		The kernel and it's h value.
		*/
		sph_scalar m_h;
		Kernel3d* m_kernel;
		/*
		The support radius of the current kernel. Based on this, the grid is created and
		particle neighbourhoods are defined.
		*/
		sph_scalar m_supportRadius;
		/*
		Precomputed squared support radius of the current kernel, used for faster distance
		checks.
		*/
		sph_scalar m_supportRadius_squared;
		/*
		If neighbours are up to date, this array contains, for every particle i, the indices
		of all particles within the support radius of the kernel.
		The number of neighbours for each particle is stored in m_currentNumNeighbours, so
		only the first few entries into this array make sense.
		NOTE: neighbours of particle i include i itself.
		*/
		sph_denseMati m_currentNeighbours;
		std::vector<int> m_currentNumNeighbours;
		/*
		If false, the list m_currentNeighbours is not up to data and a call to getNeighbours()
		will trigger a updateNeighbours call first.
		*/
		bool m_neighboursUpToDate = false;
		/*
		A list of current particle densities, which gets reset, every time updateField() is
		called.
		If a density is available, the corresponding entry is positive, otherwise it is
		negative and getDensity(i) must be called.
		*/
		std::vector<sph_scalar> m_densities;

	public:
		/*
		Provide a list of particle positions to initialize an SPH field with particles
		that are ordered as in the positions array.
		The parameter h for the kernel can be specified, or left negative, in which case
		it is chosen to be the minimum of the closest distances of each particle to a
		neighbour.
		The grid_size_multiplier determines the support size of the grid which is used
		internally to efficiently determine particle neighbourhoods. Specifically the
		grid will be a cube whose side-lengths are grid_size_multiplier times as large
		as the diameter of the initial particle configuration, and its center is the
		geometric center of the initial particles' positions.
		Optionally, velocities, masses, and rest densities can be provided, which are
		otherwise set to (0,0,0), 1 and 1 respectively.
		*/
		SPHField(const sph_denseMat3& positions,
			sph_scalar h = -1.,
			sph_scalar grid_size_multiplier = 10.,
			sph_scalar masses = 1.,
			const std::vector<sph_vec3>& velocities = std::vector<sph_vec3>(),
			const std::vector<sph_scalar>& rest_densities = std::vector<sph_scalar>(),
			const std::vector<bool>& onBd = std::vector<bool>()
		);
		/*
		Returns the fixed mass of each particle
		*/
		sph_scalar getParticleMass();
		/*
		Returns the value of the current kernel with the difference vector v.
		*/
		sph_scalar getKernelValue(const sph_vec3& v);
		/*
		Returns the gradient of the kernel function w.r.t. the difference vector v.
		*/
		sph_vec3 getKernelGradient(const sph_vec3& v);
		/*
		Returns the x, y and z diameter of the current particle configuration. Optionally
		returns the position of the geometric center of the current particles, disregarding
		mass.
		*/
		sph_vec3 getDiameter(sph_vec3* center = nullptr);
		/*
		If the internal list of current neighbours is up to date, nothing happens,
		otherwise, a single neighbourhood is computed anew.
		Afterwards, the updated neighbours can be requested via getNeighbours(i, 1 ... getNumNeighbours()[i]);
		Only the first getNumNeighbours()[i] entries refer to indices, the rest is
		filled for efficiency reasons.
		NOTE: neighbours of particle i include i itself.
		WARNING: this method assumes that updateField() has been called since the last time
		the particles' positions have changed!!!
		*/
		void makeNeighbours(const int& i);
		/*
		Returns the number of neighbours of a particle i. Fast evaluation if internal
		neighbourhood list is up to date.
		*/
		int getNumNeighbours(int i);
		/*
		Return a list of particle indices that are closer to a location x than the support
		radius of the current kernel. If x happens to be a particle, use getNeighbours()
		instead!
		*/
		void SPH::SPHField::getLocationNeighbours(const sph_vec3 & x, std::vector<int>& neighbours);
		/*
		Return a list of particle indices that are closer to particle i than the support
		radius of the current kernel.
		This returns a list that is created once, every time the grid has been updated
		via updateField(). I.e. calling this function checks whether the neighbourhoods
		are up to date, and if not, trigger a recomputation.
		NOTE: neighbours of particle i include i itself.
		WARNING: this method assumes that sortGrid() has been called since the last time
		the particles' positions have changed!!!
		*/
		sph_denseMati& getNeighbours();
		/*
		Returns the sizes of each neighbourhood, i.e. how many of the entries returned by
		getNeighbours() refer to actual neighbours.
		*/
		const std::vector<int>& getNumNeighbours();
		/*
		Make sure, that the grid is up-to-date, in the sense that every cell of the grid contains
		exactly the particles i, for which getParticleIndex(i) is that cell's index.
		*/
		void sortGrid();
		/*
		Forces an updates of the internal list of current neighbours, such that the calls
		to getNeighbours() are precomputed.
		A call to the getNeighbours() method without arguments, triggers this call if it is
		required, while a call to getNeighbours(i) (which only computes the neighbourhood for
		one particle) will not trigger this call, but instead computes but doesn't store the
		local neighbourhood (unless the neighbourhood list is up to date, in which case it
		returns a copy from there.
		*/
		void updateNeighbours();
		/*
		IMPORTANT: call this method after particle positions have changed.
		This will trigger the call to sortGrid(), and will lead to recomputation of the neighbourhoods
		*if* they are requested.
		*/
		void updateField();
		/*
		Evaluate a quantity at a non-particle position, via the SPH formula, using the current
		kernel, where the quantities at the particles are passed as an array, indexed in the order
		from which the SPH field was constructed.
		*/
		sph_scalar evaluateQuantity(const std::vector<sph_scalar>& quantity, const sph_vec3& atPos);
		/*
		Evaluate the underlying Kernel.
		*/
		sph_scalar evaluateKernel(sph_scalar x);
		/*
		Evaluate a quantity at a particle with index i, via the SPH formula, using the current
		kernel, where the quantities at the particles are passed as an array, indexed in the order
		from which the SPH field was constructed.
		If you will evaluate this quantity at most vertices, force an update to the neighbourhoods
		via updateNeighbours() beforehand.
		If you will evaluate this quantity at ALL vertices, us evaluateQuantity(quantity) instead.
		*/
		sph_scalar evaluateQuantity(const std::vector<sph_scalar>& quantity, int i);
		/*
		Evaluate a quantity at all particles, via the SPH formula, using the current
		kernel, where the quantities at the particles are passed as an array, indexed in the order
		from which the SPH field was constructed.
		If not already up-to-date, this call will trigger an update to the neighbourhood list.
		*/
		std::vector<sph_scalar> evaluateQuantity(const std::vector<sph_scalar>& quantity);
		/*
		Returns x, y and z index of the cell in which a particle should reside (after sortGrid()).
		To get an index into the grid (which is a 1-dim array) flattenIndex() needs to be used.
		*/
		sph_ivec3 getParticleIndex(int i);
		/*
		Returns x, y and z index of the cell in which a location x should reside (after sortGrid()).
		To get an index into the grid (which is a 1-dim array) flattenIndex() needs to be used.
		*/
		sph_ivec3 getLocationIndex(const sph_vec3& x);
		/*
		Computes and returns the density of a particle. The value will be stored and reused in
		further calls to this method, until updateField() is called.
		*/
		sph_scalar getDensity(const int& i);
		/*
		Returns the density at a location x.
		*/
		sph_scalar getDensity(const sph_vec3& x);
		/*
		Preocmputes all densities, such that getDensity(i) will be precomputed
		*/
		void updateDensities();
		/*
		Returns a pointer to a particle
		*/
		Particle* getParticle(int i);
		/*
		Set a particle's position
		*/
		void setParticlePos(int i, const sph_vec3& p);
		/*
		Returns the two points defining the bounding box of the initial configuration of the particles.
		Does NOT get updated when particles move!
		*/
		const std::pair<sph_vec3, sph_vec3>& getBoundingBox();
	};


	class PressureConstraints {
	private:
		SPHField m_sph;
		std::vector<std::vector<sph_vec3>> m_projections;
		std::vector<std::vector<sph_vec3>> m_deltaC;
		sph_denseMat m_fullProjection;
		sph_denseMat m_subProjection;
		sph_denseMat m_tempLHSMat;
		sph_denseMat m_globalStepSubLHS;
		void computeProjection(int i, std::vector<sph_vec3>& projection);
		int m_numParticles = 0;
		sph_scalar m_h;
		sph_scalar m_particleSpacing;
		bool m_reduced = false;
		sph_vec m_weights;
		sph_scalar m_maxStepSquared;

		std::vector< std::vector<int> > m_inverseNeighbours;

		sph_denseMat m_subspace;
		std::vector<int> m_subRowNNZ;
		std::vector<std::vector<int>> m_subRowColInds;
		std::vector<int> m_subColNNZ;
		std::vector<std::vector<sph_scalar>> m_subColEntries;
		std::vector<std::vector<int>> m_subColRowInds;
		std::vector<sph_scalar> m_neighbourCount;
		void updateWeights();
		std::vector<std::pair<int, int>> m_lhsNNZ;
		sph_denseMat3 m_particlePositions; // Only up-to-date when getParticlePositions gets called!!
		sph_denseMat3 m_subspaceProjectionResult;
		sph_denseSolve m_subspaceProjectionSolver;
		sph_denseMat3 m_subspaceProjectionRHS;
		sph_scalar m_subspaceProjectionRegularize = 0;
		void setupSubspaceProjection();

		int m_numMeshParticles = -1;

		sph_denseMat3 m_rectBounds;
		bool m_hasRectBounds = false;
	public:
		const sph_denseMat3& getRectBounds();
		/* Updates and returns a reference to the particle positions matrix */
		const sph_denseMat3& getParticlePositions();
		/* Constructs a set of pressure constraints for particles listed in
		the particlePos matrix, given a kernel support size h.
		For reduced constraints, you should pass a set of sample positions from which
		a skinning space is constructed. This should only be used for rigid or deformable
		bodies on which other inner forces act to keep the general structure intact,
		otherwise the subspace is too restrictive.
		If no reduction should be used, pass an empty list here. */
		PressureConstraints(const sph_denseMat3& particlePos, sph_scalar h, 
			int numMeshParticles,
			const std::vector<sph_vec3>& subspaceSamples, sph_scalar subspaceRadius,
			sph_scalar particleMass, const std::vector<bool>& onBd = std::vector<bool>());
		/* Updates the sph field (i.e. recomputes neighbourhoods) and
		then computes, in parallel, all projections. */
		void updateProjections(bool computeSubRHS = true, bool updateField = true);
		/* Only for reduced constraints: pass the vector of subspace coefficients,
		which will be used to update particle positions.
		NOTE: the SPH field will not be updated after making this call (this will only be
		done in the updateProjections() method). */
		void setSubspaceCoords(const sph_denseMat3& subspaceCoords);
		/* Set positions for each particle. Can be used for full and reduced constraints.
		NOTE: the SPH field will not be updated after making this call (this will only be
		done in the updateProjections() method). */
		void setParticlePositions(const sph_denseMat3& pos);
		/* For reduced pressure constraints, return U^T \sum_i S_i^T S_I U, otherwise returns empty matrix.
		NOTE: For full pressure constraints, the LHS matrix is simply the diagonal matrix with entries from getNeighbourCounts() */
		const sph_denseMat& globalStepSubMatrix();
		/* Returns the vector that contains the number of neighbours
		of each particle, where each count is multiplied by the corresponding constraint's weight.
		Thus, this is the diagonal matrix for the global step. */
		const std::vector<sph_scalar>& getWeights();
		/* If reduced, returns U^T \sum_i S_i p_i, if full return \sum S_i p_i
		Assumes that updateProjections() has been used first. */
		const sph_denseMat& getRHS(bool alwaysFull = false);
		/* If reduced, returns the particle subspace. */
		const sph_denseMat& getParticleSubspace();
		/* The fixed mass of each particle */
		sph_scalar getParticleMass();
		/* Projects the current particle positions to the subspace
		   (if the subspace has been defined). */
		sph_denseMat3& projectParticlesToSubspace(const sph_denseMat3& prevSubCoords, const sph_denseMat3& precomputedRHS = sph_denseMat3(0, 3));
		/* Set a regularization weight for subspace projections */
		void setSubspaceProjectionRegularizer(sph_scalar reg);
		/*
		Set box bounds for the particles x-, y- and z-coordinates
		*/
		void setRectBounds(sph_scalar xMin = -1., sph_scalar xMax = 1., sph_scalar yMin = 0., sph_scalar yMax = 5, sph_scalar zMin = -1, sph_scalar zMax = 1);
		/*
		Returns the list current neighbours for all particles
		*/
		const sph_denseMati& getNeighbours();
		/*
		Returns the sizes of each neighbourhood, i.e. how many of the entries returned by
		getNeighbours() refer to actual neighbours.
		*/
		const std::vector<int>& getNumNeighbours();
		/*
		Returns a reference to the i-th particle
		*/
		Particle* getParticle(int i);
		/*
		Once updated positions have been propagated, this can be used to update particle neighbourhoods.
		Before calling this method, sortGrid() has to be called.
		*/
		void updateNeighbours();
		/*
		Puts each particle into the grid cell associated to its position
		*/
		void sortGrid();
		/*
		Evaluate the underlying kernel.
		*/
		sph_scalar evaluateKernel(sph_scalar x);
		/*
		Computes the amount of density to add in the pressure computation for boundary particles.
		These are assumed to lie on a flat boundary, and that the boundary is made of regularly spaced
		particles with the same mass as the fluid.
		numBounds is the number of surrounding planar boundaries and can only be between 1 and 3.
		*/
		sph_scalar computeBoundaryPressure(int numBounds);
	};

	sph_denseMat3 particlesFromTetMesh(const sph_denseMat3& verts, const sph_denseMati4& tets,
		sph_scalar gridSize, const std::pair<sph_vec3, sph_vec3>& boundingBox,
		std::vector<bool>& onBoundary,
		sph_denseMati3& tris, sph_denseMat3& particleNormals);

	void transferTriangleNormalsToParticles(const sph_denseMat3& verts, const sph_denseMati3& tris,
		const sph_ivec3& box_size, const sph_vec3& box_corner, double cellLength, bool*** particles, sph_vec3* gridNormals);

	sph_denseMat3 particlesBox(sph_scalar gridSize, sph_scalar xMin = -1., sph_scalar xMax = 1., sph_scalar yMin = 0., sph_scalar yMax = 5, sph_scalar zMin = -1, sph_scalar zMax = 1, int particlesPerCell = 0);
	sph_denseMat3 particlesStream(sph_scalar gridSize,
		sph_scalar xMid, sph_scalar yMid, sph_scalar zMid,
		sph_scalar radius, sph_scalar height,
		int ppc);
	sph_denseMat3 particlesBoxBoundary(sph_denseMat3& particles, sph_scalar gridSize, sph_scalar xMin = -1., sph_scalar xMax = 1., sph_scalar yMin = 0., sph_scalar yMax = 5, sph_scalar zMin = -1, sph_scalar zMax = 1);

	sph_denseMat createParticleSubspace(int numMeshParticles, const sph_denseMat3& particlePos,
		const std::vector<sph_vec3>& subspaceSamples, sph_scalar subspaceRadius);

	std::pair<sph_vec3, sph_vec3> meshBoundingBox(const sph_denseMat3& verts);

	bool sameSide(const sph_vec3& v1, const sph_vec3& v2, const sph_vec3& v3, const sph_vec3& v4, const sph_vec3& p);

	bool pointInTetrahedron(const sph_vec3& v1, const sph_vec3& v2, const sph_vec3& v3, const sph_vec3& v4, const sph_vec3& p);

	std::pair<sph_vec3, sph_vec3> tetBoundingBox(const sph_vec3& v1, const sph_vec3& v2, const sph_vec3& v3, const sph_vec3& v4);
}