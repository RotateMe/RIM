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

#include <Eigen\Dense>
#include <Eigen\Sparse>
#include "ProjDynTypeDef.h"
#include <queue>

#define STVD_MODE_GRAPH_DIJKSTRA 0
#define STVD_MODE_ST_DIJKSTRA 1
#define STVD_DEFAULT_K 10

using namespace PD;

typedef std::pair<unsigned int, float> QueVert;

class QueVertComp 
{
public:
	bool operator() (QueVert v1, QueVert v2) {
		return v1.second > v2.second;
	}
};

class STVD {

public:
	STVD();

	void init(PDPositions const& verts, PDTriangles const& tris = PDTriangles(0, 3), PDTets const& tets = PDTets(0, 4));

	STVD(PDPositions const& verts, PDTriangles const& tris = PDTriangles(0, 3), PDTets const& tets = PDTets(0, 4));

	/*
	Removes all source points from the mesh.
	*/
	void resetSources();

	/*
	Marks a vertex as a source, such that computeDistances() will yield the minimal geodesic distances
	to this vertex and all other marked vertices.
	*/
	void addSource(unsigned int vertexInd);

	/*
	Fills the vector distances with per-vertex values that correspond to the geodesic distance to the
	nearest marked vertex.
	*/
	void computeDistances(bool update = false, unsigned int mode = STVD_MODE_GRAPH_DIJKSTRA, PDScalar maxDist = -1);

	/*
	Sets all distances to -1 (i.e. infinity), so even if computeDistances is run with update == true,
	the distances will be new.
	*/
	void resetDistances();

	/* Returns the current distance of the vector with index vInd to the sources set via addSource. 
	   Requires updated distances via computeDistances()! */
	double getDistance(unsigned int vInd);
	PDVector& getDistances();

	/* Set the parameter k which defines the size of the short term memory. Only
	   affects the process if the mode is set to STVD_MODE_ST_DIJKSTRA. */
	void setK(unsigned int k);

private:
	unsigned int m_numVerts;
	bool m_forTets;
	std::vector<unsigned int> m_sources;
	std::vector<std::vector<unsigned int>> m_neighbours;
	unsigned int m_k;
	unsigned int m_mode;
	bool m_isUpdated;
	PDVector m_distances;
	PDPositions m_positions;
	PDPositions m_outerVertNormals;

	double updateVertDist(unsigned int v1, unsigned int v2, std::vector<int>& predecessors);
};