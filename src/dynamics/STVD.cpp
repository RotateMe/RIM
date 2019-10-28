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
#include <iostream>
#include <limits>

#include "dynamics/STVD.h"
#include "dynamics/ProjDynUtil.h"

STVD::STVD()
{
}

void STVD::init(PDPositions const& verts, PDTriangles const& tris, PDTets const& tets)
{
	m_numVerts = verts.rows();
	m_neighbours.resize(m_numVerts);
	m_mode = 0;
	m_k = STVD_DEFAULT_K;
	m_distances.resize(m_numVerts);
	m_distances.setConstant(-1);
	m_isUpdated = false;
	m_positions = verts;

	/* Establish neighbourhood structure for tets or triangles */
	if (tets.rows() > 0) {
		m_forTets = true;
		for (unsigned int tet = 0; tet < tets.rows(); tet++) {
			for (unsigned int locV1 = 0; locV1 < 4; locV1++) {
				int locV1Ind = tets(tet, locV1);
				for (unsigned int locV2 = 0; locV2 < 4; locV2++) {
					if (locV2 != locV1) {
						int locV2Ind = tets(tet, locV2);
						if (std::find(m_neighbours.at(locV1Ind).begin(), m_neighbours.at(locV1Ind).end(), locV2Ind) == m_neighbours.at(locV1Ind).end()) {
							m_neighbours.at(locV1Ind).push_back(locV2Ind);
						}
					}
				}
			}
		}
	}
	else {
		PDTriangles trisCopy = tris;
		PDPositions vertsCopy = verts;
		m_outerVertNormals = PD::getVertexNormals(trisCopy, vertsCopy, -1);
		for (unsigned int tri = 0; tri < tris.rows(); tri++) {
			for (unsigned int locV1 = 0; locV1 < 3; locV1++) {
				int locV1Ind = tris(tri, locV1);
				for (unsigned int locV2 = 0; locV2 < 3; locV2++) {
					if (locV2 != locV1) {
						int locV2Ind = tris(tri, locV2);
						if (std::find(m_neighbours.at(locV1Ind).begin(), m_neighbours.at(locV1Ind).end(), locV2Ind) == m_neighbours.at(locV1Ind).end()) {
							m_neighbours.at(locV1Ind).push_back(locV2Ind);
						}
					}
				}
			}
		}
	}

}

STVD::STVD(PDPositions const& verts, PDTriangles const& tris, PDTets const& tets)
{
	init(verts, tris, tets);
}

void STVD::resetSources()
{
	m_sources.clear();
}

void STVD::addSource(unsigned int vertexInd)
{
	if (vertexInd < m_numVerts) {
		m_isUpdated = false;
		m_sources.push_back(vertexInd);
	}
}

void STVD::computeDistances(bool update, unsigned int mode, PDScalar maxDist)
{
	m_mode = mode;

	/* Initialize distances, queue, predecessor list and "is final" status */
	if (!update) m_distances.setConstant(-1.);
	std::priority_queue<QueVert, std::vector<QueVert>, QueVertComp > queue;
	std::vector<int> predecessors;
	predecessors.resize(m_numVerts, -1);
	std::vector<bool> isFinal;
	isFinal.clear();
	isFinal.resize(m_numVerts, false);

	/* Add source vertices into queue */
	for (unsigned int v : m_sources) {
		m_distances[v] = 0;
		QueVert vd;
		vd.first = v;
		vd.second = 0.;
		queue.push(vd);
	}

	/* Main loop */
	while (!queue.empty()) {
		/* Extract vertex with shortest distance to sources */
		QueVert curV = queue.top();
		queue.pop();
		/* We check if vertex has already been finalized, since we don't check if vertices are
		already in the queue when inserting them. */
		if (!isFinal[curV.first]) {
			/* Vertex gets finalized. */
			isFinal[curV.first] = true;
			/* If this vertex is beyond maximal distance, do not add neighbours and stop distance evaluation */
			if (maxDist > 0 && m_distances[curV.first] > maxDist) {
				continue;
			}
			/* For all non-finalized neighbours... */
			for (unsigned int nv : m_neighbours[curV.first]) {
				if (!isFinal[nv]) {
					/* ... update their distance and predecessor if new distance is better. */
					double updatedDistance = updateVertDist(curV.first, nv, predecessors);
					if (m_distances[nv] < 0 || updatedDistance < m_distances[nv]) {
						/* Update distance, predecessors and push into queue. */
						m_distances[nv] = updatedDistance;
						predecessors[nv] = curV.first;
						QueVert vd;
						vd.first = nv;
						vd.second = updatedDistance;
						queue.push(vd);
					}
				}
			}
		}
	}

	m_isUpdated = true;
}

void STVD::resetDistances()
{
	m_distances.resize(m_numVerts);
	m_distances.setConstant(-1);
}

double STVD::getDistance(unsigned int vInd)
{
	if (!m_isUpdated) {
		std::cout << "Attempt to get out of date distances!" << std::endl;
		return -1;
	}
	if (vInd >= m_numVerts) {
		std::cout << "Attempt to get distance of illegal vertex!" << std::endl;
		return -1;
	}
	return m_distances(vInd);
}

PDVector& STVD::getDistances()
{
	if (!m_isUpdated) {
		std::cout << "Attempt to get out of date distances!" << std::endl;
	}
	return m_distances;
}

void STVD::setK(unsigned int k)
{
	m_k = k;
}

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

/* Returns the signed angle between v1 and v2 with respect to a normal n, i.e.
   the positive angle phi between them if v2 has to be rotated counterclockwise by phi to get to v1
   and the negative angle phi, if v2 has to be rotated clockwise by |phi| to get to v1*/
PDScalar getSignedAngle(PD3dVector v1, PD3dVector v2, PD3dVector n) {
	return std::acos(v1.dot(v2) / (v1.norm() * v2.norm())) * sgn(n.cross(v1).dot(v2));
}

double STVD::updateVertDist(unsigned int v1, unsigned int v2, std::vector<int>& predecessors)
{
	if (m_mode == STVD_MODE_GRAPH_DIJKSTRA) {
		return m_distances(v1) + (m_positions.row(v1) - m_positions.row(v2)).norm();
	}
	else if (m_mode == STVD_MODE_ST_DIJKSTRA) {
		// Implementation of the STVD update_dist function from [Campen et al. 2013]
	
		// This is the previous edge that was added to the sum, still in 3d
		PD3dVector prevEdge = (m_positions.row(v2) - m_positions.row(v1));

		// Store actual predecessor temporarily and assume v1 would be predecessor
		// of v2
		int tmpPred = predecessors[v2];
		predecessors[v2] = v1;

		// The "front-most" vertex in the edge chain
		int prevV = v2;
		int curV = v1;

		PD3dVector e_sum3d = prevEdge;
		PDScalar eLength = e_sum3d.norm();

		// The following is only used for surface meshes
		// This will be the sum of the unfolded 2d vectors
		PD2dVector e_sum2d;
		e_sum2d.setZero();
		e_sum2d(1) = eLength;
		// The angle between the last edge that has been unfolded and the x-axis
		PDScalar curAngle = 0;

		// The currently best distance is the standard dijkstra update
		double bestDist = m_distances(v1) + eLength;

		// Now we check if unfolded edge sums can enhance this distance
		for (int i = 2; i <= m_k; i++) {
			if (predecessors[curV] < 0) break;
			
			// Get nexte edge in the chain
			int nextV = predecessors[curV];
			PD3dVector nextEdge = (m_positions.row(curV) - m_positions.row(nextV));

			PDScalar curDist = -1;
			// If both edges run along the surface, unfold them into the common plane
			// defined by the vertex normal
			if (!m_forTets) {
				PD3dVector n = m_outerVertNormals.row(curV);
				PD3dVector nextEdgeFlat = nextEdge - n * (n.dot(nextEdge));
				PD3dVector prevEdgeFlat = prevEdge - n * (n.dot(prevEdge));
				PDScalar angle = getSignedAngle(prevEdgeFlat, nextEdgeFlat, n);
				curAngle += angle;
				PDScalar l = nextEdgeFlat.norm();
				PD2dVector nextEdge2d(std::sin(curAngle) * l, std::cos(curAngle) * l);
				e_sum2d += nextEdge2d;
				curDist = m_distances[nextV] + e_sum2d.norm();
			}
			else {
				e_sum3d += nextEdge;
				curDist = m_distances[nextV] + e_sum3d.norm();
			}
			if (curDist >= 0 && curDist < bestDist) bestDist = curDist;

			prevV = curV;
			curV = nextV;
			prevEdge = nextEdge;
		}

		return bestDist;
	}
	return m_distances(v1) + (m_positions.row(v1) - m_positions.row(v2)).norm();
}
