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

#include "dynamics/ProjDynMeshSampler.h"
#include "dynamics/ProjDynutil.h"
#include "dynamics/ProjDynSimulator.h"

PD::ProjDynMeshSampler::ProjDynMeshSampler()
{
}

void PD::ProjDynMeshSampler::init(PDPositions & pos, PDTriangles & tris, PDTets & tets)
{
	m_stvd.init(pos, tris, tets);
	m_nVertices = (pos.rows());
	m_upToDate = false;
}

PD::ProjDynMeshSampler::ProjDynMeshSampler(PDPositions & pos, PDTriangles & tris, PDTets & tets)
{
	init(pos, tris, tets);
}

std::vector<unsigned int> PD::ProjDynMeshSampler::getSamples(unsigned int numSamples)
{
	std::vector<unsigned int> samples;

	clearSources();

	unsigned int firstVert = rand() % m_nVertices;
	samples.push_back(firstVert);
	addSource(firstVert);

	addSamples(numSamples - 1, samples);

	return samples;
}

void PD::ProjDynMeshSampler::addSamples(unsigned int numSamples, std::vector<unsigned int>& samples) {
	if (numSamples <= 0) return;
	PDMatrix heatHistory(m_nVertices, numSamples);
	heatHistory.setZero();
	for (int i = 0; i < numSamples - 1; i++) {
		computeDistances();
		heatHistory.col(i) = m_distances;
		double maxDist = 0;
		unsigned int bestVert = -1;
		for (int v = 0; v < m_nVertices; v++) {
			double curDist = getDistance(v);
			if (maxDist < curDist) {
				bestVert = v;
				maxDist = curDist;
			}
			if (curDist < 0) {
				bestVert = v;
				maxDist = curDist;
				break;
			}
		}
		if (bestVert == -1) {
			std::cout << "Error during sampling, returning with fewer samples... " << std::endl;
			return;
		}
		if (std::find(samples.begin(), samples.end(), bestVert) != samples.end()) {
			std::cout << "Duplicate vertex was selected in sampling, canceling." << std::endl;

			return;
		}
		addSource(bestVert);
		samples.push_back(bestVert);
	}

	//storeBase(heatHistory, "D:\\temp.smp");
}

void PD::ProjDynMeshSampler::extendSamples(unsigned int numTotalSamples, std::vector<unsigned int>& currentSamples)
{
	if (numTotalSamples <= currentSamples.size()) {
		return;
	}

	clearSources();

	for (unsigned int s : currentSamples) {
		addSource(s);
	}

	addSamples(numTotalSamples - currentSamples.size(), currentSamples);
}

void PD::ProjDynMeshSampler::addSource(unsigned int index)
{
	m_stvd.addSource(index);
	m_upToDate = false;
}

void PD::ProjDynMeshSampler::computeDistances(PDScalar maxDist, bool accurate)
{
	if (accurate) {
		m_stvd.computeDistances(true, STVD_MODE_ST_DIJKSTRA, maxDist);
	}
	else {
		m_stvd.computeDistances(true, STVD_MODE_GRAPH_DIJKSTRA, maxDist);
	}
	m_distances = m_stvd.getDistances();

	m_upToDate = true;
}

void PD::ProjDynMeshSampler::clearSources()
{
	m_stvd.resetSources();
	m_stvd.resetDistances();
}

double PD::ProjDynMeshSampler::getDistance(unsigned int index)
{
	if (!m_upToDate) {
		std::cout << "Sources have been added and computeDistances() needs to be called first (or computeDistances() has never been called)!" << std::endl;
		return 0;
	}
	return m_distances(index);
}

PDScalar PD::ProjDynMeshSampler::getSampleDiameter(std::vector<unsigned int>& samples) {
	clearSources();
	for (int v = 0; v < samples.size(); v++) addSource(samples[v]);
	computeDistances();
	double furthestDist = 0;
	for (unsigned int v = 0; v < m_distances.rows(); v++) {
		if (getDistance(v) > furthestDist) {
			furthestDist = getDistance(v);
		}
	}

	return furthestDist;
}

PD::PDMatrix PD::ProjDynMeshSampler::getRadialBaseFunctions(std::vector<unsigned int>& samples, bool partitionOfOne, double r, double eps, int numSmallSamples, double smallSampleRadius)
{
	if (eps < 0) eps = std::sqrt(-std::log(BASE_FUNC_CUTOFF)) / r;

	int nSamples = samples.size();
	PDMatrix baseFunctions;
	baseFunctions.setZero(m_nVertices, nSamples);
	PDScalar a = (1. / std::pow(r, 4.));
	PDScalar b = -2. * (1. / (r * r));

	for (int i = 0; i < nSamples; i++) {
		if (numSmallSamples > 0 && i > nSamples - numSmallSamples) {
			r = smallSampleRadius;
			eps = std::sqrt(-std::log(BASE_FUNC_CUTOFF)) / r;
		}

		unsigned int curSample = samples[i];
		clearSources();
		addSource(curSample);
		computeDistances(r, false);

		for (int v = 0; v < m_nVertices; v++) {
			double curDist = getDistance(v);
			double val = 0;
			if (curDist < 0) {
				val = 0;
			}
			else if (USE_LINEAR) {
				if (curDist >= r) {
					val = 0;
				}
				else {
					val = 1. - (curDist / r);;
				}
			}
			else if (USE_QUARTIC_POL) {
				if (curDist >= r) {
					val = 0;
				}
				else {
					val = a * std::pow(curDist, 4.) + b * (curDist * curDist) + 1;
				}
			}
			else {
				val = std::exp(-(curDist*eps*curDist*eps));
				if (val < BASE_FUNC_CUTOFF) val = 0;
			}
			baseFunctions(v, i) = val;
		}

	}

	if (partitionOfOne) {
		for (int v = 0; v < m_nVertices; v++) {
			PDScalar sum = baseFunctions.row(v).sum();
			if (sum < 1e-6) {
				std::cout << "Warning: a vertex isn't properly covered by any of the radial basis functions!" << std::endl;
				baseFunctions(v, indexOfMaxColCoeff(baseFunctions, v)) = 1.;
			}
			else {
				baseFunctions.row(v) /= sum;
			}
		}
	}

	return baseFunctions;
}

