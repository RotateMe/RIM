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

// The mesh sampler currently requires cholmod, which comes with suitesparse
// read _README.txt in the source directory of this Plugin

#pragma once

#include "ProjDynTypeDef.h"
#include "STVD.h"

#define BASE_FUNC_CUTOFF 0.0001
#define USE_HEAT_METHOD false
#define USE_LINEAR false
#define	USE_QUARTIC_POL true

namespace PD {


	class ProjDynMeshSampler {
	public:
		ProjDynMeshSampler();
		ProjDynMeshSampler(PDPositions& pos, PDTriangles& tris, PDTets& tets);
		void init(PDPositions& pos, PDTriangles& tris, PDTets& tets);
		std::vector< unsigned int > getSamples(unsigned int numSamples);
		void extendSamples(unsigned int numTotalSamples, std::vector< unsigned int >& currentSamples);
		void addSource(unsigned int index);
		void computeDistances(PDScalar maxDist = -1, bool accurate = false);
		void clearSources();
		double getDistance(unsigned int index);
		PDMatrix getRadialBaseFunctions(std::vector< unsigned int >& samples, bool partitionOfOne, double r, double eps = -1., int numSmallSamples = -1, double smallSamplesRadius = 1.);
		PDScalar getSampleDiameter(std::vector<unsigned int>& samples);

	private:
		bool m_upToDate;
		int m_nVertices;

		PDVector m_distances;
		STVD m_stvd;

		void addSamples(unsigned int numSamples, std::vector<unsigned int>& samples);
	};

}
