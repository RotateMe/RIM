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

// Remove this when including OpenGL headers, it's just there to make sure, the code runs
// without them.
typedef unsigned int GLuint;

namespace PD {

	typedef double PDScalar;
	template <int rows, int cols>
		using PDMatrixF = Eigen::Matrix<PDScalar, rows, cols>;
	typedef Eigen::Matrix<PDScalar, -1, 1> PDVector;
	typedef Eigen::Matrix<PDScalar, 3, 1> PD3dVector;
	typedef Eigen::Matrix<PDScalar, 2, 1> PD2dVector;
	typedef Eigen::Matrix<PDScalar, -1, -1> PDMatrix;
	typedef Eigen::Matrix<PDScalar, -1, 3> PDPositions;
	typedef Eigen::Matrix<float, -1, 3> PDPositionsF;
	typedef Eigen::SparseMatrix<PDScalar, Eigen::ColMajor> PDSparseMatrix;
	typedef Eigen::SparseMatrix<PDScalar, Eigen::RowMajor> PDSparseMatrixRM;
	typedef Eigen::Matrix<int, -1, 3> PDTriangles;
	typedef Eigen::Matrix<int, -1, 4> PDTets;

	struct Measurement {
		unsigned int frame;
		PDScalar interpolError, interpolProjectionError;
		std::vector<PDScalar> subspaceApproxErrors;
		std::vector<PDScalar> energyLevels;
		int localTime, globalTime, partialUpdateTime, fullUpdateTime;
	};

	class ProjDynBoneNode {
	public:
		ProjDynBoneNode(PDPositions& pos, std::string name) {
			m_pos = pos;
			m_name = name;
			if (m_name.find('_') > 0) {
				m_name = m_name.substr(m_name.find('_') + 1);
			}
		}

		void addChild(ProjDynBoneNode* bone) {
			m_children.push_back(bone);
		}

		const PDPositions& getPos() {
			return m_pos;
		}

		std::vector<ProjDynBoneNode*>& getChildren() {
			return m_children;
		}

		const std::string& getName() {
			return m_name;
		}

	private:
		PDPositions m_pos;
		std::string m_name;
		std::vector<ProjDynBoneNode*> m_children;
	};

}