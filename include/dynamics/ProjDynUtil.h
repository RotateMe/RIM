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
#include "Eigen/SparseCholesky"

#include "ProjDynTypeDef.h"

namespace PD {

	struct Edge;

	class ProjDynConstraint;

	void writeCSVFromMeasurements(const std::vector<PD::Measurement>& measures, std::string fileName, int numIterations = 1);
	void writeCSVFromSubApproxErrors(std::vector<PD::PDScalar>& errors, std::string meshURL);
	void writeCSVFromDetLog(std::vector<std::pair<unsigned int, double>>& detLog, std::string meshURL);

	PD3dVector getTriangleNormal(Eigen::Vector3i& triangle, PDPositions& positions);
	PD3dVector getTriangleNormal(PDTriangles& triangle, PDPositions& positions, bool dontNormalize = false);

	PDPositions getVertexNormals(PDTriangles& triangle, PDPositions& positions, int numOuterVertices);
	void getVertexNormals(PDTriangles& triangle, PDPositions& positions, int numOuterVertices, PDPositions& normalsOut);

	PDScalar triangleArea(PDTriangles tri, PDPositions& positions);
	PDScalar tetArea(PDTets tet, PDPositions& positions);
	double distPointToTriangle(PD3dVector* triangle, PD3dVector point);

	std::vector< std::vector< Edge > > makeVertexStars(int numVertices, int numTriangles, PDTriangles& triangles);
	std::vector< std::vector< unsigned int > > makeTetsPerVertexList(int numVertices, PDTets& tets);
	PDVector vertexMasses(PDTriangles& tris, PDPositions& positions);
	PDVector vertexMasses(PDTets& tets, PDPositions& positions);
	PDSparseMatrix bigTetMassMatrix(PDTets& tets, PDPositions& positions);

	/* Sets a to a + A*b for dense vectors a and b and a sparse matrix A.
	   It's faster than the usual Eigen routines assuming that b is a small vector, such that A*b will be
	   highly sparse. */
	void fastDensePlusSparseTimesDense(PDVector& a, PDSparseMatrix& A, PDVector& b, PDScalar weight = 1);

	void fastDensePlusSparseTimesDenseCol(PDPositions& a, PDSparseMatrix& A, PDPositions& b, unsigned int d, PDScalar weight = 1);

	/* Returns the rotation matrix R which minimizes ||A - RB||^2_F */
	Eigen::Matrix<PDScalar, 3, 3> findBestRotation(Eigen::Matrix<PDScalar, 3, 3>& A, Eigen::Matrix<PDScalar, 3, 3>& B);

	PDScalar clamp(PDScalar, PDScalar, PDScalar);

	class PDSparseVector {
	public:
		unsigned int size, nnz;
		double* values;
		unsigned int* inds;
		PDSparseVector();
		PDSparseVector(unsigned int, unsigned int);
		~PDSparseVector();

		PDSparseVector& operator=(const PDSparseVector& other);
	};

	//PDSparseVector sparseTimesSmall(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& a);
	void densePlusSparse(Eigen::VectorXd& a, PDSparseVector& b);

	PDMatrix getTetWeightsFromVertexWeights(PDMatrix& vertexWeights, PDTets& tets);
	PDMatrix getTriangleWeightsFromVertexWeights(PDMatrix& vertexWeights, PDTriangles& tris);

	PDMatrix createSkinningSpace(PDPositions& restPositions, PDMatrix& weights, std::vector<ProjDynConstraint*>* forConstraints = nullptr, unsigned int linesPerConstraint = 1, PDTriangles* tris = nullptr, PDTets* tets = nullptr, bool flatSpace = false);
	PDMatrix createFullSchurRHSMatrix(std::vector<ProjDynConstraint*>& constraints);

	void storeMesh(PDPositions& positions, PDTriangles& tris, std::string url, const PDPositions& normals = PDPositions(0,3));

	void storeBase(PDMatrix& funcs, std::string url);
	void storeBaseBinary(PDMatrix& funcs, std::string url);
	bool loadBaseBinary(std::string url, PDMatrix& funcs);

	//std::string convertWString(std::wstring s);

	/* Returns the matrix that computes the weighted sum of the auxiliary variables,
	   multiplied from the left by the (square root of the) weights and the S matrices associated to
	   each constraint. */
	PDSparseMatrix getAssemblyMatrix(std::vector<ProjDynConstraint*>& constraints, bool sqrtWeights = false, bool noWeights = false);

	/* Returns the index of the row which has maximum norm */
	int indexOfMaxRowNorm(PDPositions& v);

	/* Returns the index of the row which the largest absolute coefficient */
	int indexOfMaxRowCoeff(PDVector& v);

	/* Returns the index of the row which the largest absolute coefficient in the j-th column of the matrix A */
	int indexOfMaxRowCoeff(PDMatrix& A, unsigned int j);

	/* Returns the index of the column which the largest absolute coefficient in the i-th row of the matrix A */
	int indexOfMaxColCoeff(PDMatrix & A, unsigned int i);

	/* Simply turns a full path like "D:\files\mesh.obj" into something like "D:\files\mesh_base.bin" */
	std::string getMeshFileName(std::string meshURL, std::string suffix);

	/* Fills however many samples are missing in the list "samples" with random number between 0 and max, that
	   are not yet in the list, until "samples" contains numSamples numbers.*/
	void fillSamplesRandomly(std::vector<unsigned int>& samples, unsigned int numSamples, unsigned int max);

}