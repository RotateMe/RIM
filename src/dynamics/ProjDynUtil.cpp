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
#include <fstream>

#include "dynamics/ProjDynUtil.h"
#include "dynamics/ProjDynSimulator.h"
#include "dynamics/ProjDynConstraints.h"

namespace PD {

	PDVector vertexMasses(PDTriangles& tris, PDPositions& positions) {
		PDVector vMasses(positions.rows());
		vMasses.fill(0);
		int numTris = tris.rows();
		for (int tInd = 0; tInd < numTris; tInd++) {
			PDScalar curArea = triangleArea(tris.row(tInd), positions) * (1. / 3.);

			vMasses(tris(tInd, 0), 0) += curArea;
			vMasses(tris(tInd, 1), 0) += curArea;
			vMasses(tris(tInd, 2), 0) += curArea;
		}

		for (int vInd = 0; vInd < vMasses.rows(); vInd++) {
			if (vMasses(vInd, 0) < PROJ_DYN_MIN_MASS) {
				vMasses(vInd, 0) = PROJ_DYN_MIN_MASS;
			}
		}

		return vMasses;
	}

	PDVector vertexMasses(PDTets& tets, PDPositions& positions) {
		PDVector vMasses(positions.rows());
		vMasses.fill(0);
		int numTets = tets.rows();
		for (int tInd = 0; tInd < numTets; tInd++) {
			PDScalar curArea = tetArea(tets.row(tInd), positions) * (1. / 4.);

			vMasses(tets(tInd, 0), 0) += curArea;
			vMasses(tets(tInd, 1), 0) += curArea;
			vMasses(tets(tInd, 2), 0) += curArea;
			vMasses(tets(tInd, 3), 0) += curArea;
		}

		for (int vInd = 0; vInd < vMasses.rows(); vInd++) {
			if (vMasses(vInd, 0) < PROJ_DYN_MIN_MASS) {
				vMasses(vInd, 0) = PROJ_DYN_MIN_MASS;
			}
		}

		return vMasses;
	}

	PDSparseMatrix bigTetMassMatrix(PDTets& tets, PDPositions& positions) {
		PDSparseMatrix bigTetM(tets.rows() * 3, tets.rows() * 3);
		int numTets = tets.rows();
		std::vector<Eigen::Triplet<PDScalar>> massEntries;
		massEntries.reserve(tets.rows() * 3);

		for (int tInd = 0; tInd < numTets; tInd++) {
			PDScalar curArea = tetArea(tets.row(tInd), positions);
			// libigl gradient maps order the gradient vector as (x1, x2, ..., xn, y1, ..., yn, z1, ..., zn)
			massEntries.push_back(Eigen::Triplet<PDScalar>(tInd + (numTets * 0), tInd + (numTets * 0), curArea));
			massEntries.push_back(Eigen::Triplet<PDScalar>(tInd + (numTets * 1), tInd + (numTets * 1), curArea));
			massEntries.push_back(Eigen::Triplet<PDScalar>(tInd + (numTets * 2), tInd + (numTets * 2), curArea));
		}

		bigTetM.setFromTriplets(massEntries.begin(), massEntries.end());

		return bigTetM;
	}

	PDScalar triangleArea(PDTriangles tri, PDPositions& positions) {
		PDScalar l1 = (positions.row(tri(0, 0)) - positions.row(tri(0, 1))).norm();
		PDScalar l2 = (positions.row(tri(0, 1)) - positions.row(tri(0, 2))).norm();
		PDScalar l3 = (positions.row(tri(0, 2)) - positions.row(tri(0, 0))).norm();
		PDScalar p = (1.f / 2.f) * (l1 + l2 + l3);
		return std::sqrt(p * (p - l1) * (p - l2) * (p - l3));
	}

	PDScalar triangleArea(Eigen::Vector3i tri, PDPositions& positions) {
		PDScalar l1 = (positions.row(tri(0)) - positions.row(tri(1))).norm();
		PDScalar l2 = (positions.row(tri(1)) - positions.row(tri(2))).norm();
		PDScalar l3 = (positions.row(tri(2)) - positions.row(tri(0))).norm();
		PDScalar p = (1.f / 2.f) * (l1 + l2 + l3);
		return std::sqrt(p * (p - l1) * (p - l2) * (p - l3));
	}

	PDScalar tetArea(PDTets tet, PDPositions& positions) {
		// 3d edges of tet
		Eigen::Matrix<PDScalar, 3, 3> edges;
		edges.col(0) = (positions.row(tet(0, 1)) - positions.row(tet(0, 0)));
		edges.col(1) = (positions.row(tet(0, 2)) - positions.row(tet(0, 0)));
		edges.col(2) = (positions.row(tet(0, 3)) - positions.row(tet(0, 0)));

		// Weight gets multiplied by tet volume
		return edges.determinant() / 6.0f;
	}

	void storeBase(PDMatrix& funcs, std::string url) {
		std::ofstream smplFile;
		smplFile.open(url);

		smplFile << funcs.rows() << " " << funcs.cols() << "\n";
		smplFile << funcs << "\n";
		smplFile.close();
	}

	void storeBaseBinary(PDMatrix& funcs, std::string url) {
		std::fstream smplFile(url, std::ios::out | std::ios::binary);
		int rows = funcs.rows(), cols = funcs.cols();
		smplFile.write((char*)&rows, sizeof(int));
		smplFile.write((char*)&cols, sizeof(int));
		smplFile.write((char*)funcs.data(), sizeof(PDScalar)*funcs.rows()*funcs.cols());
		smplFile.close();
	}

	bool loadBaseBinary(std::string url, PDMatrix & funcs)
	{
		std::fstream smplFile(url, std::ios::in | std::ios::binary);
		if (smplFile.is_open()) {
			int rows, cols;
			smplFile.read((char*)&rows, sizeof(int));
			smplFile.read((char*)&cols, sizeof(int));
			funcs.resize(rows, cols);
			smplFile.read((char*)funcs.data(), sizeof(PDScalar)*rows*cols);
			smplFile.close();
		}
		else {
			funcs.resize(0, 0);
			std::cout << "Could not open file " << url << std::endl;
			return false;
		}
		return true;
	}

	PDSparseMatrix getAssemblyMatrix(std::vector<ProjDynConstraint*>& constraints, bool sqrtWeights, bool noWeights)
	{
		int numVertices = constraints[0]->getSelectionMatrix().cols();
		int constraintSize = constraints[0]->getSelectionMatrix().rows();
		PDSparseMatrix S(numVertices, constraintSize * constraints.size());

		std::vector< Eigen::Triplet<double> > tripletListAssembly;
		tripletListAssembly.reserve(constraints.size() * 4);
		for (unsigned int i = 0; i < constraints.size(); i++) {
			PDSparseMatrix& selMat = constraints.at(i)->getSelectionMatrixTransposed();
			PDScalar weight = 1.;
			if (!noWeights) {
				weight = constraints.at(i)->getWeight();
				if (sqrtWeights) weight = std::sqrt(weight);
				if (weight < 1e-10) {
					std::cout << "Shmall weight!" << std::endl;
				}
			}
			for (int k = 0; k < selMat.outerSize(); ++k)
				for (PDSparseMatrix::InnerIterator it(selMat, k); it; ++it)
				{
					tripletListAssembly.push_back(Eigen::Triplet<PDScalar>(it.row(), i * constraintSize + it.col(), it.value() * weight));
				}
		}
		S.setFromTriplets(tripletListAssembly.begin(), tripletListAssembly.end());

		return S;
	}

	int indexOfMaxRowNorm(PDPositions & v)
	{
		int j = 0;
		double max = 0;
		for (int i = 0; i < v.rows(); i++) {
			double curNorm = v.row(i).norm();
			if (curNorm > max) {
				max = curNorm;
				j = i;
			}
		}
		return j;
	}

	int indexOfMaxRowCoeff(PDVector & v)
	{
		int j = 0;
		double max = 0;
		for (int i = 0; i < v.rows(); i++) {
			double curNorm = std::abs(v(i));
			if (curNorm > max) {
				max = curNorm;
				j = i;
			}
		}
		return j;
	}

	int indexOfMaxRowCoeff(PDMatrix & A, unsigned int col)
	{
		int j = 0;
		double max = 0;
		for (int i = 0; i < A.rows(); i++) {
			double curNorm = std::abs(A(i, col));
			if (curNorm > max) {
				max = curNorm;
				j = i;
			}
		}
		return j;
	}

	int indexOfMaxColCoeff(PDMatrix & A, unsigned int row)
	{
		int j = 0;
		double max = 0;
		for (int i = 0; i < A.cols(); i++) {
			double curNorm = std::abs(A(row, i));
			if (curNorm > max) {
				max = curNorm;
				j = i;
			}
		}
		return j;
	}

	std::string getMeshFileName(std::string meshURL, std::string suffix)
	{
		std::string out = meshURL;
		int dotPos = meshURL.find_last_of(".");
		out.replace(dotPos, meshURL.length() - dotPos, suffix);
		return out;
	}

	void fillSamplesRandomly(std::vector<unsigned int>& samples, unsigned int numSamples, unsigned int max)
	{
		if (numSamples <= samples.size()) return;
		do {
			for (unsigned int i = 0; i < numSamples - samples.size(); i++) {
				samples.push_back(rand() % static_cast<int>(max + 1));
			}
			std::sort(samples.begin(), samples.end());
			samples.erase(std::unique(samples.begin(), samples.end()), samples.end());
		} while (samples.size() < numSamples && samples.size() != max);
	}

	void storeMesh(PDPositions& positions, PDTriangles& tris, std::string url, const PDPositions& normals) {
		std::ofstream objFile;
		objFile.open(url);
		for (int v = 0; v < positions.rows(); v++) {
			objFile << "v " << positions(v, 0) << " " << positions(v, 1) << " " << positions(v, 2) << "\n";
		}
		for (int v = 0; v < normals.rows(); v++) {
			objFile << "vn " << normals(v, 0) << " " << normals(v, 1) << " " << normals(v, 2) << "\n";
		}
		/*
		objFile << "\n";
		for (int v = 0; v < positions.rows(); v++) {
			objFile << "vn " << positions(v, 0) << " " << positions(v, 1) << " " << positions(v, 2) << "\n";
		}
		*/
		objFile << "\n";
		for (int f = 0; f < tris.rows(); f++) {
			//objFile << "f  " << (tris(f, 0) + 1) << "//" << (tris(f, 0) + 1) << " " << (tris(f, 1) + 1) << "//" << (tris(f, 1) + 1) << " " << (tris(f, 2) + 1) << "//" << (tris(f, 2) + 1) << "\n";
			objFile << "f  " << (tris(f, 0) + 1) << " " << (tris(f, 1) + 1) << " " << (tris(f, 2) + 1) << "\n";
		}
		objFile.close();
	}

	std::vector< std::vector< Edge > > makeVertexStars(int numVertices, int numTriangles, PDTriangles& triangles) {
		std::vector< std::vector< Edge > > vertexStars;
		vertexStars.resize(numVertices);

		for (int t = 0; t < numTriangles; t++) {
			for (int v = 0; v < 3; v++) {
				unsigned int vInd = triangles(t, v);
				for (int ov = 0; ov < 3; ov++) {
					if (v == ov) continue;
					unsigned int nbVInd = triangles(t, ov);
					bool found = false;
					for (Edge& checkEdge : vertexStars.at(vInd)) {
						if (nbVInd == checkEdge.v2) {
							checkEdge.t2 = t;
							checkEdge.vOtherT2 = triangles(t, 3 - (v + ov));
							found = true;
						}
					}
					if (!found) {
						Edge e;
						e.v1 = vInd;
						e.v2 = nbVInd;
						e.t1 = t;
						e.vOtherT1 = triangles(t, 3 - (v + ov));
						e.t2 = -1;
						e.vOtherT2 = -1;
						vertexStars.at(vInd).push_back(e);
					}
				}
			}
		}

		return vertexStars;
	}

	std::vector< std::vector< unsigned int > > makeTetsPerVertexList(int numVertices, PDTets& tets) {
		std::vector< std::vector< unsigned int > >  tetsPerVertex;
		tetsPerVertex.resize(numVertices);

		for (int t = 0; t < tets.rows(); t++) {
			for (int v = 0; v < 4; v++) {
				unsigned int vInd = tets(t, v);
				if (std::find(tetsPerVertex[vInd].begin(), tetsPerVertex[vInd].end(), t) == tetsPerVertex[vInd].end()) {
					tetsPerVertex[vInd].push_back(t);
				}
			}
		}

		return tetsPerVertex;
	}

	void writeCSVFromSubApproxErrors(std::vector<PD::PDScalar>& errors, std::string meshURL)
	{
		std::ofstream myfile;
		myfile.open(PD::getMeshFileName(meshURL, "_subPosApproxErrors.csv"));
		myfile << "Frame #,Rel. Approx Error\n";
		for (int frame = 0; frame < errors.size(); frame++) {
			myfile << frame << "," << errors[frame] << "\n";
		}
		myfile.close();
	}

	void writeCSVFromMeasurements(const std::vector<PD::Measurement>& measures, std::string meshURL, int numIterations)
	{
		std::ofstream myfile;
		myfile.open(PD::getMeshFileName(meshURL, "_measures.csv"));
		myfile << "Frame #,t Local Step,t Global Step,t Partial Vertex Update,"
			<< "t Full Vertex Update,FPS excl. update,FPS incl. update,Pos. Subspace Approx. Rel. Error,Projection Interpolation Rel. Error,Projection vs. Interpolation Rel. Error, \n";
		for (auto& m : measures) {
			double fps = 1000000. / (m.localTime * numIterations + m.globalTime * numIterations + m.partialUpdateTime * numIterations);
			double fpsU = 1000000. / (m.localTime * numIterations + m.globalTime * numIterations + m.partialUpdateTime * numIterations + m.fullUpdateTime);
			double approxError = -1;
			if (!m.subspaceApproxErrors.empty()) approxError = m.subspaceApproxErrors.back();
			myfile << m.frame << "," << m.localTime * numIterations << "," << m.globalTime * numIterations << ","
				<< m.partialUpdateTime * numIterations << "," << m.fullUpdateTime << "," << fps << "," << fpsU << ","
				<< approxError << "," << m.interpolError << "," << m.interpolProjectionError;
			myfile << "\n";
		}
		myfile.close();

		myfile.open(PD::getMeshFileName(meshURL, "_energies.csv"));
		myfile << "Iteration #,Time (ms)";
		for (int i = 0; i < measures.size(); i++) {
			myfile << ",Frame" << i;
		}
		myfile << "\n";
		int timePerIt = measures[0].localTime + measures[0].globalTime + measures[0].partialUpdateTime;
		for (int i = 0; i < numIterations + 1; i++) {
			myfile << i << ",";
			myfile << (i * timePerIt)/(1000);
			for (auto& m : measures) {
				myfile << ",";
				if (m.energyLevels.size() > i) {
					myfile << m.energyLevels[i];
				}
				else {
					myfile << "n/a";
				}
			}
			myfile << "\n";
		}
		myfile.close();
	}

	void writeCSVFromDetLog(std::vector<std::pair<unsigned int, double>>& detLog, std::string meshURL)
	{
		std::ofstream myfile;
		myfile.open(PD::getMeshFileName(meshURL, "_detLog.csv"));
		myfile << "Frame #, avg. det, max. deviation\n";
		unsigned int curFrame = 0;
		double curMaxDev = 0;
		double curAvg = 0;
		unsigned int curCount = 0;
		for (auto& d : detLog) {
			if (d.first != curFrame) {
				curAvg /= (double)curCount;
				myfile << curFrame << "," << curAvg << "," << curMaxDev << "\n";
				curAvg = 0;
				curCount = 0;
				curMaxDev = 0;
				curFrame = d.first;
			}

			curAvg += d.second;
			if (std::abs(1 - d.second) > curMaxDev) {
				curMaxDev = std::abs(1 - d.second);
			}
			curCount++;
		}
		myfile.close();
	}

	PD3dVector getTriangleNormal(Eigen::Vector3i& triangle, PDPositions& positions) {
		PD3dVector normal = (positions.row(triangle(1)) - positions.row(triangle(0))).cross(positions.row(triangle(2)) - positions.row(triangle(0)));
		normal.normalize();
		return normal;
	}

	PD3dVector getTriangleNormal(PDTriangles& triangles, PDPositions& positions, bool dontNormalize)
	{
		PD3dVector normal;
		normal.setZero();
		for (int i = 0; i < triangles.rows(); i++) {
			normal += (positions.row(triangles(i, 1)) - positions.row(triangles(i, 0))).cross(positions.row(triangles(i, 2)) - positions.row(triangles(i, 0))).normalized();// *triangleArea(triangles.row(i), positions);
		}
		if (!dontNormalize) normal.normalize();
		return normal;
	}

	PDPositions getVertexNormals(PDTriangles & triangles, PDPositions & positions, int numOuterVertices)
	{
		int numVerts = numOuterVertices;
		if (numVerts <= 0) numVerts = positions.rows();

		PDPositions vertNormals(numVerts, 3);
		vertNormals.setZero();

		for (int t = 0; t < triangles.rows(); t++) {
			Eigen::Vector3i tri = triangles.row(t);
			PD3dVector tn = getTriangleNormal(tri, positions);
			PDScalar area = triangleArea(tri, positions);
			for (int v = 0; v < 3; v++) {
				vertNormals.row(tri(v)) += tn * area;
			}
		}

		for (int v = 0; v < numVerts; v++) {
			vertNormals.row(v).normalize();
		}

		return vertNormals;
	}

	// HACK! returns the smallest of the distances to the corners and the middle,
	// but not the actual smallest distance
	double distPointToTriangle(PD3dVector* triangle, PD3dVector point) {
		double dist = std::numeric_limits<double>::max();
		PD3dVector mid(0., 0., 0.);
		for (int j = 0; j < 3; j++) {
			double curDist = (triangle[j] - point).norm();
			mid += triangle[j];
			if (curDist < dist) dist = curDist;
		}
		mid /= 3;
		double midDist = (mid - point).norm();
		if (midDist < dist) dist = midDist;
		return dist;
	}

	void getVertexNormals(PDTriangles & triangles, PDPositions & positions, int numOuterVertices, PDPositions & normalsOut)
	{

		int numVerts = numOuterVertices;
		if (numVerts <= 0) numVerts = positions.rows();

		normalsOut.setZero();

		for (int t = 0; t < triangles.rows(); t++) {
			Eigen::Vector3i tri = triangles.row(t);
			PD3dVector tn = getTriangleNormal(tri, positions);
			PDScalar area = triangleArea(tri, positions);
			for (int v = 0; v < 3; v++) {
				if (tri(v) < numVerts) {
					normalsOut.row(tri(v)) += tn * area;
				}
			}
		}

		for (int v = 0; v < numVerts; v++) {
			normalsOut.row(v).normalize();
		}
	}

	void fastDensePlusSparseTimesDense(PDVector& a, PDSparseMatrix& A, PDVector& b, PDScalar weight)
	{
		for (int k = 0; k < A.outerSize(); ++k)
			for (PDSparseMatrix::InnerIterator it(A, k); it; ++it)
			{
				a(it.row()) += it.value() * b(it.col()) * weight;
			}
	}

	void fastDensePlusSparseTimesDenseCol(PDPositions& a, PDSparseMatrix& A, PDPositions& b, unsigned int d, PDScalar weight)
	{
		for (int k = 0; k < A.outerSize(); ++k)
			for (PDSparseMatrix::InnerIterator it(A, k); it; ++it)
			{
				a(it.row(), d) += it.value() * b(it.col(), d) * weight;
			}
	}

	Eigen::Matrix<PDScalar, 3, 3> reflectionRemover;
	bool reflectionRemoverInitialized = false;

	Eigen::Matrix<PDScalar, 3, 3> findBestRotation(Eigen::Matrix<PDScalar, 3, 3>& A, Eigen::Matrix<PDScalar, 3, 3>& B)
	{
		if (!reflectionRemoverInitialized) {
			reflectionRemover.setZero(3, 3);
			reflectionRemover(0, 0) = 1.;
			reflectionRemover(1, 1) = 1.;
			reflectionRemover(2, 2) = -1.;
		}

		Eigen::Matrix<PDScalar, 3, 3> R = B * A.transpose();
		Eigen::JacobiSVD<Eigen::Matrix<PDScalar, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
		if (svd.matrixV().determinant() * svd.matrixU().determinant() < 0.) {
			R = svd.matrixV() * reflectionRemover * svd.matrixU().transpose();
		}
		else {
			R = svd.matrixV() * svd.matrixU().transpose();
		}

		return R;
	}

	PDScalar clamp(PDScalar a, PDScalar min, PDScalar max) {
		return std::max(min, std::min(max, a));
	}

	void densePlusSparse(Eigen::VectorXd& a, PDSparseVector& b) {
		for (int i = 0; i < b.nnz; i++) {
			a(b.inds[i]) += b.values[i];
		}
	}

	PDMatrix getTetWeightsFromVertexWeights(PDMatrix& vertexWeights, PDTets& tets) {
		PDMatrix tetWeights(tets.rows(), vertexWeights.cols());
		PROJ_DYN_PARALLEL_FOR
			for (int t = 0; t < tets.rows(); t++) {
				for (unsigned int c = 0; c < vertexWeights.cols(); c++) {
					PDScalar curWeight = 0.;
					for (unsigned int d = 0; d < 4; d++) {
						curWeight += vertexWeights(tets(t, d), c);
					}
					curWeight /= 4.;
					tetWeights(t, c) = curWeight;
				}
				tetWeights.row(t) /= tetWeights.row(t).sum();
			}
		return tetWeights;
	}

	PDMatrix getTriangleWeightsFromVertexWeights(PDMatrix& vertexWeights, PDTriangles& tris) {
		PDMatrix triWeights(tris.rows(), vertexWeights.cols());
		PROJ_DYN_PARALLEL_FOR
			for (int t = 0; t < tris.rows(); t++) {
				for (unsigned int c = 0; c < vertexWeights.cols(); c++) {
					PDScalar curWeight = 0.;
					for (unsigned int d = 0; d < 3; d++) {
						curWeight += vertexWeights(tris(t, d), c);
					}
					curWeight /= 3.;
					triWeights(t, c) = curWeight;
				}
				triWeights.row(t) /= triWeights.row(t).sum();
			}
		return triWeights;
	}

	PDMatrix createSkinningSpace(PDPositions& restPositions, PDMatrix& weights, std::vector<ProjDynConstraint*>* forConstraints, unsigned int linesPerConstraints, PDTriangles* tris, PDTets* tets, bool flatSpace)
	{
		if (weights.hasNaN() || restPositions.hasNaN()) {
			std::cout << "Warning: weights or rest-state used to create skinning space have NaN values!" << std::endl;
		}

		int numGroups = weights.cols();
		int numRows = restPositions.rows();
		int dim = restPositions.cols() - (flatSpace ? 1 : 0);
		PDMatrix skinningSpace(numRows, numGroups * (dim + 1)); // +1);

		bool error = false;
		PROJ_DYN_PARALLEL_FOR
			for (int v = 0; v < numRows; v++) {
				for (int g = 0; g < numGroups; g++) {
					double curWeight = 0;
					if (forConstraints != nullptr) {
						if (forConstraints->at(v / linesPerConstraints)->getMainVertexIndex() >= 0) {
							curWeight = weights(forConstraints->at(v / linesPerConstraints)->getMainVertexIndex(), g);
						}
						else if (forConstraints->at(v / linesPerConstraints)->getMainTriangleIndex() >= 0) {
							if (tris == nullptr) {
								std::cout << "Error creating skinning space: a constraint was associated to a triangle, but no triangle list was provided!";
								error = true;
							}
							curWeight = weights(forConstraints->at(v / linesPerConstraints)->getMainTriangleIndex(), g);
						}
						else if (forConstraints->at(v / linesPerConstraints)->getMainTetIndex() >= 0) {
							if (tets == nullptr) {
								std::cout << "Error creating skinning space: a constraint was associated to a tet, but no tet list was provided!";
								error = true;
							}
							curWeight = weights(forConstraints->at(v / linesPerConstraints)->getMainTetIndex(), g);
						}
						else {
							std::cout << "Error creating skinning space: a constraint was neither associated to a vertex, nor a triangle nor a tet!";
							error = true;
						}

					}
					else
					{
						curWeight = weights(v, g);
					}
					if (std::isnan(curWeight)) {
						std::cout << "Warning: NaN weight during skinning space construction!" << std::endl;
						error = true;
						break;
					}
					for (int d = 0; d < dim; d++) {
						skinningSpace(v, g * (dim + 1) + d) = restPositions(v, d) * curWeight;
					}
					skinningSpace(v, g*(dim + 1) + dim) = curWeight;
				}
			}

		if (error) {
			return PDMatrix(0, 0);
		}

		// Last row constant 1 for global translations
		// skinningSpace.col(skinningSpace.cols() - 1).setConstant(1.);

		if (skinningSpace.hasNaN()) {
			std::cout << "Warning: NaN entry in constructed skinning space!" << std::endl;
		}

		return skinningSpace;
	}

	PDMatrix createFullSchurRHSMatrix(std::vector<ProjDynConstraint*>& constraints) {
		unsigned int dSize = constraints[0]->getSchurD().rows();
		unsigned int numVerts = constraints[0]->getSchurD().cols();
		PDMatrix fullD(dSize * constraints.size(), numVerts);
		PDMatrix fullA_inv_D_T(numVerts, dSize * constraints.size());
		unsigned int i = 0;
		for (ProjDynConstraint* ci : constraints) {
			PDMatrix& D = ci->getSchurD();
			PDMatrix& A_inv_D_T = ci->getSchurAinvDT();
			for (unsigned j = 0; j < dSize; j++) {
				fullD.row(i*dSize + j) = D.row(j);
				fullA_inv_D_T.col(i*dSize + j) = A_inv_D_T.col(j);
			}
			i++;
		}
		PDMatrix fullSchur = fullD * fullA_inv_D_T;
		return fullSchur;
	}

	PDSparseVector::PDSparseVector()
		: PDSparseVector(0, 0)
	{
	}

	PDSparseVector::PDSparseVector(unsigned int size, unsigned int nnz) :
		size(size),
		nnz(nnz),
		values(new double[nnz]),
		inds(new unsigned int[nnz])
	{
		for (int i = 0; i < nnz; i++) {
			values[i] = 0;
			inds[i] = 0;
		}
	}

	PDSparseVector::~PDSparseVector()
	{
		delete[] values;
		delete[] inds;
	}

	PDSparseVector & PDSparseVector::operator=(const PDSparseVector & other)
	{
		size = other.size;
		nnz = other.nnz;
		delete[] values;
		delete[] inds;
		values = new double[nnz];
		inds = new unsigned int[nnz];
		for (int i = 0; i < nnz; i++) {
			values[i] = other.values[i];
			inds[i] = other.inds[i];
		}

		return *this;
	}

};