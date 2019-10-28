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

#include "cuda_runtime.h"
#include "cublas_v2.h"
#include <iostream>

// Define this to turn on error checking
//#define CUDA_ERROR_CHECK
// Define this to turn safety syncing
//#define CUDA_SAFE_SYNC

#define CudaSafeCall( err ) __cudaSafeCall( err, __FILE__, __LINE__ )
#define CudaCheckError()    __cudaCheckError( __FILE__, __LINE__ )
#define CublasSafeCall(err) __cublasSafeCall(err, __FILE__, __LINE__);

inline void __cublasSafeCall(cublasStatus_t err, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
	if (err != cublasStatus_t::CUBLAS_STATUS_SUCCESS)
	{
		fprintf(stderr, "cublasSafeCall() failed at %s:%i \n",
			file, line);
		system("pause");
		exit(-1);
	}
#endif

	return;
}

inline void __cudaSafeCall(cudaError err, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
	if (cudaSuccess != err)
	{
		fprintf(stderr, "cudaSafeCall() failed at %s:%i : %s\n",
			file, line, cudaGetErrorString(err));
		system("pause");
		exit(-1);
	}
#endif

	return;
}

inline void __cudaCheckError(const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
	cudaError err = cudaGetLastError();
	if (cudaSuccess != err)
	{
		fprintf(stderr, "cudaCheckError() failed at %s:%i : %s\n",
			file, line, cudaGetErrorString(err));
		system("pause");
		exit(-1);
	}

	// More careful checking. However, this will affect performance.
	// Comment away if needed.
	err = cudaDeviceSynchronize();
	if (cudaSuccess != err)
	{
		fprintf(stderr, "cudaCheckError() with sync failed at %s:%i : %s\n",
			file, line, cudaGetErrorString(err));
		system("pause");
		exit(-1);
	}
#endif

	return;
}


#define CudaSafeSync() __cudaSafeSync(__FILE__, __LINE__)

inline void __cudaSafeSync(const char *file, const int line)
{
#ifdef CUDA_SAFE_SYNC
	cudaError_t err = cudaDeviceSynchronize();
	if (err != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize() failed at %s:%i : %s\n",
			file, line, cudaGetErrorString(err));
		system("pause");
		exit(-1);
	}
#endif
	return;
}
