#include "device_launch_parameters.h"
#include "dynamics/CudaSafeCall.h"

__global__
void doubleToFloatMemCpyKernel(int n, int coord, double* source, float* target) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < n) {
		target[i*3 + coord] = source[i];
	}
}

__global__
void elementWiseMultiplyKernel(int n, double* a, double* b) {
	int i = blockIdx.x*blockDim.x + threadIdx.x;
	if (i < n) {
		a[i] = a[i] * b[i];
	}
}

void doubleToFloatDeviceCpy(int n, int coord, double* source, float* target) {
	doubleToFloatMemCpyKernel<<< (n + 255) / 256, 256 >>>(n, coord, source, target);
	CudaCheckError();
}

void elementWiseMultiply(int n, double * a, double * b)
{
	elementWiseMultiplyKernel<<< (n + 255) / 256, 256 >>> (n, a, b);
}
