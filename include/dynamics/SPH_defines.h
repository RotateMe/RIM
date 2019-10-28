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

#define PI_D 3.1415926535897932384626433832795028841971693993751058209749445923078164062

#define SPH_NUM_THREADS 8
#define SPH_PARALLEL_FOR __pragma(omp parallel for num_threads(SPH_NUM_THREADS)) 

// Number of maximal neighbours of a particle. If more particles reside in a particle's kernel support sphere, a "random" selection will be ignored.
#define SPH_NUM_MAX_NEIGHBOURS 70
// Maximal value of allowed pressure per constraint projection
#define SPH_PROJECTION_EPS1 1e-11
// If the change in the constraint is below this value,
// we consider it converged and terminate the constraint prohection
#define SPH_PROJECTION_EPS2 1e-16
// Sparsity cutoff epsilon (i.e. when an entry is disregarded in a sparse matrix)
#define SPH_SPARSITY_EPS 1e-14
// Minimal rest density, to prevent its inverse from being NaN
#define SPH_MINIMAL_REST_DENSITY 1e-11
// Maximum number of iterations in the gradient descent steps for the pressure projcection constraints
#define SPH_PROJECTION_MAX_ITER 10
// Use this define if you want to use the original pressure projections. 
// Assumes that average particle spacing is exactly one half of the kernel support size.
//#define SPH_SIMPLE_PROJECTION
// Maximum number of boundary particles
#define SPH_NUM_MAX_BD_NEIGHBOURS 25
// This defines whether boundaries are enforced in a hard manner
#define SPH_HARD_BOUNDS true