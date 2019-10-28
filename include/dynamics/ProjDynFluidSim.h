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
#include <vector>

namespace FluidSim {
	typedef float fluid_scalar;
	typedef int fluid_int;
	typedef long long unsigned int fluid_index;
	typedef std::vector<float> fluid_vec;

	struct fluid_vec3 {
		fluid_scalar x, y, z;

		fluid_vec3() { x = 0; y = 0; z = 0; }

		fluid_vec3(fluid_scalar xx, fluid_scalar yy, fluid_scalar zz) {
			x = xx;
			y = yy;
			z = zz;
		}


		fluid_vec3 operator+(fluid_vec3 a) {
			fluid_vec3 b(a.x + x, a.y + y, a.z + z);
			return b;
		};

		fluid_vec3 operator*(fluid_vec3 a) {
			fluid_vec3 b(a.x * x, a.y * y, a.z * z);
			return b;
		};

		fluid_vec3 operator/(fluid_vec3 a) {
			fluid_vec3 b(a.x / x, a.y / y, a.z / z);
			return b;
		};

	};

	struct fluid_ivec3 {
		fluid_int x, y, z;

		fluid_ivec3() { x = 0; y = 0; z = 0; }

		fluid_ivec3(fluid_int xx, fluid_int yy, fluid_int zz) {
			x = xx;
			y = yy;
			z = zz;
		}

		fluid_ivec3 operator+(fluid_ivec3 a) {
			fluid_ivec3 b(a.x + x, a.y + y, a.z + z);
			return b;
		}

		fluid_ivec3 operator*(fluid_ivec3 a) {
			fluid_ivec3 b(a.x * x, a.y * y, a.z * z);
			return b;
		}

		fluid_ivec3 operator/(fluid_ivec3 a) {
			fluid_ivec3 b(a.x / x, a.y / y, a.z / z);
			return b;
		}
	};

	class FluidGrid {
	protected:
		/* number of cells in x, y, z directions */
		fluid_ivec3 m_gridSize;
		/* bottom, left, back corner of the first fluid cell */
		fluid_vec3 m_gridBase;
		/* side length of a cell */
		fluid_scalar m_cellLength;
	public:
		FluidGrid(fluid_ivec3 gridSize, fluid_vec3 gridBase, fluid_scalar cellLength) {
			m_gridSize = gridSize;
			m_gridBase = gridBase;
			m_cellLength = cellLength;
		};

		fluid_scalar getCellLength() {
			return m_cellLength;
		}

		fluid_int getSize() {
			return m_gridSize.x * m_gridSize.y * m_gridSize.z;
		}

		fluid_int getMACSize() {
			return (m_gridSize.x + 1) * (m_gridSize.y + 1) * (m_gridSize.z + 1);
		}
	};
}