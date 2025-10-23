/*
 * Copyright (C) 2022 - 2025 Sean Apeler
 * This file is part of fastgltf <https://github.com/spnda/fastgltf>.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
 // This code was copied from fastgltf/examples/gl_viewer.cc

#include "additional_math.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

// It's simpler here to just declare the functions as part of the fastgltf::math namespace.
namespace fastgltf::math {
	/** Creates a right-handed view matrix */
	[[nodiscard]] mat<float, 4, 4> lookAtRH(const fvec3& eye, const fvec3& center, const fvec3& up) noexcept {
		auto dir = normalize(center - eye);
		auto lft = normalize(cross(dir, up));
		auto rup = cross(lft, dir);

		mat<float, 4, 4> ret(1.f);
		ret.col(0) = { lft.x(), rup.x(), -dir.x(), 0.f };
		ret.col(1) = { lft.y(), rup.y(), -dir.y(), 0.f };
		ret.col(2) = { lft.z(), rup.z(), -dir.z(), 0.f };
		ret.col(3) = { -dot(lft, eye), -dot(rup, eye), dot(dir, eye), 1.f };
		return ret;
	}

	/**
	 * Creates a right-handed perspective matrix, with the near and far clips at -1 and +1, respectively.
	 * @param fov The FOV in radians
	 */
	[[nodiscard]] mat<float, 4, 4> perspectiveRH(float fov, float ratio, float zNear, float zFar) noexcept {
		mat<float, 4, 4> ret(0.f);
		auto tanHalfFov = std::tanf(fov / 2.f);
		ret.col(0).x() = 1.f / (ratio * tanHalfFov);
		ret.col(1).y() = 1.f / tanHalfFov;
		ret.col(2).z() = -(zFar + zNear) / (zFar - zNear);
		ret.col(2).w() = -1.f;
		ret.col(3).z() = -(2.f * zFar * zNear) / (zFar - zNear);
		return ret;
	}

	fvec3 min(const fvec3& x, const fvec3& y) noexcept {
		return fvec3(
			x[0] < y[0] ? x[0] : y[0],
			x[1] < y[1] ? x[1] : y[1],
			x[2] < y[2] ? x[2] : y[2]
		);
	}

	fvec3 max(const fvec3& x, const fvec3& y) noexcept {
		return fvec3(
			x[0] > y[0] ? x[0] : y[0],
			x[1] > y[1] ? x[1] : y[1],
			x[2] > y[2] ? x[2] : y[2]
		);
	}
} // namespace fastgltf::math