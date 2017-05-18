/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
//#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {


	template <int kDataDimension>
	void CubicHermiteSpline(const Eigen::Matrix<double, kDataDimension, 1>& p0,
		const Eigen::Matrix<double, kDataDimension, 1>& p1,
		const Eigen::Matrix<double, kDataDimension, 1>& p2,
		const Eigen::Matrix<double, kDataDimension, 1>& p3,
		const double x,
		double* f,
		double* dfdx) {
		typedef Eigen::Matrix<double, kDataDimension, 1> VType;
		const VType a = 0.5 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3);
		const VType b = 0.5 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3);
		const VType c = 0.5 * (-p0 + p2);
		const VType d = p1;

		// Use Horner's rule to evaluate the function value and its
		// derivative.

		// f = ax^3 + bx^2 + cx + d
		if (f != NULL) {
			Eigen::Map<VType>(f, kDataDimension) = d + x * (c + x * (b + x * a));
		}

		// dfdx = 3ax^2 + 2bx + c
		if (dfdx != NULL) {
			Eigen::Map<VType>(dfdx, kDataDimension) = c + x * (2.0 * b + 3.0 * a * x);
		}
	}


	template<typename Grid>
	class  BiCubicInterpolator {
	public:
		explicit BiCubicInterpolator(const Grid& grid)
			: grid_(grid) {
			// The + casts the enum into an int before doing the
			// comparison. It is needed to prevent
			// "-Wunnamed-type-template-args" related errors.
			CHECK_GE(+Grid::DATA_DIMENSION, 1);
		}

		// Evaluate the interpolated function value and/or its
		// derivative. Returns false if r or c is out of bounds.
		void Evaluate(double r, double c,
			double* f, double* dfdr, double* dfdc) const {
			// BiCubic interpolation requires 16 values around the point being
			// evaluated.  We will use pij, to indicate the elements of the
			// 4x4 grid of values.
			//
			//          col
			//      p00 p01 p02 p03
			// row  p10 p11 p12 p13
			//      p20 p21 p22 p23
			//      p30 p31 p32 p33
			//
			// The point (r,c) being evaluated is assumed to lie in the square
			// defined by p11, p12, p22 and p21.

			const int row = std::floor(r);
			const int col = std::floor(c);

			Eigen::Matrix<double, Grid::DATA_DIMENSION, 1> p0, p1, p2, p3;

			// Interpolate along each of the four rows, evaluating the function
			// value and the horizontal derivative in each row.
			Eigen::Matrix<double, Grid::DATA_DIMENSION, 1> f0, f1, f2, f3;
			Eigen::Matrix<double, Grid::DATA_DIMENSION, 1> df0dc, df1dc, df2dc, df3dc;

			grid_.GetValue(row - 1, col - 1, p0.data());
			grid_.GetValue(row - 1, col, p1.data());
			grid_.GetValue(row - 1, col + 1, p2.data());
			grid_.GetValue(row - 1, col + 2, p3.data());
			CubicHermiteSpline<Grid::DATA_DIMENSION>(p0, p1, p2, p3, c - col,
				f0.data(), df0dc.data());

			grid_.GetValue(row, col - 1, p0.data());
			grid_.GetValue(row, col, p1.data());
			grid_.GetValue(row, col + 1, p2.data());
			grid_.GetValue(row, col + 2, p3.data());
			CubicHermiteSpline<Grid::DATA_DIMENSION>(p0, p1, p2, p3, c - col,
				f1.data(), df1dc.data());

			grid_.GetValue(row + 1, col - 1, p0.data());
			grid_.GetValue(row + 1, col, p1.data());
			grid_.GetValue(row + 1, col + 1, p2.data());
			grid_.GetValue(row + 1, col + 2, p3.data());
			CubicHermiteSpline<Grid::DATA_DIMENSION>(p0, p1, p2, p3, c - col,
				f2.data(), df2dc.data());

			grid_.GetValue(row + 2, col - 1, p0.data());
			grid_.GetValue(row + 2, col, p1.data());
			grid_.GetValue(row + 2, col + 1, p2.data());
			grid_.GetValue(row + 2, col + 2, p3.data());
			CubicHermiteSpline<Grid::DATA_DIMENSION>(p0, p1, p2, p3, c - col,
				f3.data(), df3dc.data());

			// Interpolate vertically the interpolated value from each row and
			// compute the derivative along the columns.
			CubicHermiteSpline<Grid::DATA_DIMENSION>(f0, f1, f2, f3, r - row, f, dfdr);
			if (dfdc != NULL) {
				// Interpolate vertically the derivative along the columns.
				CubicHermiteSpline<Grid::DATA_DIMENSION>(df0dc, df1dc, df2dc, df3dc,
					r - row, dfdc, NULL);
			}
		}

		// The following two Evaluate overloads are needed for interfacing
		// with automatic differentiation. The first is for when a scalar
		// evaluation is done, and the second one is for when Jets are used.
		void Evaluate(const double& r, const double& c, double* f) const {
			Evaluate(r, c, f, NULL, NULL);
		}

		template<typename JetT> void Evaluate(const JetT& r,
			const JetT& c,
			JetT* f) const {
			double frc[Grid::DATA_DIMENSION];
			double dfdr[Grid::DATA_DIMENSION];
			double dfdc[Grid::DATA_DIMENSION];
			Evaluate(r.a, c.a, frc, dfdr, dfdc);
			for (int i = 0; i < Grid::DATA_DIMENSION; ++i) {
				f[i].a = frc[i];
				f[i].v = dfdr[i] * r.v + dfdc[i] * c.v;
			}
		}

	private:
		const Grid& grid_;
	};
// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
class OccupiedSpaceCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified map, resolution
  // level, and point cloud.
  OccupiedSpaceCostFunctor(const double scaling_factor,
                           const sensor::PointCloud& point_cloud,
                           const ProbabilityGrid& probability_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        probability_grid_(probability_grid) {}

  OccupiedSpaceCostFunctor(const OccupiedSpaceCostFunctor&) = delete;
  OccupiedSpaceCostFunctor& operator=(const OccupiedSpaceCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(probability_grid_);
    BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = probability_grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                         (T(point_cloud_[i].y())), T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              T(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              T(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * (1. - residual[i]);
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const ProbabilityGrid& probability_grid)
        : probability_grid_(probability_grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = mapping::kMinProbability;
      } else {
        *value = static_cast<double>(probability_grid_.GetProbability(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return probability_grid_.limits().cell_limits().num_y_cells +
             2 * kPadding;
    }

    int NumCols() const {
      return probability_grid_.limits().cell_limits().num_x_cells +
             2 * kPadding;
    }

   private:
    const ProbabilityGrid& probability_grid_;
  };

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const ProbabilityGrid& probability_grid_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_
