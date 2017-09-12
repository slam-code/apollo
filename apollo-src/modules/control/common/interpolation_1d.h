/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#ifndef MODULES_CONTROL_COMMON_INTERPOLATION_1D_H_
#define MODULES_CONTROL_COMMON_INTERPOLATION_1D_H_

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "unsupported/Eigen/Splines"

namespace apollo {
namespace control {

class Interpolation1D {
 public:
  //[{x0,y0},{x1,y1},{x2,y2}...]
  typedef std::vector<std::pair<double, double>> DataType;

  Interpolation1D() = default;

  // Return true if init is ok.
  //先对pair<d,d>排序，然后创建插值对象。
  bool Init(const DataType& xy);

  // Only interplation x between [x_min, x_max]
  // For x out of range, start or end y value is returned.
  //对x进行插值运算，返回x应对的y值（估计值）。
  double Interpolate(double x) const;

 private:
  // Helpers to scale X values down to [0, 1]
  double ScaledValue(double x) const;

  //行向量
  //RowVector3f a, b, c; // 1x3 float matrix.
  Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  // Spline of one-dimensional "points."
  std::unique_ptr<Eigen::Spline<double, 1>> spline_;//一维样条插值
};

}  // namespace control
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_INTERPOLATION_1D_H_
