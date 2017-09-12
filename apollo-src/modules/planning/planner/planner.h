/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PLANNING_PLANNER_PLANNER_H_
#define MODULES_PLANNING_PLANNER_PLANNER_H_

#include <vector>

#include "modules/common/proto/path_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.

 Planner是纯虚基类。有一个公有的Plan()：
 纯虚函数。继承它的子类必须重写这个纯虚函数才能实例化。
 所有的路径规划的class都继承自这个类。
 目的：虚函数指针，运行时决断，父类指针指向子类对象，runtime时调用子类的成员函数。

 具体到Planner 而言就是提供一个公有的接口 virtual bool Plan().
 使得在运行时可以执行不同的planning算法。

 */
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  /**
   * @brief Compute a trajectory for execution.
   * @param start_point The trajectory point where planning starts
   * @param discretized_trajectory The computed trajectory
   * @return true if planning succeeds; false otherwise.

参数1:planning起始点
参数2：路径规划结果集，由一系列离散的轨迹点组成。

目的：根据历史行驶的一系列轨迹节点，
并结合Perception模块+Prediction模块+Decision模块+
Localization地图定位模块，来进行推算未来一段时间的行驶轨迹。
本质上是路径规划算法。

   */
  virtual bool Plan(
      const apollo::common::TrajectoryPoint &start_point,
      std::vector<apollo::common::TrajectoryPoint> *discretized_trajectory) = 0;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNER_PLANNER_H_ */
