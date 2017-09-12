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

#ifndef MODULES_PLANNING_PLANNING_H_
#define MODULES_PLANNING_PLANNING_H_

#include <memory>
#include <utility>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/planner/planner.h"

namespace apollo {
namespace planning {
//class Planning 是上层的路径规划方法封装。
class Planning {
 public:
  /**
   * @brief Constructor
   */
  Planning();

  /**
   * @brief Destructor
   */
  ~Planning() = default;

  /**
   * @brief Plan the trajectory given current vehicle state
   * @param vehicle_state variable describes the vehicle state, including
   * position,
   *        velocity, acceleration, heading, etc
   * @param is_on_auto_mode whether the current system is on auto-driving mode
   * @param publishable_trajectory the computed planning trajectory

   从历史信息+目前的车辆状态(车辆速度，加速度，行驶方向等)进行路径规划，
   结果存储在discretized_trajectory中。
   
   参数1：目前车辆状态信息
   参数2:是否自动驾驶模式
   参数3:路径规划发布时间
   参数4:路径规划的轨迹point结果集合

   */
  bool Plan(const common::vehicle_state::VehicleState &vehicle_state,
            const bool is_on_auto_mode, const double publish_time,
            std::vector<common::TrajectoryPoint> *discretized_trajectory);

  /**
   * @brief Reset the planner to initial state.

   重置为初始状态。清空轨迹point的信息
   */
  void Reset();

 private:

/*
计算从上次【预测】的轨迹记录历史得到的起始点的车辆自身point位置
参数:当前时间
返回值：轨迹point+索引

*/
  std::pair<common::TrajectoryPoint, std::size_t>
  ComputeStartingPointFromLastTrajectory(const double curr_time) const;


/*计算车辆启动时的地理位置所在的轨迹point。
参数1:当前车辆状态(车辆速度，加速度，行驶方向等)
参数2：planning的预测时间段。

*/
  common::TrajectoryPoint ComputeStartingPointFromVehicleState(
      const common::vehicle_state::VehicleState &vehicle_state,
      const double forward_time) const;

/*
参数1：与起始点match的轨迹点
参数2：需要回退的size，正常情况下回退10个轨迹point
返回值：
    在match之前的10个轨迹point

  */
  std::vector<common::TrajectoryPoint> GetOverheadTrajectory(
      const std::size_t matched_index, const std::size_t buffer_size);

  std::unique_ptr<Planner> ptr_planner_;//由工厂方法模式创建的路径规划实例。

  std::vector<common::TrajectoryPoint> last_trajectory_;
  //最近近一段时间planning的轨迹point集合

  double last_header_time_ = 0.0; //最近planning时间
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_H_ */
