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
#include "modules/planning/planning.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner_factory.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterManager;
using TrajectoryPb = ADCTrajectory;

Planning::Planning() {
  //利用工厂方法模式，new 一个实例对象
  ptr_planner_ = PlannerFactory::CreateInstance(PlannerType::RTK_PLANNER);
}

//
bool Planning::Plan(const common::vehicle_state::VehicleState &vehicle_state,
                    const bool is_on_auto_mode, const double publish_time,
                    std::vector<TrajectoryPoint> *planning_trajectory) {
  //间隔时间，0.2s
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;//默认是1.0/5
  double execution_start_time = publish_time;

  /*如果是自动驾驶模式，并且【预测】历史轨迹不为空(已有历史预测信息)
那么:
【1】从【预测的】历史轨迹中寻找与当前时间匹配的point
【2】计算匹配点与当前点的距离的平方差
【3】若小于阈值(2.0m)，则不需要重新进行路径规划,直接从匹配点开始路径规划.
【4】将last_traj修改为*planning_traj以用于下一次路径规划
【5】若非自动驾驶模式/没有历史轨迹点/匹配点距离太远：
【6】从当前车辆状态获取point位置，然后再从当前位置进行路径规划。
【7】保存预测轨迹，然后用于下一次预测
  */
  if (is_on_auto_mode && !last_trajectory_.empty()) {
    // if the auto-driving mode is on and we have the trajectory from last
    // cycle, then
    // find the planning starting point from the last planning result.
    // this ensures the smoothness of planning output and
    // therefore the smoothness of control execution.

    //从历史轨迹中找到匹配的point
    auto matched_info =
        ComputeStartingPointFromLastTrajectory(execution_start_time);
    TrajectoryPoint matched_point = matched_info.first;
    std::size_t matched_index = matched_info.second;

    // Compute the position deviation between current vehicle
    // position and target vehicle position.
    // If the deviation exceeds a specific threshold,
    // it will be unsafe to planning from the matched point.

    //计算匹配点与当前点的距离的平方差
    double dx = matched_point.x() - vehicle_state.x();
    double dy = matched_point.y() - vehicle_state.y();
    double position_deviation = std::sqrt(dx * dx + dy * dy);

    //若小于2.0，则不需要重新进行路径规划。

    if (position_deviation < FLAGS_replanning_threshold) {
      // planned trajectory from the matched point, the matched point has
      // relative time 0.
      //直接从匹配点开始路径规划
      bool planning_succeeded =
          ptr_planner_->Plan(matched_point, planning_trajectory);

      if (!planning_succeeded) {
        last_trajectory_.clear();
        return false;
      }

      // a segment of last trajectory to be attached to planned trajectory in
      // case controller needs.

      //后退10个点
      auto overhead_trajectory = GetOverheadTrajectory(
          matched_index, (std::size_t)FLAGS_rtk_trajectory_backward);//默认是10

      //在指定位置loc前插入区间[start, end)的所有元素 .
      planning_trajectory->insert(planning_trajectory->begin(),
                                  overhead_trajectory.begin(),
                                  overhead_trajectory.end());

      // store the planned trajectory and header info for next planning cycle.

      //将last_traj修改为*planning_traj以用于下一次路径规划
      last_trajectory_ = *planning_trajectory;
      last_header_time_ = execution_start_time;
      return true;
    }
  }

  // if 1. the auto-driving mode is off or
  //    2. we don't have the trajectory from last planning cycle or
  //    3. the position deviation from actual and target is too high
  // then planning from current vehicle state.
  
  //非自动驾驶模式/没有历史轨迹点/匹配点距离太远：

  //从当前车辆状态获取point位置，然后再从当前位置进行路径规划。
  TrajectoryPoint vehicle_state_point =
      ComputeStartingPointFromVehicleState(vehicle_state, planning_cycle_time);

  bool planning_succeeded =
      ptr_planner_->Plan(vehicle_state_point, planning_trajectory);
  if (!planning_succeeded) {
    last_trajectory_.clear();
    return false;
  }
  // store the planned trajectory and header info for next planning cycle.
  //保存预测轨迹，然后用于下一次预测
  last_trajectory_ = *planning_trajectory;
  last_header_time_ = execution_start_time;
  return true;
}

//计算从上次【预测的】轨迹记录历史得到的起始点的车辆自身point位置
std::pair<TrajectoryPoint, std::size_t>
Planning::ComputeStartingPointFromLastTrajectory(
    const double start_time) const {
  /*函数lower_bound()在first和last中的前闭后开区间进行二分查找，返回大于或等于val的第一个元素位置。如果所有元素都小于val，则返回last的位置  
  */

  //找到轨迹point，使得其time小于给定的time
  auto comp = [](const TrajectoryPoint &p, const double t) {
    return p.relative_time() < t;
  };

  //二分查找，找到【start_time】小于【轨迹point的time+last_header】的第一个值。
  auto it_lower =
      std::lower_bound(last_trajectory_.begin(), last_trajectory_.end(),
                       start_time - last_header_time_, comp);
  if (it_lower == last_trajectory_.end()) {
    it_lower--;
  }
  std::size_t index = it_lower - last_trajectory_.begin();
  return std::pair<TrajectoryPoint, std::size_t>(*it_lower, index);
}

//计算车辆启动时的地理位置所在的轨迹point。
TrajectoryPoint Planning::ComputeStartingPointFromVehicleState(
    const common::vehicle_state::VehicleState &vehicle_state,
    const double forward_time) const {
  // Eigen::Vector2d estimated_position =
  // vehicle_state.EstimateFuturePosition(forward_time);
  TrajectoryPoint point;
  // point.set_x(estimated_position.x());
  // point.set_y(estimated_position.y());

  //获取当前车辆的位置x,y,z
  point.set_x(vehicle_state.x());
  point.set_y(vehicle_state.y());
  point.set_z(vehicle_state.z());

  //获取当前车辆的速度，加速度信息
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_kappa(0.0);//曲率，半径越小，曲率越大。曲线接近平直的时候，曲率接近0
  const double speed_threshold = 0.1;

  //设置转弯半径
  if (point.v() > speed_threshold) {
    point.set_kappa(vehicle_state.angular_velocity() /
                    vehicle_state.linear_velocity());
  }
  point.set_dkappa(0.0);
  point.set_s(0.0);
  point.set_relative_time(0.0);
  return point;
}

void Planning::Reset() {
  last_header_time_ = 0.0;
  last_trajectory_.clear();
}

std::vector<TrajectoryPoint> Planning::GetOverheadTrajectory(
    const std::size_t matched_index, const std::size_t buffer_size) {


  const std::size_t start_index =
      matched_index < buffer_size ? 0 : matched_index - buffer_size;

  std::vector<TrajectoryPoint> overhead_trajectory(
      last_trajectory_.begin() + start_index,
      last_trajectory_.begin() + matched_index);

  double zero_relative_time = last_trajectory_[matched_index].relative_time();
  // reset relative time
  for (auto &p : overhead_trajectory) {
    p.set_relative_time(p.relative_time() - zero_relative_time);
  }
  return overhead_trajectory;//正常情况下回退10个轨迹point
}

}  // namespace planning
}  // namespace apollo
