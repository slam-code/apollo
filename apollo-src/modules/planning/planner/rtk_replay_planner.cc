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

#include "modules/planning/planner/rtk_replay_planner.h"

#include <fstream>

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::vehicle_state::VehicleState;

RTKReplayPlanner::RTKReplayPlanner() {
  ReadTrajectoryFile(FLAGS_rtk_trajectory_filename);
}

bool RTKReplayPlanner::Plan(
    const TrajectoryPoint &start_point,
    std::vector<TrajectoryPoint> *ptr_discretized_trajectory) {
  if (complete_rtk_trajectory_.empty() || complete_rtk_trajectory_.size() < 2) {
    AERROR << "RTKReplayPlanner doesn't have a recorded trajectory or "
              "the recorded trajectory doesn't have enough valid trajectory "
              "points.";
    return false;
  }

  //找到与起点start_point最近的轨迹中的点的index。
  //why找这个值：即在历史数据里找到一个点，然后把它看做start_point的近似。
  std::size_t matched_index =
      QueryPositionMatchedPoint(start_point, complete_rtk_trajectory_);

  //默认是800个点
  std::size_t forward_buffer = FLAGS_rtk_trajectory_forward;
  
  //在match至轨迹的end之间找到一个end_index，使得end_index不越界，同时又尽可能的长。
  std::size_t end_index =
      matched_index + forward_buffer >= complete_rtk_trajectory_.size()
          ? complete_rtk_trajectory_.size() - 1
          : matched_index + forward_buffer - 1;

  //void assign(const_iterator first,const_iterator last);
  //将区间[first,last)的元素赋值到当前的vector容器中

  ptr_discretized_trajectory->assign(
      complete_rtk_trajectory_.begin() + matched_index,
      complete_rtk_trajectory_.begin() + end_index + 1);

  // reset relative time
  //修改在ptr_discretized_trajectory轨迹集合中的相对时间。以第一个point为0起始点。
  double zero_time = complete_rtk_trajectory_[matched_index].relative_time();
  for (auto &trajectory : *ptr_discretized_trajectory) {
    trajectory.set_relative_time(trajectory.relative_time() - zero_time);
  }

  // check if the trajectory has enough points;
  // if not, append the last points multiple times and
  // adjust their corresponding time stamps.

  //如果point没有800个点，则直接用最后的一个点填充直到满足800个点。
  while (ptr_discretized_trajectory->size() < FLAGS_rtk_trajectory_forward) {
    ptr_discretized_trajectory->push_back(ptr_discretized_trajectory->back());
    auto &last_point = ptr_discretized_trajectory->back();
    last_point.set_relative_time(last_point.relative_time() +
                                 FLAGS_trajectory_resolution);//r=0.01
  }
  return true;
}

void RTKReplayPlanner::ReadTrajectoryFile(const std::string &filename) {
  if (!complete_rtk_trajectory_.empty()) {
    complete_rtk_trajectory_.clear();
  }

  std::ifstream file_in(filename.c_str());//打开轨迹文件
  if (!file_in.is_open()) {
    AERROR << "RTKReplayPlanner cannot open trajectory file: " << filename;
    return;
  }
/*
file内容举例：
x,y,z,speed,acceleration,curvature,curvature_change_rate,time,theta,gear,s,throttle,brake,steering
586385.858607, 4140674.7357, -28.3670201628, 0.216666668653, 0.887545286694, 0.0227670812611, -0.0177396744278, 1496957374.6140, 2.83470068837, 1, 0.00216666668653, 22.0157165527, 13.6934461594, 12.6382980347
*/
  std::string line;
  // skip the header line.
  getline(file_in, line);

  while (true) {//读取数据
    getline(file_in, line);
    if (line == "") {
      break;
    }

    auto tokens = apollo::common::util::StringTokenizer::Split(line, "\t ");
    if (tokens.size() < 11) { //至少12维数据，一般情况下有14维数据
      AERROR << "RTKReplayPlanner parse line failed; the data dimension does "
                "not match.";
      AERROR << line;
      continue;
    }

//轨迹file存储格式：车辆 x,y,z,speed,acceleration,curvature,curvature_change_rate,time,theta,gear,s,throttle,brake,steering
    TrajectoryPoint point;
    point.set_x(std::stod(tokens[0]));
    point.set_y(std::stod(tokens[1]));
    point.set_z(std::stod(tokens[2]));

    point.set_v(std::stod(tokens[3]));
    point.set_a(std::stod(tokens[4]));

    point.set_kappa(std::stod(tokens[5]));
    point.set_dkappa(std::stod(tokens[6]));

    point.set_relative_time(std::stod(tokens[7]));

    point.set_theta(std::stod(tokens[8]));

    point.set_s(std::stod(tokens[10]));
    complete_rtk_trajectory_.push_back(point);//将轨迹point集合存储到vector中
  }

  file_in.close();
}
/*
参数1：planning起始点
参数2：历史轨迹

返回值：距离start_point的以l2距离度量的最近邻轨迹点。

*/
std::size_t RTKReplayPlanner::QueryPositionMatchedPoint(
    const TrajectoryPoint &start_point,
    const std::vector<TrajectoryPoint> &trajectory) const {
  //求距离的平方
  auto func_distance_square = [](const TrajectoryPoint &point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  //求所有的轨迹的点与start_point的距离的平方距离的最小值
  double d_min = func_distance_square(trajectory.front(), start_point.x(),
                                      start_point.y());
  std::size_t index_min = 0;

  //遍历轨迹点，找到相对于起点的最短距离点。
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    double d_temp =
        func_distance_square(trajectory[i], start_point.x(), start_point.y());
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return index_min;//返回最短距离点的index
}

}  // namespace planning
}  // namespace apollo
