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

#ifndef MODULES_PLANNING_PLANNING_NODE_H_
#define MODULES_PLANNING_PLANNING_NODE_H_

#include <vector>

#include "modules/common/proto/path_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/planning.h"
#include "modules/planning/proto/planning.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class PlanningNode
 * @brief PlanningNode is a class that
 *        implements a ros node for planning module.

 PlanningNode是Apollo的 planning模块与ROS通信的Node封装。

 */
class PlanningNode {
 public:
  /**
   * @brief Constructor
   */
  PlanningNode();

  /**
   * @brief Destructor
   */
  virtual ~PlanningNode();

  /**
   * @brief Start the planning node.
   在ROS中按照设置的频率(默认5hz),定时发布路径规划结果
   多次调用RunOnce() 。
   
   */
  void Run();

  /**
   * @brief Reset the planning node.

   复位，清空，调用Planning类的Reset()成员函数
   */
  void Reset();

 private:

//进行一次planning路径规划算法
  void RunOnce();

//序列化轨迹轨迹point到文件
  ADCTrajectory ToTrajectoryPb(
      const double header_time,
      const std::vector<common::TrajectoryPoint> &discretized_trajectory);

  Planning planning_;//进行路径规划的实例对象
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_NODE_H_ */
