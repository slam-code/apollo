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

#ifndef MODULES_PLANNING_PLANNER_RTK_REPLAY_PLANNER_H_
#define MODULES_PLANNING_PLANNER_RTK_REPLAY_PLANNER_H_

#include <string>
#include <vector>

#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class RTKReplayPlanner
 * @brief RTKReplayPlanner is a derived class of Planner.
 *        It reads a recorded trajectory from a trajectory file and
 *        outputs proper segment of the trajectory according to vehicle
 * position.

 RTKReplayPlanner继承自虚基类Planner。
 这个类通过从历史轨迹的file文件中读取轨迹点，
 并且从这些历史轨迹点中选择连续的800个point组成历史轨迹的结果集合，
 输出到ptr_trajectory中。

 目的：从历史信息中提取与start_point相关联的轨迹point集合，
      为下一个具体的planner类进行路径规划做数据准备。


 */
class RTKReplayPlanner : public Planner {
 public:
  /**
   * @brief Constructor

   构造函数直接读取历史轨迹节点文件file中的内容.并存储到complete_rtk_trajectory_数据成员中
   */
  RTKReplayPlanner();

  /**
   * @brief Destructor
   */
  virtual ~RTKReplayPlanner() = default;

  /**
   * @brief Overrode function Plan in parent class Planner.
   * @param start_point The trajectory point where planning starts
   * @param discretized_trajectory The computed trajectory
   * @return true if planning succeeds; false otherwise.
 
参数1:planning起始点

参数2：路径规划的结果集，800个连续的point存储在ptr_trajectory中，由一系列离散的轨迹点组成。

ptr_trajectory存储的是历史轨迹节点而不是planning好的未来的轨迹节点。

故本class 命名为ReplayPlanner

*/
  bool Plan(
      const apollo::common::TrajectoryPoint &start_point,
      std::vector<apollo::common::TrajectoryPoint> *ptr_trajectory) override;

  /**
   * @brief Read the recorded trajectory file.
   * @param filename The name of the trajectory file.
   从轨迹节点文件读取历史轨迹信息。
   存储到complete_rtk_trajectory_中
   */
  void ReadTrajectoryFile(const std::string &filename);

 private:
  std::size_t QueryPositionMatchedPoint(
      const apollo::common::TrajectoryPoint &start_point,
      const std::vector<apollo::common::TrajectoryPoint> &trajectory) const;

  std::vector<apollo::common::TrajectoryPoint> complete_rtk_trajectory_;
  //存储从filename加载的轨迹point集合。
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNER_RTK_REPLAY_PLANNER_H_ */
