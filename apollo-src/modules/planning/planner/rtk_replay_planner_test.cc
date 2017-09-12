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

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

using apollo::common::TrajectoryPoint;

namespace apollo {
namespace planning {

class RTKReplayPlannerTest : public ::testing::Test {};

TEST_F(RTKReplayPlannerTest, ComputeTrajectory) {
  FLAGS_rtk_trajectory_filename = "modules/planning/testdata/garage.csv";
  RTKReplayPlanner planner;

  TrajectoryPoint start_point;
  start_point.set_x(586385.782842);
  start_point.set_y(4140674.76063);

  std::vector<TrajectoryPoint> trajectory;
  bool planning_succeeded = planner.Plan(start_point, &trajectory);

  EXPECT_TRUE(planning_succeeded);
  EXPECT_TRUE(!trajectory.empty());
  //800个点。不够时用最后一个点填充。
  EXPECT_EQ(trajectory.size(), (std::size_t)FLAGS_rtk_trajectory_forward);

  auto first_point = trajectory.front();
  EXPECT_DOUBLE_EQ(first_point.x(), 586385.782841);//第一个匹配点，.csv第20个
  EXPECT_DOUBLE_EQ(first_point.y(), 4140674.76065);

  auto last_point = trajectory.back();
  EXPECT_DOUBLE_EQ(last_point.x(), 586355.063786);//最后一个匹配点，.csv第819个。
  EXPECT_DOUBLE_EQ(last_point.y(), 4140681.98605);
}

/*
x,y,z,speed,acceleration,curvature,curvature_change_rate,time,theta,gear,s,throttle,brake,steering
586385.858607, 4140674.7357, -28.3670201628, 0.216666668653, 0.887545286694, 0.0227670812611, -0.0177396744278, 1496957374.6140, 2.83470068837, 1, 0.00216666668653, 22.0157165527, 13.6934461594, 12.6382980347
0.0, 1496957374.6246, 2.83481834381, 1, 0.00433333337307, 22.0157165527, 13.6751356125, 12.6382980347

*/
TEST_F(RTKReplayPlannerTest, ErrorTest) {
  FLAGS_rtk_trajectory_filename =
      "modules/planning/testdata/garage_no_file.csv";
  RTKReplayPlanner planner;
  FLAGS_rtk_trajectory_filename = "modules/planning/testdata/garage_error.csv";
  RTKReplayPlanner planner_with_error_csv;//file不完整，只有1行数据时，直接返回false
  TrajectoryPoint start_point;
  start_point.set_x(586385.782842);
  start_point.set_y(4140674.76063);
  std::vector<TrajectoryPoint> trajectory;
  EXPECT_TRUE(!planner_with_error_csv.Plan(start_point, &trajectory));
}

}  // namespace planning
}  // namespace apollo
