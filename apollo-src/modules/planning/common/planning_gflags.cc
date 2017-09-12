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

#include "modules/planning/common/planning_gflags.h"

//发布运动规划planning的频率，单位是HZ。
DEFINE_int32(planning_loop_rate, 5, "Loop rate for planning node"); 

//用RTK定位算法得到的最近一段时间的历史轨迹
DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

//planning时，用于回退匹配的轨迹节点数量。
DEFINE_uint64(rtk_trajectory_backward, 10,
              "The number of points to be included in RTK trajectory "
              "before the matched point");
//前向匹配的轨迹节点数量
DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");
//重新规划planning的阈值，高于此值则重新planning
DEFINE_double(replanning_threshold, 2.0,
              "The threshold of position deviation "
              "that triggers the planner replanning");
//轨迹点之间的时间分辨率/间隔，0.01s
DEFINE_double(trajectory_resolution, 0.01,
              "The time resolution of "
              "output trajectory.");
