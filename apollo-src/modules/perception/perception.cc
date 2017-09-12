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

#include "modules/perception/perception.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;

std::string Perception::Name() const { return "perception"; }

Status Perception::Init() {
  AdapterManager::Init();
  return Status::OK();
}

Status Perception::Start() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();
  ros::Rate loop_rate(FLAGS_perception_loop_rate);//感知频率
  while (ros::ok()) {
    AdapterManager::Observe();///进行一次感知测量/检测
    PerceptionObstacles perceptionObstacles;///检测障碍物实例对象

    AdapterManager::FillPerceptionObstaclesHeader(
        Name(), perceptionObstacles.mutable_header());
    AdapterManager::PublishPerceptionObstacles(perceptionObstacles);
    ///向ROS发布检测到的障碍物对象

    TrafficLightDetection trafficLightDetection;///交通信号灯
    AdapterManager::FillTrafficLightDetectionHeader(
        Name(), trafficLightDetection.mutable_header());

    ///向ROS发布检测到的交通信号灯结果
    AdapterManager::PublishTrafficLightDetection(trafficLightDetection);
  }
  return Status::OK();
}

void Perception::Stop() {}

}  // namespace perception
}  // namespace apollo
