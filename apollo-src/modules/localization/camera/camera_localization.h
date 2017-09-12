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

/**
 * @file localization_rtk.h
 * @brief The class of CameraLocalization
 */

#ifndef MODULES_LOCALIZATION_CAMERA_LOCALIZATION_CAMERA_H_
#define MODULES_LOCALIZATION_CAMERA_LOCALIZATION_CAMERA_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/localization/proto/camera.pb.h"
#include "modules/localization/proto/camera_parameter.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "glog/logging.h"
#include "gtest/gtest_prod.h"
#include "modules/common/monitor/monitor.h"
#include "modules/common/status/status.h"
#include "modules/localization/localization_base.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class CameraLocalization
 *
 * @brief generate localization info based on Camera


CameraLocalization类继承自LocalizationBase，重写了Start()和Stop() 2个函数。
该类主要是利用摄像机进行定位算法。但是目前Apollo的1.0版本还没有用具体的相机定位算法。可能后续会补充。
目前只是添加了这个功能原型，搭建了消息收发的接口。但是目前没有实用。
 */
class CameraLocalization : public LocalizationBase {
 public:
  CameraLocalization();
  virtual ~CameraLocalization() = default;

  /**
   * @brief module start function
   * @return start status

   初始化配置，并执行本类的定位算法
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status

  清理工作
   */
  apollo::common::Status Stop() override;

 private:
  void OnTimer(const ros::TimerEvent &event);//数据到达，进行一次定位算法
  bool PublishLocalization();//发布gps和camera二者的综合定位结果

  void RunWatchDog();//检查时间合理性，是否延迟过大。

  bool PrepareLocalizationMsg(LocalizationEstimate *localization);
  //该函数尚未实现。应该是利用camera产生定位结果

  //根据gps_msg和camera_msg，得到综合的定位结果
  bool CreateLocalizationMsg(const ::apollo::localization::Gps &gps_msg,
                             const ::apollo::localization::Camera &camera_msg,
                             LocalizationEstimate *localization);

 private:
  ros::Timer timer_;  //ros系统的时间

  //监视本模块工作状态，以便于发送给ros topic。
  apollo::common::monitor::Monitor monitor_; 

  CameraParameter camera_parameter_; //相机参数，包括内参和外参
  double last_received_timestamp_sec_ = 0.0;//最近接收data的时间
  double last_reported_timestamp_sec_ = 0.0;//最近发布定位的时间
  bool use_imu_ = false;                    //融合imu
  bool service_started_ = false;            //开始工作
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_Camera_LOCALIZATION_Camera_H_
