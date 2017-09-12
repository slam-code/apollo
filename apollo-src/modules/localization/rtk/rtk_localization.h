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
 * @file rtk_localization.h
 * @brief The class of RTKLocalization
 */

#ifndef MODULES_LOCALIZATION_RTK_RTK_LOCALIZATION_H_
#define MODULES_LOCALIZATION_RTK_RTK_LOCALIZATION_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

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
 * @class RTKLocalization
 *
 * @brief generate localization info based on RTK
    


Real_Time_Kinematic，即差分GPS技术：将一台GPS接收机安置在基准站上进行观测。根据基准站已知精密坐标，计算出基准站到卫星的距离改正数，并由基准站实时将这一数据发送出去。用户接收机在进行GPS观测的同时，也接收到基准站发出的改正数，并对其定位结果进行改正，从而提高定位精度。

目前Apollo的1.0主要依靠RTK-GPS定位算法。其他算法目前还尚见到。
关于RTK-GPS可以参阅https://en.wikipedia.org/wiki/Real_Time_Kinematic

优点:精度能达到厘米级。
缺点：要安装基站，成本高，在没有基站的户外就不适用了。

RTKLocalization类继承自LocalizationBase，重写了Start()和Stop() 2个函数。

 */
class RTKLocalization : public LocalizationBase {
 public:
  RTKLocalization();
  virtual ~RTKLocalization() = default;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  apollo::common::Status Stop() override;

 private:
  void OnTimer(const ros::TimerEvent &event);//数据到达，进行一次定位算法
  void PublishLocalization();//发布 定位结果
  void RunWatchDog();//检查时间合理性，是否延迟过大。

  void PrepareLocalizationMsg(LocalizationEstimate *localization);
  ///计算gps的定位结果

  void ComposeLocalizationMsg(const ::apollo::localization::Gps &gps,
                              const ::apollo::localization::Imu &imu,
                              LocalizationEstimate *localization);
  ///将gps和imu做数据融合。

  ///根据时间戳找到大于该时间戳的第一个imu数据
  bool FindMatchingIMU(const double gps_timestamp_sec, Imu *imu_msg);

  ///对imu数据进行插值计算，得到给定timestamp_sec的值
  void InterpolateIMU(const Imu &imu1, const Imu &imu2,
                      const double timestamp_sec, Imu *msgbuf);

  template <class T>
  T InterpolateXYZ(const T &p1, const T &p2, const double &frac1);

 private:
  ros::Timer timer_;//ros系统的时间
  //监视本模块工作状态，以便于发送给ros topic。
  apollo::common::monitor::Monitor monitor_;

  const std::vector<double> map_offset_;//在地图上的偏移量
  double last_received_timestamp_sec_ = 0.0;//最近接收data的时间
  double last_reported_timestamp_sec_ = 0.0;//最近发布定位的时间
  bool service_started_ = false; //开始工作

  FRIEND_TEST(RTKLocalizationTest, InterpolateIMU);//便于测试
  FRIEND_TEST(RTKLocalizationTest, ComposeLocalizationMsg);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_RTK_RTK_LOCALIZATION_H_
