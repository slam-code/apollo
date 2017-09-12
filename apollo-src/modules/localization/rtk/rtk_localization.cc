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

#include "modules/localization/rtk/rtk_localization.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using ::Eigen::Vector3d;
using ::apollo::common::adapter::AdapterManager;
using ::apollo::common::adapter::ImuAdapter;
using ::apollo::common::monitor::MonitorMessageItem;
using ::apollo::common::Status;
using ::apollo::common::time::Clock;

//构造函数，初始化monitor_和map_offset_2个成员变量。
RTKLocalization::RTKLocalization()
    : monitor_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

Status RTKLocalization::Start() {
  //初始化配置
  AdapterManager::Init(FLAGS_rtk_adapter_config_file);

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  //回调函数OnTimer()的频率
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &RTKLocalization::OnTimer, this);

  common::monitor::MonitorBuffer buffer(&monitor_);
  //没有GPS信息
  if (!AdapterManager::GetGps()) {
    buffer.ERROR() << "GPS input not initialized. Check file "
                   << FLAGS_rtk_adapter_config_file;
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no GPS adapter");
  }
  //没有imu信息
  if (!AdapterManager::GetImu()) {
    buffer.ERROR("IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no IMU adapter");
  }
  return Status::OK();
}

Status RTKLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}


// 
void RTKLocalization::OnTimer(const ros::TimerEvent &event) {
  //dt
  double time_delay =
      common::time::ToSecond(Clock::Now()) - last_received_timestamp_sec_;
  common::monitor::MonitorBuffer buffer(&monitor_);
  //gps延迟太大，则报错，并写入log。
  if (FLAGS_enable_gps_timestamp &&
      time_delay > FLAGS_gps_time_delay_tolerance) {
    buffer.ERROR() << "GPS message time delay: " << time_delay;
    buffer.PrintLog();
  }

  // Take a snapshot of the current received messages.
  AdapterManager::Observe();

  if (AdapterManager::GetGps()->Empty()) { //没有GPS数据，则报错。
    AERROR << "GPS message buffer is empty.";
    if (service_started_) {
      buffer.ERROR("GPS message buffer is empty.");
    }
    return;
  }
  if (AdapterManager::GetImu()->Empty()) {//没有imu数据也报错。
    AERROR << "IMU message buffer is empty.";
    if (service_started_) {
      buffer.ERROR("IMU message buffer is empty.");
    }
    return;
  }

  // publish localization messages
  PublishLocalization();
  service_started_ = true;

  // watch dog
  RunWatchDog();

  //更新最新接收数据的时间
  last_received_timestamp_sec_ = common::time::ToSecond(Clock::Now());
}

///插值计算
template <class T>
T RTKLocalization::InterpolateXYZ(const T &p1, const T &p2,
                                  const double &frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}

///根据gps的时间戳寻找符合条件的imu数据
bool RTKLocalization::FindMatchingIMU(const double gps_timestamp_sec,
                                      Imu *imu_msg) {
  if (imu_msg == nullptr) {
    AERROR << "imu_msg should NOT be nullptr.";
    return false;
  }
  auto *imu_adapter = AdapterManager::GetImu();
  if (imu_adapter->Empty()) {
    AERROR << "[FindMatchingIMU]: Cannot find Matching IMU. "
           << "IMU message Queue is empty! GPS timestamp[" << gps_timestamp_sec
           << "]";
    return false;
  }

  //找到时间戳大于GPS的时间戳的第一个imu data
  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  ImuAdapter::Iterator imu_it = imu_adapter->begin();
  for (; imu_it != imu_adapter->end(); ++imu_it) {
    if ((*imu_it)->header().timestamp_sec() - gps_timestamp_sec >
        FLAGS_timestamp_sec_tolerance) {
      break;
    }
  }

  if (imu_it != imu_adapter->end()) {  // found one
    if (imu_it == imu_adapter->begin()) {
    ///imu的数据全大于GPS的数据，说明GPS延迟很大。
    
      AERROR << "[FindMatchingIMU]: IMU queue too short or request too old. "
             << "Oldest timestamp["
             << imu_adapter->GetOldestObserved().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_adapter->GetLatestObserved().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
      *imu_msg = imu_adapter->GetOldestObserved();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      InterpolateIMU(**imu_it_1, **imu_it, gps_timestamp_sec, imu_msg);
    }
  } else {

    //在imu_adapter没找到新的imu-data。
    //那么就直接使用最新测量得到的imu数据。并检查数据合理性

    // give the newest imu, without extrapolation
    *imu_msg = imu_adapter->GetLatestObserved();

    if (fabs(imu_msg->header().timestamp_sec() - gps_timestamp_sec) >
        FLAGS_report_gps_imu_time_diff_threshold) {
      // 20ms threshold to report error
      AERROR << "[FindMatchingIMU]: Cannot find Matching IMU. "
             << "IMU messages too old"
             << "Newest timestamp["
             << imu_adapter->GetLatestObserved().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
    }
  }
  return true;
}

///imu1<timestamp<imu2
void RTKLocalization::InterpolateIMU(const Imu &imu1, const Imu &imu2,
                                     const double timestamp_sec, Imu *imu_msg) {
  DCHECK_NOTNULL(imu_msg);
  ///timestamp<imu1时，则直接使用imu1
  if (timestamp_sec - imu1.header().timestamp_sec() <
      FLAGS_timestamp_sec_tolerance) { ///tolerance= 10e-7
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             FLAGS_timestamp_sec_tolerance) {
    ///timestamp>imu2时，则直接使用imu1.
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << imu2.header().timestamp_sec() << "]";
    *imu_msg = imu1;//？应该是imu2？

  } else {///符合要求imu1<=timestamp<=imu2。
    *imu_msg = imu1;
    imu_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    if (imu1.has_header() && imu1.header().has_timestamp_sec() &&
        imu2.has_header() && imu2.header().has_timestamp_sec()) {
      double time_diff =
          imu2.header().timestamp_sec() - imu1.header().timestamp_sec();
      if (fabs(time_diff) >= 0.001) {
      ///大于阈值才插值计算，小于则直接使用imu1的数据。
        double frac1 =
            (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;

        if (imu1.has_imu() && imu1.imu().has_angular_velocity() &&
            imu2.has_imu() && imu2.imu().has_angular_velocity()) {
          auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                    imu2.imu().angular_velocity(), frac1);
          imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
        }

        if (imu1.has_imu() && imu1.imu().has_linear_acceleration() &&
            imu2.has_imu() && imu2.imu().has_linear_acceleration()) {
          auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                    imu2.imu().linear_acceleration(), frac1);
          imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
        }
      }
    }
  }
}

/*
message LocalizationEstimate {
  optional apollo.common.Header header = 1;
  optional apollo.localization.Pose pose = 2;
  optional Uncertainty uncertainty = 3;

  // The time of pose measurement, seconds since the GPS epoch (Jan 6, 1980).
  optional double measurement_time = 4;  // In seconds.
}

*/

void RTKLocalization::PrepareLocalizationMsg(
    LocalizationEstimate *localization) {
  const auto &gps_msg = AdapterManager::GetGps()->GetLatestObserved();

  bool imu_valid = true;
  Imu imu_msg;
  if (FLAGS_enable_gps_imu_interprolate) {//允许imu信息融合。
    // find the matching gps and imu message
    double gps_time_stamp = gps_msg.header().timestamp_sec();
    if (!FindMatchingIMU(gps_time_stamp, &imu_msg)) {
      imu_valid = false;
    }
  } else {
    imu_msg = AdapterManager::GetImu()->GetLatestObserved();
  }

  if (imu_valid &&
      fabs(gps_msg.header().timestamp_sec() - imu_msg.header().timestamp_sec() >
           FLAGS_gps_imu_timestamp_sec_diff_tolerance)) {
    //超过20ms即看做不为同一时刻的测量值

    // not the same time stamp, 20ms threshold
    AERROR << "[PrepareLocalizationMsg]: time stamp of GPS["
           << gps_msg.header().timestamp_sec()
           << "] is different from timestamp of IMU["
           << imu_msg.header().timestamp_sec() << "]";
  }

  //根据gps和imu综合得到location位置。
  ComposeLocalizationMsg(gps_msg, imu_msg, localization);
}

void RTKLocalization::ComposeLocalizationMsg(
    const ::apollo::localization::Gps &gps_msg,
    const ::apollo::localization::Imu &imu_msg,
    LocalizationEstimate *localization) {
  localization->Clear();

  // header，填充head信息。
  AdapterManager::FillLocalizationHeader(FLAGS_localization_module_name,
                                         localization->mutable_header());
  if (FLAGS_enable_gps_timestamp) {
    // copy time stamp, do NOT use Clock::Now()
    //location 的时间戳是gps的时间戳
    localization->mutable_header()->set_timestamp_sec(
        gps_msg.header().timestamp_sec());
  }

  // combine gps and imu
  auto mutable_pose = localization->mutable_pose();
  if (gps_msg.has_localization()) {
    const auto &pose = gps_msg.localization();//gps的定位。

    if (pose.has_position()) {
      // position
      // world frame -> map frame
      //将世界坐标转换为机器坐标。
      mutable_pose->mutable_position()->set_x(pose.position().x() -
                                              map_offset_[0]);
      mutable_pose->mutable_position()->set_y(pose.position().y() -
                                              map_offset_[1]);
      mutable_pose->mutable_position()->set_z(pose.position().z() -
                                              map_offset_[2]);
    }

    // orientation
    if (pose.has_orientation()) {
      mutable_pose->mutable_orientation()->CopyFrom(pose.orientation());
      double heading = common::math::QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz());
      mutable_pose->set_heading(heading);
    }
    // linear velocity
    if (pose.has_linear_velocity()) {
      mutable_pose->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
    }
  }

  if (imu_msg.has_imu()) {
    const auto &imu = imu_msg.imu();
    // linear acceleration
    if (imu.has_linear_acceleration()) {
      if (FLAGS_enable_map_reference_unify) {
        if (localization->pose().has_orientation()) {
          // linear_acceleration:
          // convert from vehicle reference to map reference
          Vector3d orig(imu.linear_acceleration().x(),
                        imu.linear_acceleration().y(),
                        imu.linear_acceleration().z());
          Vector3d vec = common::math::QuaternionRotate(
              localization->pose().orientation(), orig);
          mutable_pose->mutable_linear_acceleration()->set_x(vec[0]);
          mutable_pose->mutable_linear_acceleration()->set_y(vec[1]);
          mutable_pose->mutable_linear_acceleration()->set_z(vec[2]);

          // linear_acceleration_vfr
          mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
              imu.linear_acceleration());

        } else {
          AERROR << "[PrepareLocalizationMsg]: "
                 << "fail to convert linear_acceleration";
        }
      } else {
        mutable_pose->mutable_linear_acceleration()->CopyFrom(
            imu.linear_acceleration());
      }
    }

    // angular velocity
    if (imu.has_angular_velocity()) {
      if (FLAGS_enable_map_reference_unify) {
        if (localization->pose().has_orientation()) {
          // angular_velocity:
          // convert from vehicle reference to map reference
          Vector3d orig(imu.angular_velocity().x(), imu.angular_velocity().y(),
                        imu.angular_velocity().z());
          Vector3d vec = common::math::QuaternionRotate(
              localization->pose().orientation(), orig);
          mutable_pose->mutable_angular_velocity()->set_x(vec[0]);
          mutable_pose->mutable_angular_velocity()->set_y(vec[1]);
          mutable_pose->mutable_angular_velocity()->set_z(vec[2]);

          // angular_velocity_vf
          mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
              imu.angular_velocity());
        } else {
          AERROR << "[PrepareLocalizationMsg]: "
                 << "fail to convert angular_velocity";
        }
      } else {
        mutable_pose->mutable_angular_velocity()->CopyFrom(
            imu.angular_velocity());
      }
    }
  }
}

void RTKLocalization::PublishLocalization() {
  LocalizationEstimate localization;
  PrepareLocalizationMsg(&localization);//计算定位结果

  // publish localization messages
  AdapterManager::PublishLocalization(localization); //发送定位结果
  AINFO << "[OnTimer]: Localization message publish success!";
}

void RTKLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) {
    return;
  }

  common::monitor::MonitorBuffer buffer(&monitor_);

  // check GPS time stamp against ROS timer
  double gps_delay_sec =
      common::time::ToSecond(Clock::Now()) -
      AdapterManager::GetGps()->GetLatestObserved().header().timestamp_sec();
  int64_t gps_delay_cycle_cnt =
      static_cast<int64_t>(gps_delay_sec * FLAGS_localization_publish_freq);

  bool msg_lost = false;
  if (FLAGS_enable_gps_timestamp &&
      (gps_delay_cycle_cnt > FLAGS_report_threshold_err_num)) {
    msg_lost = true;

    buffer.ERROR() << "Raw GPS Message Lost. GPS message is "
                   << gps_delay_cycle_cnt << " cycle " << gps_delay_sec
                   << " sec behind current time.";
    buffer.PrintLog();
  }

  // check IMU time stamp against ROS timer
  double imu_delay_sec =
      common::time::ToSecond(Clock::Now()) -
      AdapterManager::GetImu()->GetLatestObserved().header().timestamp_sec();
  int64_t imu_delay_cycle_cnt =
      static_cast<int64_t>(imu_delay_sec * FLAGS_localization_publish_freq);
  if (FLAGS_enable_gps_timestamp &&
      imu_delay_cycle_cnt > FLAGS_report_threshold_err_num) {
    msg_lost = true;

    buffer.ERROR() << "Raw GPS Message Lost. IMU message is "
                   << imu_delay_cycle_cnt << " cycle " << imu_delay_sec
                   << " sec behind current time.";
    buffer.PrintLog();
  }

  // to prevent it from beeping continuously
  if (msg_lost && (last_reported_timestamp_sec_ < 1. ||
                   common::time::ToSecond(Clock::Now()) >
                       last_reported_timestamp_sec_ + 1.)) {
    AERROR << "gps/imu frame lost!";
    last_reported_timestamp_sec_ = common::time::ToSecond(Clock::Now());
  }
}

}  // namespace localization
}  // namespace apollo
