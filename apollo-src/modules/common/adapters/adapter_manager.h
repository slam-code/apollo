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
 * @file
 */

#ifndef MODULES_ADAPTERS_ADAPTER_MANAGER_H_
#define MODULES_ADAPTERS_ADAPTER_MANAGER_H_

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "modules/common/adapters/adapter.h"
#include "modules/common/adapters/message_adapters.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"

#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::common::adapter
 * @brief apollo::common::adapter
 */
namespace apollo {
namespace common {
namespace adapter {

/// Macro to prepare all the necessary adapter functions when adding a
/// new input/output. For example when you want to listen to
/// car_status message for your module, you can do
/// REGISTER_ADAPTER(CarStatus) write an adapter class called
/// CarStatusAdapter, and call EnableCarStatus(`car_status_topic`,
/// true, `callback`(if there's one)) in AdapterManager.
/// 宏定义。提供适配器ADAPTER的name即可注册到AdapterManager中。
///主要作用是提供一系列公共的函数接口避免代码冗余。

#define REGISTER_ADAPTER(name)                                                 \
 public:                                                                       \
  static void Enable##name(const std::string &topic_name,                      \
                           AdapterConfig::Mode mode,                           \
                           int message_history_limit) {                        \
    CHECK(message_history_limit > 0)                                           \
        << "Message history limit must be greater than 0";                     \
    instance()->InternalEnable##name(topic_name, mode, message_history_limit); \
  }                                                                            \
  static name##Adapter *Get##name() {                                          \
    return instance()->InternalGet##name();                                    \
  }                                                                            \
  static void Feed##name##ProtoFile(const std::string &proto_file) {           \
    CHECK(instance()->name##_)                                                 \
        << "Initialize adapter before feeding protobuf";                       \
    Get##name()->FeedProtoFile(proto_file);                                    \
  }                                                                            \
  static void Publish##name(const name##Adapter::DataType &data) {             \
    instance()->InternalPublish##name(data);                                   \
  }                                                                            \
  static void Fill##name##Header(const std::string &module_name,               \
                                 apollo::common::Header *header) {             \
    instance()->name##_->FillHeader(module_name, header);                      \
  }                                                                            \
  static void Set##name##Callback(name##Adapter::Callback callback) {          \
    CHECK(instance()->name##_)                                                 \
        << "Initialize adapter before setting callback";                       \
    instance()->name##_->SetCallback(callback);                                \
  }                                                                            \
  template <class T>                                                           \
  static void Set##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data), T *obj) {            \
    Set##name##Callback(std::bind(fp, obj, std::placeholders::_1));            \
  }                                                                            \
                                                                               \
 private:                                                                      \
  std::unique_ptr<name##Adapter> name##_;                                      \
  ros::Publisher name##publisher_;                                             \
  ros::Subscriber name##subscriber_;                                           \
                                                                               \
  void InternalEnable##name(const std::string &topic_name,                     \
                            AdapterConfig::Mode mode,                          \
                            int message_history_limit) {                       \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, message_history_limit));          \
    if (mode != AdapterConfig::PUBLISH_ONLY && node_handle_) {                 \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, message_history_limit,           \
                                  &name##Adapter::OnReceive, name##_.get());   \
    }                                                                          \
    if (mode != AdapterConfig::RECEIVE_ONLY && node_handle_) {                 \
      name##publisher_ = node_handle_->advertise<name##Adapter::DataType>(     \
          topic_name, message_history_limit);                                  \
    }                                                                          \
                                                                               \
    observers_.push_back([this]() { name##_->Observe(); });                    \
  }                                                                            \
  name##Adapter *InternalGet##name() { return name##_.get(); }                 \
  void InternalPublish##name(const name##Adapter::DataType &data) {            \
    name##publisher_.publish(data);                                            \
  }

/**
 * @class AdapterManager
 *
 * @brief this class hosts all the specific adapters and manages them.
 * It provides APIs for the users to initialize, access and interact
 * with the adapters that they are interested in.
 *
 * \par
 * Each (potentially) useful adapter needs to be registered here with
 * the macro REGISTER_ADAPTER.
 *
 * \par
 * The AdapterManager is a singleton.


 AdapterManager 是单例模式。
 所有的message/IO适配器均需要通过REGISTER_ADAPTER(name)在这里注册。
 所有的数据交互IO/通信也通过AdapterManager来进行统一的管理。


 */
class AdapterManager {
 public:
  /**
   * @brief Initialize the /class AdapterManager singleton.

   加载默认config
   */
  static void Init();

  /**
   * @brief Initialize the /class AdapterManager singleton with the
   * provided configuration. The configuration is specified by the
   * file path.
   * @param adapter_config_filename the path to the proto file that
   * contains the adapter manager configuration.

   加载给定filename的config
   */
  static void Init(const std::string &adapter_config_filename);

  /**
   * @brief Initialize the /class AdapterManager singleton with the
   * provided configuration.
   * @param configs the adapter manager configuration proto.

   加载proto格式的配置。

  message AdapterManagerConfig {
  repeated AdapterConfig config = 1;
  required bool is_ros = 2;  // Whether the message comes from ROS
}

   */
  static void Init(const AdapterManagerConfig &configs);

  static void Observe();//对所有的adapter对象执行observe()函数。

  /**
   * @brief create a timer which will call a callback at the specified
   * rate. It takes a class member function, and a bare pointer to the
   * object to call the method on.
   ROS节点回调函数。
   */
  template <class T>
  static ros::Timer CreateTimer(ros::Duration period,
                                void (T::*callback)(const ros::TimerEvent &),
                                T *obj, bool oneshot = false,
                                bool autostart = true) {
    CHECK(instance()->node_handle_)
        << "ROS node is only available in ROS mode, "
           "check your adapter config file!";
    return instance()->node_handle_->createTimer(period, callback, obj, oneshot,          
                                       autostart);
  }

 private:
  /// The node handler of ROS, owned by the /class AdapterManager
  /// singleton.
  std::unique_ptr<ros::NodeHandle> node_handle_;

  /// Observe() callbacks that will be used to to call the Observe()
  /// of enabled adapters.
  /// 
  std::vector<std::function<void()>> observers_;

  /// The following code registered all the adapters of interest.
  REGISTER_ADAPTER(Chassis);
  REGISTER_ADAPTER(ChassisDetail);
  REGISTER_ADAPTER(ControlCommand);
  REGISTER_ADAPTER(Decision);
  REGISTER_ADAPTER(Gps);
  REGISTER_ADAPTER(Imu);
  REGISTER_ADAPTER(Camera);
  REGISTER_ADAPTER(Localization);
  REGISTER_ADAPTER(Monitor);
  REGISTER_ADAPTER(Pad);
  REGISTER_ADAPTER(PerceptionObstacles);
  REGISTER_ADAPTER(PlanningTrajectory);
  REGISTER_ADAPTER(Prediction);
  REGISTER_ADAPTER(TrafficLightDetection);

  DECLARE_SINGLETON(AdapterManager); //instance()函数原型出处。
};

}  // namespace adapter
}  // namespace common
}  // namespace apollo

#endif
