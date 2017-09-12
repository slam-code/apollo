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
 * @file monitor.h
 * @brief The class of Monitor
 */

#ifndef MODULES_MONITOR_MONITOR_H_
#define MODULES_MONITOR_MONITOR_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "glog/logging.h"

#include "modules/common/monitor/monitor_buffer.h"
#include "modules/common/monitor/proto/monitor.pb.h"

/**
 * @namespace apollo::common::monitor
 * @brief apollo::common::monitor
 */
namespace apollo {
namespace common {
namespace monitor {
/*


message MonitorMessageItem {
  enum MessageSource {
    UNKNOWN = 1;
    CANBUS = 2;
    CONTROL = 3;
    DECISION = 4;
    LOCALIZATION = 5;
    PLANNING = 6;
    PREDICTION = 7;
    SIMULATOR = 8;
    HWSYS = 9;
  }

  optional MessageSource source = 1 [ default = UNKNOWN ];

  optional string msg = 2;

  enum LogLevel {
      INFO = 0;
      WARN = 1;
      ERROR = 2;
      FATAL = 3;
  }
  optional LogLevel log_level = 3 [ default = INFO ];
}

-----

message MonitorMessage {
  optional apollo.common.Header header = 1;

  repeated MonitorMessageItem item = 2;
}


*/
/**
 * class Monitor
 *
 * @brief This class help collect and publish MonitorMessage pb to monitor
 * topic. A module who wants to publish message can use macro
 * `MONITOR(log_level, log_msg)` to record messages, and call
 * Publish to broadcast the message to other modules.

Monitor类主要用于收集各个模块的message信息，并发布到相关的topic中。

 */
class Monitor {
 public:
  /**
   * @brief Construct the monitor with the source of the monitor messages. The
   * source is usually the module name who publish the monitor messages.
   * @param source the source of the monitor messages.

   创建一个消息项，指定message来源哪一个模块。
   */
  explicit Monitor(const MonitorMessageItem::MessageSource &source)
      : source_(source) {}

  /**
   * @brief Publish the messages.
   * @param messages a list of messages for
  组合多条消息，并发布到相关的topic。
   */
  virtual void Publish(const std::vector<MessageItem> &messages) const;

 private:
  ///发布已经整合的消息。
  virtual void DoPublish(MonitorMessage *message) const;

  MonitorMessageItem::MessageSource source_;
};

}  // namespace monitor
}  // namespace common
}  // namespace apollo

#endif  // MODULES_MONITOR_MONITOR_H_
