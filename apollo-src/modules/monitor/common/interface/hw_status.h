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

#ifndef MODULES_PLATFORM_INTERFACE_HW_STATUS_H_
#define MODULES_PLATFORM_INTERFACE_HW_STATUS_H_

/**
 * @namespace apollo::platform::hw
 * @brief apollo::platform::hw
 */
namespace apollo {
namespace platform {
namespace hw {

//车辆　硬件设备　的工作状态
/// Status of HW component (CAN, Camera, ...).
enum Status {
  /// This code may only be used for initialization, may never be returned by a
  /// function.
  UNDEF = -1,　//未定义
  /// HW is OK.
  OK = 0,　　　　　//硬件状态良好
  /// HW device is present and in working order, but not ready for service
  /// (e.g., no GPS lock).
  NOT_READY = 1,//在ready就绪状态
  /// HW is not preent.
  NOT_PRESENT = 2,　//未就绪
  /// HW error, can't be used.
  ERR = 3　　　　　　　　//出错，不可用
};

}  // namespace hw
}  // namespace platform
}  // namespace apollo

#endif  // MODULES_PLATFORM_INTERFACE_HW_STATUS_H_
