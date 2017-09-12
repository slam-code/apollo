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
 * @brief hysteresis filter
 */

#ifndef MODULES_CONTROL_COMMON_HYSTERESIS_FILTER_H_
#define MODULES_CONTROL_COMMON_HYSTERESIS_FILTER_H_

/**
 * @namespace apollo::control
 * @brief apollo::control

 迟滞滤波器，迟滞比较器，施密特触发电路。
 作用：抗干扰，减少输入的瞬时波动带来的控制变量的突变。考虑了历史信息以平滑整个控制过程。

 与物理中的磁滞现象类似。
 参考：https://en.wikipedia.org/wiki/Hysteresis

 */
namespace apollo {
namespace control {

class HysteresisFilter {
 public:
  HysteresisFilter() = default;
  //滤波器。
  void filter(const double input_value,     //系统输入值
              const double threshold,       //系统的噪声阈值
              const double hysteresis_upper,//系统的正向阈值
              const double hysteresis_lower,//系统的负向阈值
              int *state,
              double *output_value);

 private:
  int previous_state_ = 0;
};

}  // namespace control
}  // namespace apollo
#endif  // MODULES_CONTROL_COMMON_HYSTERESIS_FILTER_H_

/*

在电子学中，施密特触发器（英语：Schmitt trigger）是包含正反馈的比较器电路。
对于标准施密特触发器，当输入电压高于正向阈值电压，输出为高；当输入电压低于负向阈值电压，输出为低；当输入在正负向阈值电压之间，输出不改变，也就是说输出由高电准位翻转为低电准位，或是由低电准位翻转为高电准位对应的阈值电压是不同的。只有当输入电压发生足够的变化时，输出才会变化，因此将这种元件命名为触发器。这种双阈值动作被称为迟滞现象，表明施密特触发器有记忆性。从本质上来说，施密特触发器是一种双稳态多谐振荡器。
施密特触发器可作为波形整形电路，能将模拟信号波形整形为数字电路能够处理的方波波形，而且由于施密特触发器具有滞回特性，所以可用于抗干扰，其应用包括在开回路配置中用于抗扰，以及在闭回路正回授/负回授配置中用于实现多谐振荡器。
*/