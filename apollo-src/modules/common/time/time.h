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
 *
 * @brief This library provides the utilities to deal with timestamps.
 * currently our assumption is that every timestamp will be of a
 * precision at 1us.
 */
#ifndef MODULES_COMMON_TIME_TIME_H_
#define MODULES_COMMON_TIME_TIME_H_

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <type_traits>

#include "modules/common/macro.h"

/**
 * @namespace apollo::common::time
 * @brief apollo::common::time

apollo内部使用c++ 11的 chrono库作为时间管理工具。默认精度是纳秒(1e-9).
std::chrono::duration 表示时间间隔大小。 
std::chrono::time_point 表示时间中的一个点。

  定义2个别名：
  1)Duration,1纳秒，1e-9s。
  2)Timestamp,以纳秒ns为单位的时间点。

 */
namespace apollo {
namespace common {
namespace time {

/**
 * @class Duration
 * @brief the default Duration is of precision nanoseconds (1e-9 seconds).
 */
using Duration = std::chrono::nanoseconds;

/**
 * @class Timestamp
 * @brief the default timestamp uses std::chrono::system_clock. The
 * system_clock is a system-wide realtime clock.
 必须是使用64位整数表示。
 */
using Timestamp = std::chrono::time_point<std::chrono::system_clock, Duration>;

static_assert(std::is_same<int64_t, Duration::rep>::value,
              "The underlying type of the microseconds should be int64.");

using nanos = std::chrono::nanoseconds; //ns
using micros = std::chrono::microseconds; //us
using millis = std::chrono::milliseconds; //ms
using seconds = std::chrono::seconds;     //s
using minutes = std::chrono::minutes;     //m
using hours = std::chrono::hours;         //h

/**
 * @brief converts the input duration (nanos) to a 64 bit integer, with
 * the unit specified by PrecisionDuration.
 * @param duration the input duration that needs to be converted to integer.
 * @return an integer representing the duration in the specified unit.

 将纳秒ns转换为以PrecisionDuration为精度单位计的int64整数。
 count()函数返回的是时钟周期ticks。
 */
template <typename PrecisionDuration>
int64_t AsInt64(const Duration &duration) {
  return std::chrono::duration_cast<PrecisionDuration>(duration).count();
}

/**
 * @brief converts the input timestamp (nanos) to a 64 bit integer, with
 * the unit specified by PrecisionDuration.
 * @param timestamp the input timestamp that needs to be converted to integer.
 * @return an integer representing the timestamp in the specified unit.
  
  将Timestamp(时间点)转换为64位整数表示。
  返回的是自1970以来的以PrecisionDuration计的int64整数，如自1970年以来的s/ms/us/ns等。
 */
template <typename PrecisionDuration>
int64_t AsInt64(const Timestamp &timestamp) {
  return AsInt64<PrecisionDuration>(timestamp.time_since_epoch());
}

/**
 * @brief converts the input duration (nanos) to a double in seconds.
 * The original precision will be preserved.
 * @param duration the input duration that needs to be converted.
 * @return a doule in seconds.

 将纳秒ns转换为秒s。
 */
inline double ToSecond(const Duration &duration) {
  return static_cast<double>(AsInt64<nanos>(duration)) * 1e-9;
}

/**
 * @brief converts the input timestamp (nanos) to a double in seconds.
 * The original precision will be preserved.
 * @param timestamp the input timestamp that needs to be converted.
 * @return a doule representing the same timestamp in seconds.
 
将以纳秒表示的时间点ns转换为秒s
 */
inline double ToSecond(const Timestamp &timestamp) {
  return static_cast<double>(AsInt64<nanos>(timestamp.time_since_epoch())) *
         1e-9;
}

/**
 * @brief converts the integer-represented timestamp to \class Timestamp.
 * @return a Timestamp object.

 将以PrecisionDuration为精度计的int64整数转换为Timestamp(时间点)。
 */
template <typename PrecisionDuration>
inline Timestamp FromInt64(int64_t timestamp_value) {
  return Timestamp(
      std::chrono::duration_cast<nanos>(PrecisionDuration(timestamp_value)));
}

/**
 * @brief converts the double to \class Timestamp. The input double has
 * a unit of seconds.
 * @return a Timestamp object.
  
  将秒s转换为时间点Timestamp。
  即FromInt64的特化版本：先将时间转换为ns计的64位整数nanos_value，再转换为Timestamp。
*/

inline Timestamp From(double timestamp_value) {
  int64_t nanos_value = static_cast<int64_t>(timestamp_value * 1e9);
  return FromInt64<nanos>(nanos_value);
}

/**
 * @class Clock
 * @brief a singleton clock that can be used to get the current current
 * timestamp. The source can be either system clock or a mock clock.
 * Mock clock is for testing purpose mainly. The mock clock related
 * methods are not thread-safe.

Clock是封装c++ 11chrono后抽象的时钟计时类class。

线程安全的单例模式(c++ 11的语法可确保)。

数据成员:

bool is_system_clock_; true表示系统时间，false表示模拟时间
Timestamp mock_now_; 模拟时间的当前值。

为啥要标记模拟时间?:为了仿真训练。多次仿真，反复训练。

Clock类没有公有的构造函数。私有构造函数初始化为使用cpu系统时间。
提供4个static 公有函数:

static Timestamp Now() 
static void UseSystemClock(bool is_system_clock) 
static bool IsSystemClock()
static void SetNow(const Duration &duration)

 */
class Clock {
 public:
  //num 和 den分别表示分子(numerator)和分母(denominator)。
  static constexpr int64_t PRECISION =
      std::chrono::system_clock::duration::period::den /
      std::chrono::system_clock::duration::period::num;

  /// PRECISION >= 1000000 means the precision is at least 1us.
  static_assert(PRECISION >= 1000000,
                "The precision of the system clock should be at least 1 "
                "microsecond.");

  /**
   * @brief gets the current timestamp.
   * @return a Timestamp object representing the current time.

   返回当前时间的最新值。
   */
  static Timestamp Now() {
    return instance()->is_system_clock_ ? SystemNow() : instance()->mock_now_;
  }

  /**
   * @brief Set the behavior of the \class Clock.
   * @param is_system_clock if provided with value TRUE, further call
   * to Now() will return timestamp based on the system clock. If
   * provided with FALSE, it will use the mock clock instead.

   设置是否使用模拟时间。

   */
  static void UseSystemClock(bool is_system_clock) {
    instance()->is_system_clock_ = is_system_clock;
  }

  /**
   * @brief Check whether the \class Clock instance is using system clock.
   * @return TRUE if it is using system clock, and FALSE otherwise.

    返回是否使用系统时间。
   */
  static bool IsSystemClock() { return instance()->is_system_clock_; }

  /**
   * @brief This is for mock clock mode only. It will set the timestamp
   * for the mock clock.

   当Clock使用mock时间时，将系统时间设定为给定值。
   否则抛出运行时错误（只有模拟时间才可调整值，系统时间不可调整）
   */
  static void SetNow(const Duration &duration) {
    Clock *clock = instance();
    if (clock->is_system_clock_) {
      throw std::runtime_error("Cannot set now when using system clock!");
    }
    clock->mock_now_ = Timestamp(duration);
  }

 private:
  /**
   * @brief constructs the \class Clock instance
   * @param is_system_clock See UseSystemClock.
   */
  explicit Clock(bool is_system_clock)
      : is_system_clock_(is_system_clock), mock_now_(Timestamp()) {}

  /**
   * @brief Returns the current timestamp based on the system clock.
   * @return the current timestamp based on the system clock.

   返回以纳秒ns计的当前系统时间。
   */
  static Timestamp SystemNow() {
    return std::chrono::time_point_cast<Duration>(
        std::chrono::system_clock::now());
  }

  /// NOTE: Unless is_system_clock_ and mock_now_ are guarded by a
  /// lock or become atomic, having multiple threads calling mock
  /// clock related functions are STRICTLY PROHIBITED.

  /// Indicates whether it is in the system clock mode or the mock
  /// clock mode.
  //true表示系统时间，false表示模拟时间
  bool is_system_clock_;

  /// Stores the currently set timestamp, which serves mock clock
  /// queries.
  /// 模拟时间的当前值
  Timestamp mock_now_;

  /// Explicitly disable default and move/copy constructors.
  ///禁止移动/拷贝/赋值运算符
  DECLARE_SINGLETON(Clock);
};

inline Clock::Clock() : Clock(true) {}

}  // namespace time
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_TIME_TIME_H_
//http://zh.cppreference.com/w/cpp/chrono/time_point
//http://zh.cppreference.com/w/cpp/chrono/duration

/*

std::chrono::system_clock 表示当前的系统时钟，系统中运行的所有进程使用now()得到的时间是一致的。

稳定的时间间隔，后一次调用now()得到的时间总是比前一次的值大
（这句话的意思其实是，如果中途修改了系统时间，也不影响now()的结果），每次tick都保证过了稳定的时间间隔。
*/