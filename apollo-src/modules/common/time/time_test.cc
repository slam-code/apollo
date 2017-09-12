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

#include "modules/common/time/time.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace time {

TEST(TimeTest, DurationToMicros) {
  Duration duration = std::chrono::milliseconds(12); //12ms 
  EXPECT_EQ(12000, AsInt64<micros>(duration));      //==121000us
}

TEST(TimeTest, DurationToMillis) {
  Duration duration = std::chrono::microseconds(1234567);//1234567us
  EXPECT_EQ(1234, AsInt64<millis>(duration));            //==1234ms
}

TEST(TimeTest, AsDouble) {
  Duration duration = std::chrono::microseconds(123 456 789 012);//123...us
  EXPECT_FLOAT_EQ(123456.789012, ToSecond(duration));         //123456(s)
}

TEST(TimeTest, TimestampAsDouble) {
  Timestamp timestamp = FromInt64<nanos>(123 456 789 012 345);    //ns
  EXPECT_FLOAT_EQ(123456.789012345, ToSecond(timestamp));         //s
}

TEST(TimeTest, TimestampFromAndTo) {
  Timestamp timestamp = FromInt64<micros>(1234567);// us
  EXPECT_EQ(1234, AsInt64<millis>(timestamp));     //ms
}

TEST(TimeTest, TimestampFromAndToDouble) {
  Timestamp timestamp = From(1234567.889923456);            //s
  EXPECT_FLOAT_EQ(1234567.889923456, ToSecond(timestamp));  // s
}

TEST(TimeTest, MockTime) {
  EXPECT_TRUE(Clock::IsSystemClock()); //默认是cpu系统时间
  Clock::UseSystemClock(false);        //修改。
  EXPECT_FALSE(Clock::IsSystemClock());

  EXPECT_EQ(0, AsInt64<micros>(Clock::Now())); //模拟时间的初值是0.
  Clock::SetNow(micros(123));                  //修改为123 (us)

  EXPECT_EQ(123, AsInt64<micros>(Clock::Now()));
}

}  // namespace time
}  // namespace common
}  // namespace apollo
