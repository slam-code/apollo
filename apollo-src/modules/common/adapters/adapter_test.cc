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

#include "modules/common/adapters/adapter.h"

#include <string>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace common {
namespace adapter {

using IntegerAdapter = Adapter<int>;

TEST(AdapterTest, Empty) {
  //创建一个IntegerAdapter对象，监听integer_topic话题，保存历史消息数量是10
  IntegerAdapter对象， adapter("Integer", "integer_topic", 10);
  EXPECT_TRUE(adapter.Empty());
}

TEST(AdapterTest, Observe) {
  IntegerAdapter adapter("Integer", "integer_topic", 10);
  adapter.OnReceive(173);//接收到了一个数据，但是尚未处理数据。

  // Before calling Observe.
  EXPECT_TRUE(adapter.Empty());

  // After calling Observe.
  //进行一次观测，即接收待处理数据。
  adapter.Observe();
  EXPECT_FALSE(adapter.Empty());
}

TEST(AdapterTest, GetLatestObserved) {
  IntegerAdapter adapter("Integer", "integer_topic", 10);
  adapter.OnReceive(173);

  adapter.Observe();
  EXPECT_FALSE(adapter.Empty());
  EXPECT_EQ(173, adapter.GetLatestObserved());//最新数据是173

  adapter.OnReceive(5);//接收5
  adapter.OnReceive(7);//接收7
  // Before calling Observe() again.
  EXPECT_FALSE(adapter.Empty());
  EXPECT_EQ(173, adapter.GetLatestObserved());//只接收，尚未观测
  adapter.Observe();
  // After calling Observe() again.
  EXPECT_FALSE(adapter.Empty());
  EXPECT_EQ(7, adapter.GetLatestObserved());//观测，返回最新数据
}

TEST(AdapterTest, History) {
  IntegerAdapter adapter("Integer", "integer_topic", 3);//队列大小是3
  adapter.OnReceive(1);
  adapter.OnReceive(2);

  adapter.Observe();
  {
    // Currently the history contains [2, 1].
    std::vector<std::shared_ptr<int>> history(adapter.begin(), adapter.end());
    EXPECT_EQ(2, history.size());
    EXPECT_EQ(2, *history[0]);
    EXPECT_EQ(1, *history[1]);//历史数据排列：[2,1]
  }

  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(3);
  adapter.OnReceive(4);
  adapter.OnReceive(5);

  {
    // Although there are more messages, without calling Observe,
    // the history is still [2, 1].
    std::vector<std::shared_ptr<int>> history(adapter.begin(), adapter.end());
    EXPECT_EQ(2, history.size());//data只是接收，没有进行观测，所以size()大小是2
    EXPECT_EQ(2, *history[0]);
    EXPECT_EQ(1, *history[1]);
  }

  adapter.Observe();
  {
    // After calling Observe(), the history starts from 5. Since we only
    // maintain 3 elements in this adapter, 1 and 2 will be thrown out.
    //
    // History should be 5, 4, 3.
    std::vector<std::shared_ptr<int>> history(adapter.begin(), adapter.end());
    EXPECT_EQ(3, history.size());//进行了一次观测，size()==3.data-list排列:[5,4,3]
    EXPECT_EQ(5, *history[0]);
    EXPECT_EQ(4, *history[1]);
    EXPECT_EQ(3, *history[2]);
  }
}

TEST(AdapterTest, Callback) {
  IntegerAdapter adapter("Integer", "integer_topic", 3);

  // Set the callback to act as a counter of messages.
  int count = 0;
  adapter.SetCallback([&count](int x) { count += x; });

  adapter.OnReceive(11);
  adapter.OnReceive(41);
  adapter.OnReceive(31);
  EXPECT_EQ(11 + 41 + 31, count);//接收一次消息就回调一次函数。
}

using MyLocalizationAdapter = Adapter<localization::LocalizationEstimate>;

TEST(AdapterTest, Dump) {
  FLAGS_enable_adapter_dump = true;
  std::string temp_dir = std::getenv("TEST_TMPDIR");//dump输出路径

  MyLocalizationAdapter adapter("local", "local_topic", 3, temp_dir);
  localization::LocalizationEstimate loaded;

  localization::LocalizationEstimate msg;

  msg.mutable_header()->set_sequence_num(17);
  adapter.OnReceive(msg);//接收时，即已经写入dump文件。

  //从dump文件读取信息到loaded中。
  apollo::common::util::GetProtoFromASCIIFile(temp_dir + "/local/17.pb.txt",
                                              &loaded);
  EXPECT_EQ(17, loaded.header().sequence_num());

  msg.mutable_header()->set_sequence_num(23);
  adapter.OnReceive(msg);
  //同上。
  apollo::common::util::GetProtoFromASCIIFile(temp_dir + "/local/23.pb.txt",
                                              &loaded);
  EXPECT_EQ(23, loaded.header().sequence_num());
}

}  // namespace adapter
}  // namespace common
}  // namespace apollo
