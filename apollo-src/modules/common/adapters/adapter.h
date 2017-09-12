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

#ifndef MODULES_ADAPTERS_ADAPTER_H_
#define MODULES_ADAPTERS_ADAPTER_H_

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>

#include "glog/logging.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/proto/header.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"

/**
 * @namespace apollo::common::adapter
 * @brief apollo::common::adapter
 */
namespace apollo {
namespace common {
namespace adapter {

/**
 * @class Adapter
 * @brief this class serves as the interface and a layer of
 * abstraction for Apollo modules to interact with various I/O (e.g.
 * ROS). The adapter will also store history data, so that other
 * Apollo modules can get access to both the current and the past data
 * without having to handle communication protocols directly.
 *
 * \par
 * Each \class Adapter instance only works with one single topic and
 * its corresponding data type.
 *
 * \par
 * Under the hood, a queue is used to store the current and historical
 * messages. In most cases, the underlying data type is a proto, though
 * this is not necessary.
 *
 * \note
 * Adapter::Observe() is thread-safe, but calling it from
 * multiple threads may introduce unexpected behavior. Adapter is
 * thread-safe w.r.t. data access and update.


Adapter是来自传感器的底层数据和Apollo各个模块交互的统一接口。(c++ 适配器模式)
使得Apollo各个模块不强依赖于ROS框架:将数据IO抽象。上层是Apollo框架，底层是data IO，二者松耦合。
Adapter持有一段时间内的所有历史数据，因此所有的模块可获取这些数据而无需使用ROS通信机制进行数据交换(提高通信效率)。

1) 构造函数初始化成员变量。 
Adapter(const std::string &adapter_name, const std::string &topic_name,
          size_t message_num, const std::string &dump_dir = "/tmp")
指定Adapter的名称/自己要监听的ros_topic/历史消息最大数量/dump路径

2） FeedProtoFile()从proto文件读取data。

3)  OnReceive(const D &message)  接收原始数据data。并调用相关回调函数。

4)  void Observe() 进行一次观察/测量。只有调用了Observe()，接收的数据才能被有效处理。


5)  const D &GetLatestObserved()获取观测集中的最新值 

6)  begin(),end()，迭代器模式指向观测数据集。支持for循环。

7)  SetCallback() ,消息回调函数。


模板参数可以是message消息类型，不一定是传感器数据。

 */
template <typename D>
class Adapter {
 public:
  /// The user can use Adapter::DataType to get the type of the
  /// underlying data.
  typedef D DataType;

  typedef typename std::list<std::shared_ptr<D>>::const_iterator Iterator;
  typedef typename std::function<void(const D &)> Callback;

  /**
   * @brief Construct the \class Adapter object.
   * @param adapter_name the name of the adapter. It is used to log
   * error messages when something bad happens, to help people get an
   * idea which adapter goes wrong.
   * @param topic_name the topic that the adapter listens to.
   * @param message_num the number of historical messages that the
   * adapter stores. Older messages will be removed upon calls to
   * Adapter::OnReceive().

构造函数参数：
1，Adapter标识自己的名称，
2，监听的ros_topic，
3，保存历史消息的最大数量，
4，dump路径
   */
  Adapter(const std::string &adapter_name, const std::string &topic_name,
          size_t message_num, const std::string &dump_dir = "/tmp")
      : topic_name_(topic_name),
        message_num_(message_num),
        enable_dump_(FLAGS_enable_adapter_dump && HasSequenceNumber<D>()),
        dump_path_(enable_dump_ ? dump_dir + "/" + adapter_name : "") {
    if (enable_dump_) {
      if (!apollo::common::util::EnsureDirectory(dump_path_)) {
        AERROR << "Cannot enable dumping for '" << adapter_name
               << "' adapter because the path " << dump_path_
               << " cannot be created or is not a directory.";
        enable_dump_ = false;
      } else if (!apollo::common::util::RemoveAllFiles(dump_path_)) {
        AERROR << "Cannot enable dumping for '" << adapter_name
               << "' adapter because the path " << dump_path_
               << " contains files that cannot be removed.";
        enable_dump_ = false;
      }
    }
  }

  /**
   * @brief returns the topic name that this adapter listens to.
   */
  const std::string &topic_name() const { return topic_name_; }

  /**
   * @brief reads the proto message from the file, and push it into
   * the adapter's data queue.
   * @param message_file the path to the file that contains a (usually
   * proto) message of DataType.

   从proto文件读取数据data。
   */
  void FeedProtoFile(const std::string &message_file) {
    D data;
    CHECK(apollo::common::util::GetProtoFromFile(message_file, &data))
        << "Unable to parse input pb file " << message_file;
    FeedProto(data);
  }

  /**
   * @brief push (a copy of) the input data into the data queue of
   * the adapter.
   * @param data the input data.
   */
  void FeedProto(const D &data) {
    auto data_ptr = std::make_shared<D>(data);//值拷贝
    EnqueueData(data_ptr);                    //入队列。
  }

  /**
   * @brief the callback that will be invoked whenever a new
   * message is received.
   * @param message the newly received message.

   接收原始数据data。并调用相关回调函数。

   */
  void OnReceive(const D &message) {
    auto data_ptr = std::make_shared<D>(message);

    EnqueueData(data_ptr);
    FireCallback(message);
  }

  /**
   * @brief copy the data_queue_ into the observing queue to create a
   * view of data up to the call time for the user.
   进行一次观察/测量。只有调用了Observe()，接收的数据才能被处理。
   */
  void Observe() {
    std::lock_guard<std::mutex> lock(mutex_);
    observed_queue_ = data_queue_;//标记data_queue中的数据将要进行处理。
  }

  /**
   * @brief returns TRUE if the observing queue is empty.
   */
  bool Empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return observed_queue_.empty();
  }

  /**
   * @brief returns the most recent message in the observing queue.
   *
   * /note
   * Please call Empty() to make sure that there is data in the
   * queue before calling GetOldestObserved().

    返回最新的数据
   */
  const D &GetLatestObserved() const {
    std::lock_guard<std::mutex> lock(mutex_);
    DCHECK(!observed_queue_.empty())
        << "The view of data queue is empty. No data is received yet or you "
           "forgot to call Observe()"
        << ":" << topic_name_;
    return *observed_queue_.front();// 返回最新数据(队首)
  }

  /**
   * @brief returns the oldest message in the observing queue.
   *
   * /note
   * Please call Empty() to make sure that there is data in the
   * queue before calling GetOldestObserved().
   */
  const D &GetOldestObserved() const {
    std::lock_guard<std::mutex> lock(mutex_);
    DCHECK(!observed_queue_.empty())
        << "The view of data queue is empty. No data is received yet or you "
           "forgot to call Observe().";
    return *observed_queue_.back();//(队尾)是oldest元素。
  }

  /**
   * @brief returns an iterator representing the head of the observing
   * queue. The caller can use it to iterate over the observed data
   * from the head. The API also supports range based for loop.
   已观测的消息首迭代器
   */
  Iterator begin() const { return observed_queue_.begin(); }

  /**
   * @brief returns an iterator representing the tail of the observing
   * queue. The caller can use it to iterate over the observed data
   * from the head. The API also supports range based for loop.
   已观测的消息尾迭代器
   */
  Iterator end() const { return observed_queue_.end(); }

  /**
   * @brief registers the provided callback function to the adapter,
   * so that the callback function will be called once right after the
   * message hits the adapter.
   * @param callback the callback with signature void(const D &).
   */
  void SetCallback(Callback callback) { receive_callback_ = callback; }

  /**
   * @brief fills the fields module_name, timestamp_sec and
   * sequence_num in the header.

   添加head信息。
   */
  void FillHeader(const std::string &module_name,
                  apollo::common::Header *header) {
    double timestamp =
        apollo::common::time::ToSecond(apollo::common::time::Clock::Now());
    header->set_module_name(module_name);
    header->set_timestamp_sec(timestamp);
    header->set_sequence_num(++seq_num_);
  }

 private:
  // HasSequenceNumber returns false for non-proto-message data types.
  template <typename InputMessageType>
  static bool HasSequenceNumber(
      typename std::enable_if<
          !std::is_base_of<google::protobuf::Message, InputMessageType>::value,
          InputMessageType>::type *message = nullptr) {
    return false;
  }

  // HasSequenceNumber returns true if the data type is proto and has
  // header.sequence_num.
  template <typename InputMessageType>
  static bool HasSequenceNumber(
      typename std::enable_if<
          std::is_base_of<google::protobuf::Message, InputMessageType>::value,
          InputMessageType>::type *message = nullptr) {
    using gpf = google::protobuf::FieldDescriptor;
    InputMessageType sample;
    auto descriptor = sample.GetDescriptor();
    auto header_descriptor = descriptor->FindFieldByName("header");
    if (header_descriptor == nullptr) {
      return false;
    }
    if (header_descriptor->cpp_type() != gpf::CPPTYPE_MESSAGE) {
      return false;
    }
    auto sequence_num_descriptor =
        header_descriptor->message_type()->FindFieldByName("sequence_num");
    if (sequence_num_descriptor == nullptr) {
      return false;
    }
    if (sequence_num_descriptor->cpp_type() != gpf::CPPTYPE_UINT32) {
      return false;
    }
    return true;
  }

  // DumpMessage does nothing for non proto message data type.
  template <typename InputMessageType>
  bool DumpMessage(
      const typename std::enable_if<
          !std::is_base_of<google::protobuf::Message, InputMessageType>::value,
          InputMessageType>::type &message) {
    return true;
  }

  // DumpMessage dumps the message to a file to
  // /tmp/<adapter_name>/<name>.pb.txt, where the message is in ASCII
  // mode and <name> is the .header().sequence_num() of the message.
  //将消息message写入路径为/tmp/<adapter_name>/<name>.pb.txt的文件。
  template <typename InputMessageType>
  bool DumpMessage(
      const typename std::enable_if<
          std::is_base_of<google::protobuf::Message, InputMessageType>::value,
          InputMessageType>::type &message) {
    using google::protobuf::Message;
    auto descriptor = message.GetDescriptor();
    auto header_descriptor = descriptor->FindFieldByName("header");
    if (header_descriptor == nullptr) {
      ADEBUG << "Fail to find header field in pb.";
      return false;
    }
    const Message &header = message.GetReflection()->GetMessage(
        *static_cast<const Message *>(&message), header_descriptor);
    auto seq_num_descriptor =
        header.GetDescriptor()->FindFieldByName("sequence_num");
    if (seq_num_descriptor == nullptr) {
      ADEBUG << "Fail to find sequence_num field in pb.";
      return false;
    }
    uint32_t sequence_num =
        header.GetReflection()->GetUInt32(header, seq_num_descriptor);
    return apollo::common::util::SetProtoToASCIIFile(
        message, dump_path_ + "/" + std::to_string(sequence_num) + ".pb.txt");
  }

  /**
   * @brief proactively invokes the callback with the specified data.
   * @param data the specified data.
   */
  void FireCallback(const D &data) { 
    if (receive_callback_ != nullptr) {
      receive_callback_(data);//对到来的数据调用回调函数。
    }
  }

  /**
   * @brief push the shared-pointer-guarded data to the data queue of
   * the adapter.

   将指向Data的智能指针压入队列。(队首)
   */
  void EnqueueData(std::shared_ptr<D> data_ptr) {
    if (enable_dump_) {
      DumpMessage<D>(*data_ptr);
    }

    // Lock the queue.
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_queue_.size() + 1 > message_num_) { //已满则删除（队尾元素）
      data_queue_.pop_back();
    }
    data_queue_.push_front(data_ptr);//(队首总是最新数据)
  }

  /// The topic name that the adapter listens to.
  /// 需要监听的ROS的话题名称，message的来源topic
  std::string topic_name_;

  /// The maximum size of data_queue_ and observed_queue_
  /// 数据队列的最大容量
  size_t message_num_ = 0;

  /// The received data. Its size is no more than message_num_
  //  接收的数据队列，原始数据。该部分数据尚未进行观测
  std::list<std::shared_ptr<D>> data_queue_;

  /// It is the snapshot of the data queue. The snapshot is taken when
  /// Observe() is called.
  /// 备份已经观测过的数据。
  std::list<std::shared_ptr<D>> observed_queue_;


  /// User defined function when receiving a message
  /// 消息到达的回调函数（用于处理到达的消息）
  Callback receive_callback_ = nullptr;

  /// The mutex guarding data_queue_ and observed_queue_
  ///互斥锁保证线程安全
  mutable std::mutex mutex_;

  /// Whether dumping is enabled.
  /// 是否输出到dump路径
  bool enable_dump_ = false;

  /// The directory of dumped files.
  /// dump输出路径
  std::string dump_path_;

  /// The monotonically increasing sequence number of the message to
  /// be published.
  /// 消息的序列号，单调递增。此值随着消息的增加而单调增加。
  uint32_t seq_num_ = 0;
};

}  // namespace adapter
}  // namespace common
}  // namespace apollo

#endif  // MODULES_ADAPTERS_ADAPTER_H_
