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

#ifndef MODULES_COMMON_UTIL_FILE_H_
#define MODULES_COMMON_UTIL_FILE_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <string>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util

file.h提供了多个关于操作文件(关于protobuf文件的读,写,删)的函数。

 */
namespace apollo {
namespace common {
namespace util {

/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        ascii representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.

 将message存储到filename中，文件内容设置为ascii格式存储。

 */
template <typename MessageType>
bool SetProtoToASCIIFile(const MessageType &message,
                         const std::string &file_name) {
  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::TextFormat;
  int file_descriptor = open(file_name.c_str(), O_WRONLY | O_CREAT, S_IRWXU);
  //打开文件，返回文件描述符fd

  if (file_descriptor < 0) {
    // Failed to open;
    return false;
  }
  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  //用fd创建输出流对象。

  bool success = TextFormat::Print(message, output);//将message输出到流中。
  delete output;
  close(file_descriptor);
  return success;
}

/**
 * @brief Parses the content of the file specified by the file_name as ascii
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.

 根据filename读取message信息，并存储到指针指向的地址空间中。
 */
template <typename MessageType>
bool GetProtoFromASCIIFile(const std::string &file_name, MessageType *message) {
  using google::protobuf::io::ZeroCopyInputStream;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::TextFormat;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);//打开file
  if (file_descriptor < 0) {
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  //创建输入流对象。

  bool success = TextFormat::Parse(input, message);//将文件内容存储到message中。
  delete input;
  close(file_descriptor);
  return success;
}

/**
 * @brief Parses the content of the file specified by the file_name as binary
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.

 解析二进制filename文件的内容，并存储到message中。

 */
template <typename MessageType>
bool GetProtoFromBinaryFile(const std::string &file_name,
                            MessageType *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  return message->ParseFromIstream(&input);
}

/**
 * @brief Parses the content of the file specified by the file_name as a
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.

 从filename中解析文件内容。filename可以是二进制或者ascii格式。
 */
template <typename MessageType>
bool GetProtoFromFile(const std::string &file_name, MessageType *message) {
  if (EndWith(file_name, ".bin")) { 
    //先以2进制解析，失败再以ascii解析。
    if (!GetProtoFromBinaryFile(file_name, message) &&
        !GetProtoFromASCIIFile(file_name, message)) {
      return false;
    }
  } else {
    if (!GetProtoFromASCIIFile(file_name, message) &&
        !GetProtoFromBinaryFile(file_name, message)) {
      return false;
    }
  }
  return true;
}
/**
 * @brief Check if the directory specified by directory_path exists
 *        and is indeed a directory.
 * @param directory_path Directory path.
 * @return If the directory specified by directory_path exists
 *         and is indeed a directory.

 给定的目录路径是否存在。
 */
bool DirectoryExists(const std::string &directory_path);

/**
 * @brief Check if a specified directory specified by directory_path exists.
 *        If not, recursively create the directory (and its parents).
 * @param directory_path Directory path.
 * @return If the directory does exist or its creation is successful.

 按照directory_path创建文件夹。类似于mkdir -p选项。
 */
bool EnsureDirectory(const std::string &directory_path);

/**
 * @brief Remove all the files under a specified directory. Note that
 *        sub-directories are NOT affected.
 * @param directory_path Directory path.
 * @return If the action is successful.

 删除给定目录下所有的文件。(文件夹不删除)
 */
bool RemoveAllFiles(const std::string &directory_path);

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_FILE_H_
