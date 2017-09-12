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
 
宏定义DISALLOW_COPY_AND_ASSIGN：

    用于在C++中禁止class的拷贝构造函数和赋值构造函数，良好的c++代码应该主动管理这2个操作符。
    在caffe和cartographer或者其他的著名库中均有类似的操作。
 
宏定义DISALLOW_IMPLICIT_CONSTRUCTORS:

    禁止class的无参构造函数。

宏定义DECLARE_SINGLETON：
    单例类定义，instance() 返回指向同一个class对象的指针。
    禁止拷贝/赋值运算符。

 */

#ifndef MODULES_COMMON_MACRO_H_
#define MODULES_COMMON_MACRO_H_

#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);

#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
  classname();                                    \
  DISALLOW_COPY_AND_ASSIGN(classname);

#define DECLARE_SINGLETON(classname)        \
 public:                                    \
  static classname *instance() {            \
    static classname instance;              \
    return &instance;                       \
  }                                         \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:
#endif  // MODULES_COMMON_MACRO_H_
