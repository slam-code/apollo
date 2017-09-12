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

#ifndef MODULES_PLATFORM_ANNOTATIONS_H_
#define MODULES_PLATFORM_ANNOTATIONS_H_

//禁止拷贝构造和赋值,common模块也有。

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName &);              \
  TypeName &operator=(const TypeName &)

/// Indicating a pointer class member is not owned by an object of the class; it
/// is the
/// responsibility of the programmer to make sure the given pointer is valid
/// during the life time
/// of the object.

///表示c++ 智能指针的特性。　
///某一class对象不拥有该智能指针，即不管理该指针的生命周期。在该class存续区间，指针必须有效
#define PTR_NOT_OWNED

/// Indicating a pointer function argument is used by the object of the function
/// for the lifetime
/// of the object; it is the responsibility of the programmer to make sure the
/// given pointer
/// is valid during the life time of the object.

///函数参数。在调用期间，指针必须有效  
#define PTR_LIFETIME

/// Indicating ownership of a pointer argument is transferred to the callee.

///所有权转移
#define PTR_OWNER_XFR

/// Indicating a pointer function argument is used only in the scope of this
/// function,
/// will not be used after this function is done (e.g., not saved for future
/// use).
/// This is the default behavior of any pointer argument of a function.

///  只在该函数范围内调用，而不能通过return 返回值等方式传离函数作用域。
#define PTR_USE_ONCE

/// Indicating access to the given (data) member is lock-protected by the given
/// lock.

///上锁保护临界区  
#define XLOCK_BY(lock)

/// Indicating this function will acquire the given exclusive lock(s).

///上锁
#define ACQ_LOCK(locks...)

/// Indicating this function should only be called when the given exclusive
/// lock(s) is/are locked.

///必须上锁保护临界区
#define WITH_LOCK(locks...)

///调用必须是线程安全的
#define THREAD_SAFE

/// Indicating private global variable that is implementation specific and
/// should not be used
/// directly.

///不能直接使用的全局变量。只能通过函数等方式间接访问
#define PRIVATE

#endif  // MODULES_PLATFORM_ANNOTATIONS_H_
