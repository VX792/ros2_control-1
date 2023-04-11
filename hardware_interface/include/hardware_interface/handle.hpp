// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HARDWARE_INTERFACE__HANDLE_HPP_
#define HARDWARE_INTERFACE__HANDLE_HPP_

#include <atomic>
#include <string>
#include <thread>
#include <utility>
#include <iostream>

#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class CommandInterfaceLock
{
public:
    CommandInterfaceLock() = default;
    CommandInterfaceLock(const CommandInterfaceLock & other) = delete;
    CommandInterfaceLock(CommandInterfaceLock && other) = delete;

    void lock()
    {
        if (!is_locked_.load(std::memory_order_acquire))
        {
            is_locked_.store(true, std::memory_order_relaxed);
            active_worker_.store(std::this_thread::get_id(), std::memory_order_relaxed);
            std::cout << "[................] [Handle]: LOCK SUCCESSFUL FROM ID: " << active_worker_ << std::endl;

        } else {
            std::cout << "[................] [Handle]: LOCK FAILED FROM ID: " << std::this_thread::get_id() << std::endl;
        }
    }

    void release()
    {
        if (owns_lock())
        {
            std::cout << "[................] [Handle]: RELEASE SUCCESSFUL FROM ID: " << std::this_thread::get_id() << std::endl;
            is_locked_.store(false, std::memory_order_release);
        } else {
            std::cout << "[................] [Handle]: RELEASE UNSUCCESSFUL FROM ID: " << std::this_thread::get_id() << ", LOCK OWNED BY: " << active_worker_ << std::endl;
        }
    }

    bool owns_lock() {
      return is_locked_.load(std::memory_order_relaxed) 
              && std::this_thread::get_id() == active_worker_.load(std::memory_order_relaxed);
    }
    ~CommandInterfaceLock() = default;

private:
    
    // Checks if the lock is acquired
    std::atomic<bool> is_locked_{false};
    
    // Thread id of the process owning the lock
    std::atomic<std::thread::id> active_worker_;
    
    friend class ReadOnlyHandle;
    friend class ReadWriteHandle;
};

/// A handle used to get and set a value on a given interface.
class ReadOnlyHandle
{
public:
  ReadOnlyHandle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  explicit ReadOnlyHandle(const std::string & interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  explicit ReadOnlyHandle(const char * interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  ReadOnlyHandle(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle(ReadOnlyHandle && other) = default;

  ReadOnlyHandle & operator=(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle & operator=(ReadOnlyHandle && other) = default;

  virtual ~ReadOnlyHandle() = default;

  /// Returns true if handle references a value.
  inline operator bool() const { return value_ptr_ != nullptr; }

  const std::string get_name() const { return prefix_name_ + "/" + interface_name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  const std::string & get_prefix_name() const { return prefix_name_; }

  double get_value()
  {
    if (interface_lock_ptr_ == nullptr 
        || interface_lock_ptr_->owns_lock())
    {
      if (interface_lock_ptr_ != nullptr && interface_lock_ptr_->is_locked_.load(std::memory_order_relaxed))
      {
        std::cout << "[................] [Handle]: GET VALUE: LOCK OWNED BY ID: " << interface_lock_ptr_->active_worker_ << std::endl;
      } else {
        std::cout << "[................] [Handle]: GET VALUE: LOCK OWNED BY ID: NOT ASYNC" << std::endl;
      }
      THROW_ON_NULLPTR(value_ptr_);
      previous_value_ = *value_ptr_;
      return *value_ptr_;
    }

    return previous_value_;
  }

  void set_interface_lock(CommandInterfaceLock* lock_)
  {
    interface_lock_ptr_ = lock_;
  }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double * value_ptr_;

  double previous_value_{0};
  CommandInterfaceLock* interface_lock_ptr_{nullptr};
};

class ReadWriteHandle : public ReadOnlyHandle
{
public:
  ReadWriteHandle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : ReadOnlyHandle(prefix_name, interface_name, value_ptr)
  {
  }

  explicit ReadWriteHandle(const std::string & interface_name) : ReadOnlyHandle(interface_name) {}

  explicit ReadWriteHandle(const char * interface_name) : ReadOnlyHandle(interface_name) {}

  ReadWriteHandle(const ReadWriteHandle & other) = default;

  ReadWriteHandle(ReadWriteHandle && other) = default;

  ReadWriteHandle & operator=(const ReadWriteHandle & other) = default;

  ReadWriteHandle & operator=(ReadWriteHandle && other) = default;

  virtual ~ReadWriteHandle() = default;

  void set_value(double value)
  {
    if (interface_lock_ptr_ == nullptr
        || interface_lock_ptr_->owns_lock())
    {
      if (interface_lock_ptr_ != nullptr && interface_lock_ptr_->is_locked_.load(std::memory_order_relaxed))
      {
        std::cout << "[................] [Handle]: SET VALUE: LOCK OWNED ID: " << interface_lock_ptr_->active_worker_ << std::endl;
      } else {
        std::cout << "[................] [Handle]: SET VALUE: LOCK OWNED BY ID: NOT ASYNC" << std::endl;
      }
      THROW_ON_NULLPTR(this->value_ptr_);
      *this->value_ptr_ = value;
    }
    else
    {
      *this->value_ptr_ = previous_value_;
    }    
  }
};

class StateInterface : public ReadOnlyHandle
{
public:
  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using ReadOnlyHandle::ReadOnlyHandle;
};

class CommandInterface : public ReadWriteHandle
{
public:
  /// CommandInterface copy constructor is actively deleted.
  /**
   * Command interfaces are having a unique ownership and thus
   * can't be copied in order to avoid simultaneous writes to
   * the same resource.
   */
  CommandInterface(const CommandInterface & other) = delete;

  CommandInterface(CommandInterface && other) = default;

  using ReadWriteHandle::ReadWriteHandle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
