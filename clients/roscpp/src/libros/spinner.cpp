/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/spinner.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace {
  boost::recursive_mutex spinmutex;     //recursive_mutex比起mutex，最主要的好处就是降低了死锁的可能性。可重入的互斥锁
}//recursive_mutex是不能重入的.一个线程只能用它锁一次.否则会死锁

namespace ros
{

//单线程自旋
void SingleThreadedSpinner::spin(CallbackQueue* queue)
{
  boost::recursive_mutex::scoped_try_lock spinlock(spinmutex);//加锁
  if(!spinlock.owns_lock()) {//用于探测unique_lock是否管理着一个互斥锁且其处于上锁状态。bool operate bool() 与owns_lock等同。
    ROS_ERROR("SingleThreadedSpinner: You've attempted to call spin "
              "from multiple threads.  Use a MultiThreadedSpinner instead.");
    return;
  }

  ros::WallDuration timeout(0.1f);  //设置超时时间为0.1

  if (!queue)   //如果回调队列不为空,读取全局回调队列
  {
    queue = getGlobalCallbackQueue();
  }
  /** nodehandle.ok() 检测是否是时候该退出了
   *
   * 这个方法是检测是否ros::ok()为真同时shutdown()没有被当前的NH调用,来看他是不是时候该退出了.
   *  一旦ros::shutdown()或者Nodehandle::shutdown()被调用时,ok()返回 false
   *
   */
  ros::NodeHandle n;
  while (n.ok())    //
  {
   /**
    *时间:可以指定一个在返回前等待回调可用的时间
    * \brief 调用队列中所有当前回调函数. 如果一个回调函数调用条件不满足,将会被重新放回回调队列中.
    */
    queue->callAvailable(timeout);
  }
}
/**
 * \brief 多线程自旋的自旋锁.
 * \param thread_count是调用回调函数所用的线程数 , 0将自动使用,当你的系统会有很多硬件线程
 */
MultiThreadedSpinner::MultiThreadedSpinner(uint32_t thread_count)//多线程自旋
: thread_count_(thread_count)
{
}
//多线程锁 ros::spin会自动调用多线程
void MultiThreadedSpinner::spin(CallbackQueue* queue)
{
  boost::recursive_mutex::scoped_try_lock spinlock(spinmutex);//加锁
  if (!spinlock.owns_lock()) {
    ROS_ERROR("MultiThreadeSpinner: You've attempted to call ros::spin "
              "from multiple threads... "
              "but this spinner is already multithreaded.");
    return;
  }
  AsyncSpinner s(thread_count_, queue);
  s.start();

  ros::waitForShutdown();
}

class AsyncSpinnerImpl
{
public:
  AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue);
  ~AsyncSpinnerImpl();

  bool canStart();
  void start();
  void stop();

private:
  void threadFunc();

  boost::mutex mutex_;
  boost::recursive_mutex::scoped_try_lock member_spinlock;
  boost::thread_group threads_;

  uint32_t thread_count_;
  CallbackQueue* callback_queue_;

  volatile bool continue_;

  ros::NodeHandle nh_;
};

AsyncSpinnerImpl::AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue)
: thread_count_(thread_count)
, callback_queue_(queue)
, continue_(false)
{
  if (thread_count == 0)
  {
    thread_count_ = boost::thread::hardware_concurrency();

    if (thread_count_ == 0)
    {
      thread_count_ = 1;
    }
  }

  if (!queue)
  {
    callback_queue_ = getGlobalCallbackQueue();
  }
}

AsyncSpinnerImpl::~AsyncSpinnerImpl()
{
  stop();
}

bool AsyncSpinnerImpl::canStart()
{
  boost::recursive_mutex::scoped_try_lock spinlock(spinmutex);
  return spinlock.owns_lock();
}

void AsyncSpinnerImpl::start()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (continue_)
    return;

  boost::recursive_mutex::scoped_try_lock spinlock(spinmutex);
  if (!spinlock.owns_lock()) {
    ROS_WARN("AsyncSpinnerImpl: Attempt to start() an AsyncSpinner failed "
             "because another AsyncSpinner is already running. Note that the "
             "other AsyncSpinner might not be using the same callback queue "
             "as this AsyncSpinner, in which case no callbacks in your "
             "callback queue will be serviced.");
    return;
  }
  spinlock.swap(member_spinlock);

  continue_ = true;

  for (uint32_t i = 0; i < thread_count_; ++i)
  {
    threads_.create_thread(boost::bind(&AsyncSpinnerImpl::threadFunc, this));
  }
}

void AsyncSpinnerImpl::stop()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!continue_)
    return;

  ROS_ASSERT_MSG(member_spinlock.owns_lock(), 
                 "Async spinner's member lock doesn't own the global spinlock, hrm.");
  ROS_ASSERT_MSG(member_spinlock.mutex() == &spinmutex, 
                 "Async spinner's member lock owns a lock on the wrong mutex?!?!?");
  member_spinlock.unlock();

  continue_ = false;
  threads_.join_all();
}

void AsyncSpinnerImpl::threadFunc()
{
  disableAllSignalsInThisThread();

  CallbackQueue* queue = callback_queue_;
  bool use_call_available = thread_count_ == 1;
  WallDuration timeout(0.1);

  while (continue_ && nh_.ok())
  {
    if (use_call_available)
    {
      queue->callAvailable(timeout);
    }
    else
    {
      queue->callOne(timeout);
    }
  }
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count)
: impl_(new AsyncSpinnerImpl(thread_count, 0))
{
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count, CallbackQueue* queue)
: impl_(new AsyncSpinnerImpl(thread_count, queue))
{
}

bool AsyncSpinner::canStart()
{
  return impl_->canStart();
}

void AsyncSpinner::start()
{
  impl_->start();
}

void AsyncSpinner::stop()
{
  impl_->stop();
}

}
