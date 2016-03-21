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

#include "ros/timer.h"
#include "ros/timer_manager.h"

namespace ros
{

Timer::Impl::Impl()
  : started_(false)
  , timer_handle_(-1)
{ }

Timer::Impl::~Impl()
{
  ROS_DEBUG("Timer deregistering callbacks.");  //定时器注销回调
  stop();
}

bool Timer::Impl::isValid()
{
  return !period_.isZero(); //时间为空
}

/**
 * \brief 开始定时器.如果定时器已经开始了就什么都不做.
 */
void Timer::Impl::start()
{
  if (!started_)    //started是个flag
  {
    VoidConstPtr tracked_object;
    if (has_tracked_object_)    //追踪到目标
    {
      tracked_object = tracked_object_.lock();  //追踪到目标,上锁
    }

    timer_handle_ = TimerManager<Time, Duration, TimerEvent>::global().add(period_, callback_, callback_queue_, tracked_object, oneshot_);
    started_ = true;    //开始,TimeManager::global().add
  }
}
/**
 * \brief 停止定时器. 一旦回调函数返回了,就不会再有回调被调用.如果定时器已经停止了就什么都不做.
 */
void Timer::Impl::stop()
{
  if (started_)
  {
    started_ = false;
    TimerManager<Time, Duration, TimerEvent>::global().remove(timer_handle_);
    timer_handle_ = -1;     //开始,TimeManager::global().remove
  }
}
/**
 * \brief 无论定时器是否已经被挂起事件被挂起都返回Returns whether or not the timer has any pending events to call.
 */
bool Timer::Impl::hasPending()  //挂起
{
  if (!isValid() || timer_handle_ == -1)
  {
    return false;
  }

  return TimerManager<Time, Duration, TimerEvent>::global().hasPending(timer_handle_);
}
/**
 * \brief 设置该定时器时间段
 * \param 复位 重置计时器.如果是,定时器会忽略已经走过的时间,下一个回调发生的事件是now()+period
 */
void Timer::Impl::setPeriod(const Duration& period, bool reset)
{
  period_ = period;
  TimerManager<Time, Duration, TimerEvent>::global().setPeriod(timer_handle_, period, reset);
}

Timer::Timer(const TimerOptions& ops)
: impl_(new Impl)
{
  impl_->period_ = ops.period;
  impl_->callback_ = ops.callback;
  impl_->callback_queue_ = ops.callback_queue;
  impl_->tracked_object_ = ops.tracked_object;
  impl_->has_tracked_object_ = (ops.tracked_object != NULL);
  impl_->oneshot_ = ops.oneshot;
}

Timer::Timer(const Timer& rhs)
{
  impl_ = rhs.impl_;
}

Timer::~Timer()
{
}

void Timer::start()
{
  if (impl_)
  {
    impl_->start();
  }
}

void Timer::stop()
{
  if (impl_)
  {
    impl_->stop();
  }
}

bool Timer::hasPending()
{
  if (impl_)
  {
    return impl_->hasPending();
  }

  return false;
}

void Timer::setPeriod(const Duration& period, bool reset)
{
  if (impl_)
  {
    impl_->setPeriod(period, reset);
  }
}

}
