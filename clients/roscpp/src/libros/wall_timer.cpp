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

#include "ros/wall_timer.h"
#include "ros/timer_manager.h"
//Wall Time是如何获取到系统时间的?
/**
 * \brief 管理一个壁钟定时器回调
 *
 * 壁钟定时器是通过调用NodeHandle::createWallTimer()创建的, 或是从一个克隆过来的
 *一旦所有的壁钟定时器副本都指明已经超时, 这时与handle建立联系的回调函数将停止调用
 *
 */
namespace ros
{

WallTimer::Impl::Impl() //猜测它的作用是初始化,很多个文件中都有
  : started_(false)
  , timer_handle_(-1)
{ }

WallTimer::Impl::~Impl()
{
  ROS_DEBUG("WallTimer deregistering callbacks.");
  stop();
}

void WallTimer::Impl::start()   //开始
{
  if (!started_)
  {
    VoidConstPtr tracked_object;
    if (has_tracked_object_)
    {
      tracked_object = tracked_object_.lock();
    }
    timer_handle_ = TimerManager<WallTime, WallDuration, WallTimerEvent>::global().add(period_, callback_, callback_queue_, tracked_object, oneshot_);
    started_ = true;
  }
}

void WallTimer::Impl::stop()    //停止
{
  if (started_)
  {
    started_ = false;
    TimerManager<WallTime, WallDuration, WallTimerEvent>::global().remove(timer_handle_);
    timer_handle_ = -1;
  }
}

bool WallTimer::Impl::isValid() //是否为空
{
  return !period_.isZero();
}

bool WallTimer::Impl::hasPending()  //已经挂起
{
  if (!isValid() || timer_handle_ == -1)
  {
    return false;
  }

  return TimerManager<WallTime, WallDuration, WallTimerEvent>::global().hasPending(timer_handle_);
}

void WallTimer::Impl::setPeriod(const WallDuration& period, bool reset) //设定时期段
{
  period_ = period;
  TimerManager<WallTime, WallDuration, WallTimerEvent>::global().setPeriod(timer_handle_, period, reset);
}


WallTimer::WallTimer(const WallTimerOptions& ops)   //壁钟时间的构造函数
: impl_(new Impl)
{
  impl_->period_ = ops.period;
  impl_->callback_ = ops.callback;
  impl_->callback_queue_ = ops.callback_queue;
  impl_->tracked_object_ = ops.tracked_object;
  impl_->has_tracked_object_ = (ops.tracked_object != NULL);
  impl_->oneshot_ = ops.oneshot;
}

WallTimer::WallTimer(const WallTimer& rhs)  //拷贝函数
{
  impl_ = rhs.impl_;
}

WallTimer::~WallTimer()
{
}

void WallTimer::start()
{
  if (impl_)
  {
    impl_->start();
  }
}

void WallTimer::stop()
{
  if (impl_)
  {
    impl_->stop();
  }
}

bool WallTimer::hasPending()
{
  if (impl_)
  {
    return impl_->hasPending();
  }

  return false;
}

void WallTimer::setPeriod(const WallDuration& period, bool reset)
{
  if (impl_)
  {
    impl_->setPeriod(period, reset);
  }
}

}
