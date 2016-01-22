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

#include <cstdio>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/topic_manager.h"
#include "ros/init.h"

#ifdef _MSC_VER
  #ifdef snprintf   //将可变个参数(...)按照format格式化成字符串，然后将其复制到str中
    #undef snprintf
  #endif
  #define snprintf _snprintf_s
#endif
/*sprintf_s在缓冲区不够大时会失败，失败时缓冲区中是一个空字符串。
_snprintf不会失败，但是必须注意如果缓冲区不够大，缓冲区的内容将不是null-teminate的，必须自己注意字符串的结束。
*/

namespace ros
{

namespace names
{
void init(const M_string& remappings);
}

namespace this_node
{

std::string g_name = "empty";
std::string g_namespace;

const std::string& getName()    //获取当前的名字
{
  return g_name;
}

const std::string& getNamespace()   //获取当前命名空间
{
  return g_namespace;
}

void getAdvertisedTopics(V_string& topics)  //返回实例的发布者话题名
{
  TopicManager::instance()->getAdvertisedTopics(topics);
}

void getSubscribedTopics(V_string& topics)  //返回实例的订阅者话题名
{
  TopicManager::instance()->getSubscribedTopics(topics);
}

void init(const std::string& name, const M_string& remappings, uint32_t options)    //这前面还有argc argv 才是节点名
{
  char *ns_env = NULL;
#ifdef _MSC_VER //ROS绝逼是在Win下开发的`_^`
  _dupenv_s(&ns_env, NULL, "ROS_NAMESPACE");    //从环境中取字符串,获取环境变量的值
#else
  ns_env = getenv("ROS_NAMESPACE"); //在这里去环境变量应该指的就是命名空间惹
#endif

  if (ns_env)
  {
    g_namespace = ns_env;   //获取的命名空间值和当前命名空间相等
#ifdef _MSC_VER
    free(ns_env);
#endif
  }

  g_name = name;    //节点名给全局

  bool disable_anon = false;
  M_string::const_iterator it = remappings.find("__name");  //关联容器重命名,不知道它想干啥,看着一堆宏头大了
  if (it != remappings.end())
  {
    g_name = it->second;
    disable_anon = true;
  }

  it = remappings.find("__ns");
  if (it != remappings.end())
  {
    g_namespace = it->second;
  }

  if (g_namespace.empty())
  {
    g_namespace = "/";
  }

  g_namespace = (g_namespace == "/")
    ? std::string("/") 
    : ("/" + g_namespace)
    ;


  std::string error;
  if (!names::validate(g_namespace, error)) //若检测到命名空间非有效数据,报错
  {
    std::stringstream ss;
    ss << "Namespace [" << g_namespace << "] is invalid: " << error;
    throw InvalidNameException(ss.str());
  }

  // names must be initialized here, because it requires the namespace to already be known so that it can properly resolve names.
  // It must be done before we resolve g_name, because otherwise the name will not get remapped.
  //还不太明白它的重映射功能
  names::init(remappings);  //还是不懂重映射==

  if (g_name.find("/") != std::string::npos)    //nops的定义static const size_type npos = -1;
  {
    throw InvalidNodeNameException(g_name, "node names cannot contain /");
  }
  if (g_name.find("~") != std::string::npos)
  {
    throw InvalidNodeNameException(g_name, "node names cannot contain ~");
  }

  g_name = names::resolve(g_namespace, g_name); //resolve名字,全名/+ns+name

  if (options & init_options::AnonymousName && !disable_anon)   //init_options::AnonymousName:  匿名节点,在你的节点名后面加一些数字使它变得unique
  {
    char buf[200];
    snprintf(buf, sizeof(buf), "_%llu", (unsigned long long)WallTime::now().toNSec());
    g_name += buf;
  }

  ros::console::setFixedFilterToken("node", g_name);    //应该是,把节点和名字联系起来
}

} // namespace this_node

} // namespace ros
