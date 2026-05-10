/**
 * @file bridge_state.hpp
 * @brief bridge 共享状态声明。
 *
 * Purpose:
 *   声明 bridge_node.cpp 与 sim_comm.cpp 等通信控制模块共享的全局状态。
 *
 * Responsibilities:
 *   - 定义发送和接收路径共用的话题元信息。
 *   - 以 extern 形式暴露 ROS、ZMQ 和配置相关容器。
 *
 * Notes:
 *   本头文件刻意不包含 ros_sub_pub.hpp，避免工具模块编译单元引入消息类型辅助函数
 *   定义后造成重复符号。
 */

#ifndef __BRIDGE_STATE__
#define __BRIDGE_STATE__

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <zmqpp/zmqpp.hpp>

struct TopicInfo
{
  std::string name;
  std::string type;
  int max_freq;
  double max_bitrate;
  std::string ip;
  int port;
};

extern std::string ns; // namespace of this node
extern XmlRpc::XmlRpcValue ip_xml;
extern XmlRpc::XmlRpcValue send_topics_xml;
extern XmlRpc::XmlRpcValue recv_topics_xml;
extern int len_send; // length(number) of send topics
extern int len_recv; // length(number) of receive topics

extern std::map<std::string, std::string> ip_map; // map host name and IP

extern std::vector<TopicInfo> sendTopics; // send topics info struct vector
extern std::vector<TopicInfo> recvTopics; // receive topics info struct vector

extern zmqpp::context_t context;
extern std::vector<std::unique_ptr<zmqpp::socket>> senders;   //index senders
extern std::vector<std::unique_ptr<zmqpp::socket>> receivers; //index receivers

extern std::vector<ros::Subscriber> topic_subs;
extern std::vector<ros::Publisher> topic_pubs;

extern std::vector<ros::Time> sub_t_last;
extern std::vector<int> send_num;

extern std::vector<bool> recv_thread_flags;
extern std::vector<bool> recv_flags_last;
extern std::vector<std::thread> recv_threads;

#endif
