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
  // ROS 话题名，来自 send_topics 或 recv_topics 配置。
  std::string name;

  // ROS 消息类型字符串，例如 "std_msgs/String"。
  std::string type;

  // 最大发送频率，单位 Hz；仅发送话题使用。
  int max_freq;

  // 序列化 payload 最大码率，单位 bps；<= 0 表示不限速。
  double max_bitrate;

  // 模拟节点故障期间的话题处理策略：drop、buffer 或 latest。
  std::string fault_policy;

  // 从 YAML IP map 解析出的 ZMQ bind/connect 地址。
  std::string ip;

  // 当前话题流使用的 ZMQ TCP 端口。
  int port;
};

// 当前 ROS 节点 namespace，用于打印相对话题名。
extern std::string ns; // namespace of this node

// 从私有 ROS 参数服务器读取的原始 YAML IP map。
extern XmlRpc::XmlRpcValue ip_xml;

// 从私有 ROS 参数服务器读取的原始 send_topics 数组。
extern XmlRpc::XmlRpcValue send_topics_xml;

// 从私有 ROS 参数服务器读取的原始 recv_topics 数组。
extern XmlRpc::XmlRpcValue recv_topics_xml;

// 已配置的发送话题数量。
extern int len_send; // length(number) of send topics

// 已配置的接收话题数量。
extern int len_recv; // length(number) of receive topics

// 逻辑主机名到 IP 地址的映射，来自 YAML IP section。
extern std::map<std::string, std::string> ip_map; // map host name and IP

// 所有 outgoing topic stream 的解析后元信息。
extern std::vector<TopicInfo> sendTopics; // send topics info struct vector

// 所有 incoming topic stream 的解析后元信息。
extern std::vector<TopicInfo> recvTopics; // receive topics info struct vector

// bridge data-plane PUB/SUB socket 共用的 ZMQ context。
extern zmqpp::context_t context;

// 每个发送话题对应的 ZMQ PUB socket，索引与 sendTopics 对齐。
extern std::vector<std::unique_ptr<zmqpp::socket>> senders;   //index senders

// 每个接收话题对应的 ZMQ SUB socket，索引与 recvTopics 对齐。
extern std::vector<std::unique_ptr<zmqpp::socket>> receivers; //index receivers

// 本地 ROS subscriber，用于接收待桥接发送的 ROS 消息。
extern std::vector<ros::Subscriber> topic_subs;

// 本地 ROS publisher，用于发布 bridge 收到的远端消息。
extern std::vector<ros::Publisher> topic_pubs;

// 每个发送话题的 frequency-control 周期起始时间。
extern std::vector<ros::Time> sub_t_last;

// 每个发送话题在当前 frequency-control 周期内的发送计数。
extern std::vector<int> send_num;

// 每个接收线程的运行标志。
extern std::vector<bool> recv_thread_flags;

// 每个接收线程上一轮 receive 状态，用于首次收到消息时打印日志。
extern std::vector<bool> recv_flags_last;

// 后台接收线程，索引与 recvTopics 对齐。
extern std::vector<std::thread> recv_threads;

#endif
