/**
 * @file sim_comm.hpp
 * @brief swarm_ros_bridge 的模拟通信控制接口。
 *
 * Purpose:
 *   为 bridge_node 提供可选的发送侧通信控制能力。
 *
 * Responsibilities:
 *   - 解析可选的通信控制配置字段。
 *   - 按配置选择立即发送或限速排队发送。
 *   - 管理限速话题的 token bucket 发送线程和有界队列。
 *
 * Notes:
 *   本模块只处理模拟通信约束。ROS 话题解析、ROS 序列化和接收侧发布仍保留在
 *   bridge_node.cpp 中。
 */

#ifndef __SIM_COMM__
#define __SIM_COMM__

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <xmlrpcpp/XmlRpcValue.h>

struct SimCommSendMessage
{
  size_t data_len;
  std::vector<uint8_t> data;
};

/**
 * 从单个话题配置中读取可选数值字段。
 *
 * Parameters:
 *   topic_xml: 话题级 XmlRpc 配置对象。
 *   field_name: 要读取的可选字段名。
 *   default_value: 字段不存在时返回的默认值。
 *
 * Returns:
 *   配置中的数值；字段不存在时返回 default_value。
 *
 * Side Effects:
 *   字段存在但不是数值类型时记录 ROS fatal 日志并退出。
 */
double get_optional_numeric_param(XmlRpc::XmlRpcValue topic_xml,
                                  const std::string &field_name,
                                  double default_value);

/**
 * 初始化每个发送话题的限速队列和同步对象。
 *
 * Parameters:
 *   topic_count: 发送话题数量。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   重置本模块内部的队列、线程和停止标志容器。
 */
void initialize_send_bitrate_control(int topic_count);

/**
 * 为 max_bitrate 为正值的话题启动 token bucket 发送线程。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   为需要限速的话题创建后台发送线程。
 */
void start_send_bitrate_threads();

/**
 * 按话题限速配置发送或排队一条序列化消息。
 *
 * Parameters:
 *   topic_index: sendTopics 中的发送话题索引。
 *   message: 序列化后的 ROS payload 及其字节长度。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   未启用 max_bitrate 时立即发送；启用后将消息放入限速队列。
 */
void dispatch_send_message(int topic_index, SimCommSendMessage message);

/**
 * 停止指定发送话题的限速工作线程。
 *
 * Parameters:
 *   topic_index: sendTopics 中的发送话题索引。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   启用限速时唤醒并 join 该话题的发送线程。
 */
void stop_send_bitrate_control(int topic_index);

#endif
