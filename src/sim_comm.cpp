/**
 * @file sim_comm.cpp
 * @brief swarm_ros_bridge 的发送侧模拟通信控制实现。
 *
 * Purpose:
 *   为序列化后的 ROS 消息实现可选的话题级带宽限制。
 *
 * Responsibilities:
 *   - 为 max_bitrate 为正值的话题提供 token bucket 发送器。
 *   - 限制排队积压，并在队列过满时丢弃最早排队的消息。
 *   - 将通信控制状态从 bridge_node 的 ROS/ZMQ 主流程中分离出来。
 *
 * Notes:
 *   max_bitrate 的单位是 bits per second，作用对象是序列化后的 ROS payload 字节。
 */

#include "sim_comm.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <ros/ros.h>
#include <zmqpp/zmqpp.hpp>

#include "bridge_state.hpp"

namespace
{
const double BITRATE_QUEUE_WINDOW_SEC = 0.2;
const size_t MAX_BITRATE_QUEUE_MESSAGES = 30;

std::vector<std::unique_ptr<std::atomic<bool>>> send_thread_flags;
std::vector<std::thread> send_threads;
std::vector<std::unique_ptr<std::deque<SimCommSendMessage>>> send_queues;
std::vector<std::unique_ptr<std::mutex>> send_queue_mutexes;
std::vector<std::unique_ptr<std::condition_variable>> send_queue_conditions;
std::vector<size_t> send_queue_bytes;

/**
 * 通过对应的 ZMQ PUB socket 发送一条序列化 ROS 消息。
 *
 * Parameters:
 *   topic_index: sendTopics 和 senders 中的发送话题索引。
 *   message: 待发送的序列化 ROS payload。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   向该话题对应的 ZMQ socket 写入消息。
 */
void send_serialized_message(int topic_index, const SimCommSendMessage &message)
{
  zmqpp::message send_array;
  send_array << message.data_len;
  send_array.add_raw(reinterpret_cast<void const *>(message.data.data()), message.data_len);

  // PUB 模式下 send 通常不会阻塞，这里保留原实现的 blocking flag 行为。
  bool dont_block = false;
  senders[topic_index]->send(send_array, dont_block);
}

/**
 * 将一条序列化消息入队，并在队列过大时丢弃最旧积压。
 *
 * Parameters:
 *   topic_index: sendTopics 中的发送话题索引。
 *   message: 待入队的序列化 ROS payload。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   更新有界队列，并可能丢弃一条或多条旧消息。
 */
void enqueue_send_message(int topic_index, SimCommSendMessage message)
{
  const double max_queue_bytes_from_time =
      sendTopics[topic_index].max_bitrate / 8.0 * BITRATE_QUEUE_WINDOW_SEC;
  const size_t max_queue_bytes =
      std::max(static_cast<size_t>(max_queue_bytes_from_time), message.data_len);

  std::unique_lock<std::mutex> lock(*send_queue_mutexes[topic_index]);
  auto &send_queue = *send_queues[topic_index];

  // NOTE: 无人车控制链路更怕旧消息迟到，所以队列只保留短时间积压。
  while (!send_queue.empty() &&
         (send_queue.size() >= MAX_BITRATE_QUEUE_MESSAGES ||
          send_queue_bytes[topic_index] + message.data_len > max_queue_bytes))
  {
    send_queue_bytes[topic_index] -= send_queue.front().data_len;
    send_queue.pop_front();
    ROS_WARN_THROTTLE(
        1.0,
        "[bridge node] Drop oldest queued message on topic \"%s\" because bitrate queue is full.",
        sendTopics[topic_index].name.c_str());
  }

  send_queue_bytes[topic_index] += message.data_len;
  send_queue.emplace_back(std::move(message));
  lock.unlock();
  send_queue_conditions[topic_index]->notify_one();
}

/**
 * 使用 token bucket 节奏发送队列消息的工作函数。
 *
 * Parameters:
 *   topic_index: 启用带宽限制的发送话题索引。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   发送队列消息，并按序列化 payload 长度消耗 token。
 */
void send_func(int topic_index)
{
  const double bytes_per_second = sendTopics[topic_index].max_bitrate / 8.0;
  const double base_bucket_capacity =
      std::max(1.0, bytes_per_second * BITRATE_QUEUE_WINDOW_SEC);
  double available_tokens = base_bucket_capacity;
  auto last_refill_time = std::chrono::steady_clock::now();

  while (true)
  {
    SimCommSendMessage message;
    std::unique_lock<std::mutex> lock(*send_queue_mutexes[topic_index]);

    while (true)
    {
      send_queue_conditions[topic_index]->wait(lock, [topic_index] {
        return !send_thread_flags[topic_index]->load() || !send_queues[topic_index]->empty();
      });

      if (!send_thread_flags[topic_index]->load())
      {
        return;
      }

      // Token bucket 使用 wall-clock，避免 ROS 仿真时间暂停时导致限速线程异常等待。
      const auto now = std::chrono::steady_clock::now();
      const std::chrono::duration<double> elapsed = now - last_refill_time;
      last_refill_time = now;
      const size_t front_data_len = send_queues[topic_index]->front().data_len;
      const double bucket_capacity =
          std::max(base_bucket_capacity, static_cast<double>(front_data_len));
      available_tokens =
          std::min(bucket_capacity, available_tokens + elapsed.count() * bytes_per_second);

      if (available_tokens >= static_cast<double>(front_data_len))
      {
        available_tokens -= static_cast<double>(front_data_len);
        message = std::move(send_queues[topic_index]->front());
        send_queues[topic_index]->pop_front();
        send_queue_bytes[topic_index] -= message.data_len;
        break;
      }

      const double wait_seconds =
          (static_cast<double>(front_data_len) - available_tokens) / bytes_per_second;
      send_queue_conditions[topic_index]->wait_for(
          lock, std::chrono::duration<double>(std::min(wait_seconds, 0.02)));
    }

    lock.unlock();
    send_serialized_message(topic_index, message);
  }
}
} // namespace

double get_optional_numeric_param(XmlRpc::XmlRpcValue topic_xml,
                                  const std::string &field_name,
                                  double default_value)
{
  if (!topic_xml.hasMember(field_name))
  {
    return default_value;
  }

  XmlRpc::XmlRpcValue field_value = topic_xml[field_name];
  if (field_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<int>(field_value);
  }
  if (field_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(field_value);
  }

  ROS_FATAL("[bridge_node] Optional config \"%s\" must be a number!", field_name.c_str());
  exit(1);
}

void initialize_send_bitrate_control(int topic_count)
{
  send_thread_flags.clear();
  send_threads.clear();
  send_queues.clear();
  send_queue_mutexes.clear();
  send_queue_conditions.clear();
  send_queue_bytes.clear();

  for (int i = 0; i < topic_count; ++i)
  {
    send_thread_flags.emplace_back(new std::atomic<bool>(true));
    send_threads.emplace_back();
    send_queues.emplace_back(new std::deque<SimCommSendMessage>());
    send_queue_mutexes.emplace_back(new std::mutex());
    send_queue_conditions.emplace_back(new std::condition_variable());
    send_queue_bytes.emplace_back(0);
  }
}

void start_send_bitrate_threads()
{
  for (size_t i = 0; i < sendTopics.size(); ++i)
  {
    if (sendTopics[i].max_bitrate > 0.0)
    {
      send_threads[i] = std::thread(&send_func, static_cast<int>(i));
    }
  }
}

void dispatch_send_message(int topic_index, SimCommSendMessage message)
{
  if (sendTopics[topic_index].max_bitrate > 0.0)
  {
    enqueue_send_message(topic_index, std::move(message));
    return;
  }

  send_serialized_message(topic_index, message);
}

void stop_send_bitrate_control(int topic_index)
{
  if (sendTopics[topic_index].max_bitrate <= 0.0)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(*send_queue_mutexes[topic_index]);
    send_thread_flags[topic_index]->store(false);
  }

  send_queue_conditions[topic_index]->notify_one();
  if (send_threads[topic_index].joinable())
  {
    send_threads[topic_index].join();
  }
}
