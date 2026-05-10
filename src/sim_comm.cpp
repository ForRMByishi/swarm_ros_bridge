/**
 * @file sim_comm.cpp
 * @brief swarm_ros_bridge 的发送侧模拟通信控制实现。
 *
 * Purpose:
 *   为序列化后的 ROS 消息实现可选的话题级带宽限制和节点故障模拟。
 *
 * Responsibilities:
 *   - 为 max_bitrate 为正值的话题提供 token bucket 发送器。
 *   - 按话题 fault_policy 模拟节点网络中断期间的丢弃或缓存行为。
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
#include <string>
#include <thread>
#include <utility>

#include <ros/ros.h>
#include <zmqpp/zmqpp.hpp>

#include "bridge_state.hpp"

namespace
{
const double BITRATE_QUEUE_WINDOW_SEC = 0.2;
const double DEFAULT_REPLAY_INTERVAL_SEC = 0.01;
const double FAULT_REPLAY_RATE_MULTIPLIER = 2.0;
const size_t MAX_BITRATE_QUEUE_MESSAGES = 30;

std::vector<std::unique_ptr<std::atomic<bool>>> send_thread_flags;
std::vector<std::thread> send_threads;
std::vector<std::unique_ptr<std::deque<SimCommMessage>>> send_queues;
std::vector<std::unique_ptr<std::mutex>> send_queue_mutexes;
std::vector<std::unique_ptr<std::condition_variable>> send_queue_conditions;
std::vector<size_t> send_queue_bytes;

std::mutex node_fault_mutex;
std::condition_variable node_fault_condition;
std::thread node_fault_thread;
bool node_fault_thread_running = false;
bool node_fault_active = false;
bool node_fault_recovering = false;
double node_fault_duration_sec = 0.0;
std::chrono::steady_clock::time_point node_fault_until;
SimCommPublishCallback recv_publish_callback = nullptr;
std::vector<SimCommFaultPolicy> send_fault_policies;
std::vector<SimCommFaultPolicy> recv_fault_policies;
std::vector<std::deque<SimCommMessage>> send_fault_buffers;
std::vector<std::deque<SimCommMessage>> recv_fault_buffers;

void enqueue_send_message(int topic_index, SimCommMessage message);
void flush_expired_fault_if_needed();
void apply_fault_policy_to_buffer(SimCommFaultPolicy policy,
                                  SimCommMessage message,
                                  std::deque<SimCommMessage> &buffer);
bool consume_send_message_if_fault_active(int topic_index, SimCommMessage &message);

/**
 * 将字符串策略转换为内部枚举。
 *
 * Parameters:
 *   policy_name: YAML 中配置的 fault_policy 名称。
 *   topic_name: 用于错误日志定位的话题名。
 *
 * Returns:
 *   对应的故障策略枚举。
 *
 * Side Effects:
 *   策略名称非法时记录 fatal 日志并退出。
 */
SimCommFaultPolicy parse_fault_policy(const std::string &policy_name,
                                      const std::string &topic_name)
{
  if (policy_name == "drop")
  {
    return SimCommFaultPolicy::DROP;
  }
  if (policy_name == "buffer")
  {
    return SimCommFaultPolicy::BUFFER;
  }
  if (policy_name == "latest")
  {
    return SimCommFaultPolicy::LATEST;
  }

  ROS_FATAL("[bridge_node] Invalid fault_policy \"%s\" on topic \"%s\". Use drop, buffer, or latest.",
            policy_name.c_str(),
            topic_name.c_str());
  exit(1);
}

/**
 * 判断当前节点是否处于模拟网络故障窗口。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   true 表示故障窗口仍然有效。
 *
 * Side Effects:
 *   None.
 */
bool is_node_fault_active_locked()
{
  return node_fault_active && std::chrono::steady_clock::now() < node_fault_until;
}

/**
 * 获取故障缓存恢复时的回放间隔。
 *
 * Parameters:
 *   topic_index: 当前方向的话题索引。
 *   is_send_topic: true 表示发送话题，false 表示接收话题。
 *
 * Returns:
 *   发送侧按 max_freq 的固定倍率回放；接收侧没有 max_freq 时使用默认间隔。
 *
 * Side Effects:
 *   None.
 */
std::chrono::duration<double> get_fault_replay_interval(int topic_index, bool is_send_topic)
{
  if (is_send_topic && sendTopics[topic_index].max_freq > 0)
  {
    return std::chrono::duration<double>(
        1.0 / (static_cast<double>(sendTopics[topic_index].max_freq) *
               FAULT_REPLAY_RATE_MULTIPLIER));
  }

  return std::chrono::duration<double>(DEFAULT_REPLAY_INTERVAL_SEC);
}

/**
 * 获取接收侧故障缓存恢复时的动态回放间隔。
 *
 * Parameters:
 *   message_count: 当前接收话题缓存的消息条数。
 *   fault_duration_sec: 本次模拟故障持续时间。
 *
 * Returns:
 *   基于缓存条数和故障时长的回放间隔，目标是在半个故障时长内释放完缓存。
 *
 * Side Effects:
 *   None.
 */
std::chrono::duration<double> get_recv_fault_replay_interval(size_t message_count,
                                                             double fault_duration_sec)
{
  if (message_count > 1 && fault_duration_sec > 0.0)
  {
    return std::chrono::duration<double>(
        fault_duration_sec / (static_cast<double>(message_count) *
                              FAULT_REPLAY_RATE_MULTIPLIER));
  }

  return std::chrono::duration<double>(DEFAULT_REPLAY_INTERVAL_SEC);
}

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
void send_serialized_message(int topic_index, const SimCommMessage &message)
{
  zmqpp::message send_array;
  send_array << message.data_len;
  send_array.add_raw(reinterpret_cast<void const *>(message.data.data()), message.data_len);

  // PUB 模式下 send 通常不会阻塞，这里保留原实现的 blocking flag 行为。
  bool dont_block = false;
  senders[topic_index]->send(send_array, dont_block);
}

/**
 * 在故障策略处理完成后发送一条消息。
 *
 * Parameters:
 *   topic_index: sendTopics 中的发送话题索引。
 *   message: 待发送的序列化 ROS payload。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   继续走原有 max_bitrate 限速路径或立即 ZMQ 发送路径。
 */
void dispatch_send_message_after_fault(int topic_index,
                                       SimCommMessage message,
                                       bool should_check_fault_before_zmq)
{
  if (sendTopics[topic_index].max_bitrate > 0.0)
  {
    enqueue_send_message(topic_index, std::move(message));
    return;
  }

  if (!should_check_fault_before_zmq || !consume_send_message_if_fault_active(topic_index, message))
  {
    send_serialized_message(topic_index, message);
  }
}

/**
 * 将接收侧消息提交给上层 ROS publisher。
 *
 * Parameters:
 *   topic_index: recvTopics 中的接收话题索引。
 *   message: 待提交的序列化 ROS payload。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   调用 bridge_node.cpp 提供的反序列化发布回调。
 */
void publish_received_message_after_fault(int topic_index, SimCommMessage message)
{
  if (recv_publish_callback == nullptr)
  {
    ROS_ERROR_THROTTLE(1.0, "[bridge node] Receive publish callback is not initialized.");
    return;
  }

  recv_publish_callback(message.data.data(), message.data_len, topic_index);
}

/**
 * 在真正写入 ZMQ 前再次检查发送侧故障窗口。
 *
 * Parameters:
 *   topic_index: sendTopics 中的发送话题索引。
 *   message: 即将写入 ZMQ 的序列化 ROS payload。
 *
 * Returns:
 *   true 表示消息已被故障策略消费，不应继续发送；false 表示可以正常发送。
 *
 * Side Effects:
 *   故障窗口内按发送侧 fault_policy 缓存或丢弃消息。
 */
bool consume_send_message_if_fault_active(int topic_index, SimCommMessage &message)
{
  flush_expired_fault_if_needed();

  std::unique_lock<std::mutex> lock(node_fault_mutex);
  node_fault_condition.wait(lock, [] {
    return !node_fault_recovering;
  });

  if (!is_node_fault_active_locked())
  {
    return false;
  }

  apply_fault_policy_to_buffer(send_fault_policies[topic_index],
                               std::move(message),
                               send_fault_buffers[topic_index]);
  return true;
}

/**
 * 根据策略缓存或丢弃一条故障窗口内的消息。
 *
 * Parameters:
 *   policy: 当前话题的故障策略。
 *   message: 故障窗口内到达的消息。
 *   buffer: 当前方向和话题对应的故障缓存。
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   按 drop/buffer/latest 规则更新缓存。
 */
void apply_fault_policy_to_buffer(SimCommFaultPolicy policy,
                                  SimCommMessage message,
                                  std::deque<SimCommMessage> &buffer)
{
  if (policy == SimCommFaultPolicy::DROP)
  {
    return;
  }

  if (policy == SimCommFaultPolicy::LATEST)
  {
    buffer.clear();
  }

  buffer.emplace_back(std::move(message));
}

/**
 * 释放故障期间缓存的所有发送和接收消息。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   发送缓存重新进入正常发送路径；接收缓存重新进入正常上层提交路径。
 */
void flush_fault_buffers()
{
  std::vector<std::deque<SimCommMessage>> send_buffers_to_flush;
  std::vector<std::deque<SimCommMessage>> recv_buffers_to_flush;
  double fault_duration_sec = 0.0;

  {
    std::lock_guard<std::mutex> lock(node_fault_mutex);
    send_buffers_to_flush.swap(send_fault_buffers);
    recv_buffers_to_flush.swap(recv_fault_buffers);
    send_fault_buffers.resize(sendTopics.size());
    recv_fault_buffers.resize(recvTopics.size());
    fault_duration_sec = node_fault_duration_sec;
  }

  for (size_t topic_index = 0; topic_index < send_buffers_to_flush.size(); ++topic_index)
  {
    const auto replay_interval =
        get_fault_replay_interval(static_cast<int>(topic_index), true);
    bool is_first_replay_message = true;

    for (auto &message : send_buffers_to_flush[topic_index])
    {
      if (!is_first_replay_message)
      {
        std::this_thread::sleep_for(replay_interval);
      }
      is_first_replay_message = false;

      dispatch_send_message_after_fault(static_cast<int>(topic_index),
                                        std::move(message),
                                        false);
    }
  }

  for (size_t topic_index = 0; topic_index < recv_buffers_to_flush.size(); ++topic_index)
  {
    const auto replay_interval =
        get_recv_fault_replay_interval(recv_buffers_to_flush[topic_index].size(),
                                       fault_duration_sec);
    bool is_first_replay_message = true;

    for (auto &message : recv_buffers_to_flush[topic_index])
    {
      if (!is_first_replay_message)
      {
        std::this_thread::sleep_for(replay_interval);
      }
      is_first_replay_message = false;

      publish_received_message_after_fault(static_cast<int>(topic_index), std::move(message));
    }
  }
}

/**
 * 在收发入口同步完成已经到期的故障恢复。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   如果故障窗口已经到期，则先释放缓存，再允许当前新消息继续处理。
 */
void flush_expired_fault_if_needed()
{
  bool should_flush = false;

  {
    std::lock_guard<std::mutex> lock(node_fault_mutex);
    if (node_fault_active && std::chrono::steady_clock::now() >= node_fault_until)
    {
      node_fault_active = false;
      node_fault_recovering = true;
      should_flush = true;
    }
  }

  if (should_flush)
  {
    flush_fault_buffers();
    {
      std::lock_guard<std::mutex> lock(node_fault_mutex);
      node_fault_recovering = false;
    }
    node_fault_condition.notify_all();
  }
}

/**
 * 周期检查模拟故障是否结束，并在结束时释放缓存。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   到达恢复时间后修改故障状态并释放缓存。
 */
void node_fault_monitor_func()
{
  std::unique_lock<std::mutex> lock(node_fault_mutex);

  while (node_fault_thread_running)
  {
    if (!node_fault_active)
    {
      node_fault_condition.wait(lock, [] {
        return !node_fault_thread_running || node_fault_active;
      });
      continue;
    }

    if (node_fault_condition.wait_until(lock, node_fault_until, [] {
          return !node_fault_thread_running;
        }))
    {
      break;
    }

    if (node_fault_active && std::chrono::steady_clock::now() >= node_fault_until)
    {
      node_fault_active = false;
      node_fault_recovering = true;
      lock.unlock();
      flush_fault_buffers();
      lock.lock();
      node_fault_recovering = false;
      node_fault_condition.notify_all();
    }
  }
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
void enqueue_send_message(int topic_index, SimCommMessage message)
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
    SimCommMessage message;
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
    if (!consume_send_message_if_fault_active(topic_index, message))
    {
      send_serialized_message(topic_index, message);
    }
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

std::string get_optional_string_param(XmlRpc::XmlRpcValue topic_xml,
                                      const std::string &field_name,
                                      const std::string &default_value)
{
  if (!topic_xml.hasMember(field_name))
  {
    return default_value;
  }

  XmlRpc::XmlRpcValue field_value = topic_xml[field_name];
  if (field_value.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    return static_cast<std::string>(field_value);
  }

  ROS_FATAL("[bridge_node] Optional config \"%s\" must be a string!", field_name.c_str());
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
    send_queues.emplace_back(new std::deque<SimCommMessage>());
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

void initialize_node_fault_control(SimCommPublishCallback publish_callback)
{
  recv_publish_callback = publish_callback;
  send_fault_policies.clear();
  recv_fault_policies.clear();
  send_fault_buffers.clear();
  recv_fault_buffers.clear();

  for (const auto &topic : sendTopics)
  {
    send_fault_policies.emplace_back(parse_fault_policy(topic.fault_policy, topic.name));
    send_fault_buffers.emplace_back();
  }

  for (const auto &topic : recvTopics)
  {
    recv_fault_policies.emplace_back(parse_fault_policy(topic.fault_policy, topic.name));
    recv_fault_buffers.emplace_back();
  }

  node_fault_thread_running = true;
  node_fault_thread = std::thread(&node_fault_monitor_func);
}

void start_node_fault(double duration_sec)
{
  if (duration_sec <= 0.0)
  {
    ROS_WARN("[bridge node] Ignore non-positive node fault duration %.3f.", duration_sec);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(node_fault_mutex);
    if (!node_fault_thread_running)
    {
      return;
    }

    node_fault_active = true;
    node_fault_duration_sec = duration_sec;
    node_fault_until =
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(duration_sec));

    // 新故障窗口开始时清理旧缓存，避免上一轮未释放内容混入本轮语义。
    for (auto &buffer : send_fault_buffers)
    {
      buffer.clear();
    }
    for (auto &buffer : recv_fault_buffers)
    {
      buffer.clear();
    }
  }

  node_fault_condition.notify_one();
  ROS_WARN("[bridge node] Start simulated node fault for %.3f seconds.", duration_sec);
}

bool is_node_fault_busy()
{
  std::lock_guard<std::mutex> lock(node_fault_mutex);
  return node_fault_active || node_fault_recovering;
}

void schedule_node_fault(double start_after_sec, double duration_sec)
{
  if (start_after_sec <= 0.0)
  {
    start_node_fault(duration_sec);
    return;
  }

  std::thread([start_after_sec, duration_sec] {
    std::this_thread::sleep_for(std::chrono::duration<double>(start_after_sec));
    start_node_fault(duration_sec);
  }).detach();
}

void dispatch_send_message(int topic_index, SimCommMessage message)
{
  flush_expired_fault_if_needed();

  {
    std::unique_lock<std::mutex> lock(node_fault_mutex);
    node_fault_condition.wait(lock, [] {
      return !node_fault_recovering;
    });
    if (is_node_fault_active_locked())
    {
      apply_fault_policy_to_buffer(send_fault_policies[topic_index],
                                   std::move(message),
                                   send_fault_buffers[topic_index]);
      return;
    }
  }

  dispatch_send_message_after_fault(topic_index, std::move(message), true);
}

void dispatch_received_message(int topic_index, SimCommMessage message)
{
  flush_expired_fault_if_needed();

  {
    std::unique_lock<std::mutex> lock(node_fault_mutex);
    node_fault_condition.wait(lock, [] {
      return !node_fault_recovering;
    });
    if (is_node_fault_active_locked())
    {
      apply_fault_policy_to_buffer(recv_fault_policies[topic_index],
                                   std::move(message),
                                   recv_fault_buffers[topic_index]);
      return;
    }
  }

  publish_received_message_after_fault(topic_index, std::move(message));
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

void stop_node_fault_control()
{
  {
    std::lock_guard<std::mutex> lock(node_fault_mutex);
    node_fault_thread_running = false;
    node_fault_active = false;
    node_fault_recovering = false;
    node_fault_duration_sec = 0.0;
  }

  node_fault_condition.notify_all();
  if (node_fault_thread.joinable())
  {
    node_fault_thread.join();
  }
}
