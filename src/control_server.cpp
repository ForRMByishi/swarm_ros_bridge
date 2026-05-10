/**
 * @file control_server.cpp
 * @brief swarm_ros_bridge 的远程故障触发控制服务实现。
 *
 * Purpose:
 *   为桥节点提供一个轻量级 ZMQ REP 控制端口，使外部测试电脑可以按节点触发
 *   “节点级模拟断链”，并复用 sim_comm 中已有的缓存、丢弃和恢复回放流程。
 *
 * Responsibilities:
 *   - 监听 key=value 形式的远程控制请求。
 *   - 校验 command、target、token 和 duration_sec。
 *   - 在当前节点空闲时调用 start_node_fault() 触发模拟故障。
 *
 * Notes:
 *   本模块只负责控制面，不直接处理 ROS/ZMQ 话题数据面。请求协议保持为简单文本，
 *   避免为实验控制功能额外引入 JSON 等依赖。
 */

#include "control_server.hpp"

#include "sim_comm.hpp"

#include <ros/ros.h>
#include <zmqpp/zmqpp.hpp>

#include <atomic>
#include <chrono>
#include <cctype>
#include <exception>
#include <future>
#include <map>
#include <sstream>
#include <string>
#include <thread>

namespace
{

std::atomic<bool> control_thread_running(false);
std::thread control_thread;
ControlServerConfig active_config;

/**
 * 去除字符串首尾空白字符。
 *
 * Parameters:
 *   text: 待处理字符串。
 *
 * Returns:
 *   去除首尾空白后的字符串。
 *
 * Side Effects:
 *   None.
 */
std::string trim_copy(const std::string &text)
{
  size_t begin = 0;
  while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])))
  {
    ++begin;
  }

  size_t end = text.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
  {
    --end;
  }

  return text.substr(begin, end - begin);
}

/**
 * 解析 key=value;key=value 格式的控制请求。
 *
 * Parameters:
 *   request_text: 控制端发送的原始请求文本。
 *
 * Returns:
 *   字段名到字段值的映射。格式不完整的片段会被忽略。
 *
 * Side Effects:
 *   None.
 */
std::map<std::string, std::string> parse_key_value_request(const std::string &request_text)
{
  std::map<std::string, std::string> fields;
  std::stringstream stream(request_text);
  std::string segment;

  while (std::getline(stream, segment, ';'))
  {
    const size_t separator_pos = segment.find('=');
    if (separator_pos == std::string::npos)
    {
      continue;
    }

    const std::string key = trim_copy(segment.substr(0, separator_pos));
    const std::string value = trim_copy(segment.substr(separator_pos + 1));
    if (!key.empty())
    {
      fields[key] = value;
    }
  }

  return fields;
}

/**
 * 将控制响应编码为 key=value 文本。
 *
 * Parameters:
 *   ok: 请求是否成功处理。
 *   node_id: 当前控制服务对应的节点标识。
 *   message_key: 成功消息或错误码字段名。
 *   message_value: 成功消息或错误码字段值。
 *
 * Returns:
 *   可由脚本直接打印和解析的响应字符串。
 *
 * Side Effects:
 *   None.
 */
std::string build_response(bool ok,
                           const std::string &node_id,
                           const std::string &message_key,
                           const std::string &message_value)
{
  std::stringstream response;
  response << "ok=" << (ok ? "true" : "false")
           << ";node_id=" << node_id
           << ";" << message_key << "=" << message_value;
  return response.str();
}

/**
 * 将 duration_sec 字段解析为正数秒数。
 *
 * Parameters:
 *   fields: 已解析的请求字段。
 *   duration_sec: 输出参数，成功时写入故障持续时间。
 *
 * Returns:
 *   true 表示解析成功且 duration_sec > 0；false 表示字段缺失或非法。
 *
 * Side Effects:
 *   None.
 */
bool parse_positive_duration(const std::map<std::string, std::string> &fields, double &duration_sec)
{
  const auto iter = fields.find("duration_sec");
  if (iter == fields.end())
  {
    return false;
  }

  try
  {
    size_t parsed_chars = 0;
    duration_sec = std::stod(iter->second, &parsed_chars);
    return parsed_chars == iter->second.size() && duration_sec > 0.0;
  }
  catch (const std::exception &)
  {
    return false;
  }
}

/**
 * 处理一条远程控制请求，并在校验通过后触发模拟节点故障。
 *
 * Parameters:
 *   request_text: 控制端发送的 key=value 请求文本。
 *
 * Returns:
 *   key=value 格式响应文本。
 *
 * Side Effects:
 *   请求有效且当前节点空闲时，会调用 start_node_fault() 修改 sim_comm 故障状态。
 */
std::string handle_control_request(const std::string &request_text)
{
  const std::map<std::string, std::string> fields = parse_key_value_request(request_text);
  const auto command_iter = fields.find("command");
  if (command_iter == fields.end() || command_iter->second != "start_node_fault")
  {
    return build_response(false, active_config.node_id, "error", "invalid_command");
  }

  const auto target_iter = fields.find("target");
  if (target_iter != fields.end() && !target_iter->second.empty() &&
      !active_config.node_id.empty() && target_iter->second != active_config.node_id)
  {
    return build_response(false, active_config.node_id, "error", "target_mismatch");
  }

  if (!active_config.token.empty())
  {
    const auto token_iter = fields.find("token");
    if (token_iter == fields.end() || token_iter->second != active_config.token)
    {
      return build_response(false, active_config.node_id, "error", "unauthorized");
    }
  }

  double duration_sec = 0.0;
  if (!parse_positive_duration(fields, duration_sec))
  {
    return build_response(false, active_config.node_id, "error", "invalid_duration_sec");
  }

  if (is_node_fault_busy())
  {
    return build_response(false, active_config.node_id, "error", "node_fault_busy");
  }

  start_node_fault(duration_sec);

  std::stringstream response;
  response << build_response(true, active_config.node_id, "message", "node_fault_started")
           << ";duration_sec=" << duration_sec;
  return response.str();
}

/**
 * 控制服务后台线程主循环。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   绑定 ZMQ REP socket，并在收到有效请求后触发 sim_comm 故障控制。
 */
void control_thread_main(std::promise<bool> startup_result)
{
  const std::string endpoint =
      "tcp://" + active_config.bind_ip + ":" + std::to_string(active_config.bind_port);
  bool startup_reported = false;

  try
  {
    zmqpp::context context;
    zmqpp::socket control_socket(context, zmqpp::socket_type::rep);
    control_socket.bind(endpoint);
    startup_result.set_value(true);
    startup_reported = true;

    ROS_INFO("[control_server] Node \"%s\" listening on %s",
             active_config.node_id.c_str(), endpoint.c_str());

    while (control_thread_running.load())
    {
      zmqpp::message request;
      if (!control_socket.receive(request, true))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      std::string response_text;
      try
      {
        std::string request_text;
        request >> request_text;
        response_text = handle_control_request(request_text);
      }
      catch (const std::exception &ex)
      {
        // 外部控制请求属于网络输入，解析失败时回复错误而不是终止控制线程。
        ROS_WARN("[control_server] Invalid control request: %s", ex.what());
        response_text = build_response(false, active_config.node_id, "error", "invalid_request");
      }

      zmqpp::message response;
      response << response_text;
      control_socket.send(response);
    }

    control_socket.close();
  }
  catch (const std::exception &ex)
  {
    if (!startup_reported)
    {
      startup_result.set_value(false);
    }
    ROS_ERROR("[control_server] Control server stopped on %s: %s", endpoint.c_str(), ex.what());
  }
}

}  // namespace

bool start_control_server(const ControlServerConfig &config)
{
  if (!config.enabled)
  {
    return true;
  }

  if (control_thread_running.load())
  {
    ROS_WARN("[control_server] Control server is already running.");
    return true;
  }

  if (config.bind_port <= 0 || config.bind_port > 65535)
  {
    ROS_ERROR("[control_server] Invalid control_bind_port: %d", config.bind_port);
    return false;
  }

  active_config = config;
  control_thread_running.store(true);

  std::promise<bool> startup_result;
  std::future<bool> startup_future = startup_result.get_future();
  control_thread = std::thread(control_thread_main, std::move(startup_result));
  if (!startup_future.get())
  {
    control_thread_running.store(false);
    if (control_thread.joinable())
    {
      control_thread.join();
    }
    return false;
  }

  return true;
}

void stop_control_server()
{
  if (!control_thread_running.load())
  {
    return;
  }

  control_thread_running.store(false);
  if (control_thread.joinable())
  {
    control_thread.join();
  }
}
