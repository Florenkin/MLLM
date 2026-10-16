#pragma once
#include <chrono>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace mllm
{
enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error
};

// UTF-8 消息携带发生时间，文件和界面使用同一条记录。
struct LogMessage
{
    LogLevel level = LogLevel::Info;
    std::string module;
    std::string text;
    std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
};

class Logger final
{
  public:
    using Sink = std::function<void(const LogMessage&)>;
    using SinkId = std::size_t;
    static Logger& instance();
    SinkId addSink(Sink sink);
    void removeSink(SinkId id);
    void log(LogLevel level, const std::string& module, const std::string& text);
    void debug(const std::string& module, const std::string& text);
    void info(const std::string& module, const std::string& text);
    void warning(const std::string& module, const std::string& text);
    void error(const std::string& module, const std::string& text);

  private:
    Logger() = default;
    std::mutex mutex_;
    std::vector<std::pair<SinkId, Sink>> sinks_;
    SinkId nextSinkId_ = 1;
};
const char* toString(LogLevel level);
} // namespace mllm
