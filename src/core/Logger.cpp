#include "core/Logger.h"
#include <algorithm>

namespace mllm
{
Logger& Logger::instance()
{
    static Logger logger;
    return logger;
}
Logger::SinkId Logger::addSink(Sink sink)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto id = nextSinkId_++;
    sinks_.emplace_back(id, std::move(sink));
    return id;
}
void Logger::removeSink(SinkId id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sinks_.erase(
        std::remove_if(sinks_.begin(), sinks_.end(), [id](const auto& entry) { return entry.first == id; }),
        sinks_.end());
}
void Logger::log(LogLevel level, const std::string& module, const std::string& text)
{
    const LogMessage message{level, module, text};
    std::vector<std::pair<SinkId, Sink>> sinks;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sinks = sinks_;
    }
    // 锁外分发，接收器可以注销自己或追加错误日志。
    for (const auto& entry : sinks)
        if (entry.second)
            try
            {
                entry.second(message);
            }
            catch (...)
            {
                // 单个接收器失败不能中断业务，也不能阻止其他接收器写入。
            }
}
void Logger::debug(const std::string& module, const std::string& text)
{
    log(LogLevel::Debug, module, text);
}
void Logger::info(const std::string& module, const std::string& text)
{
    log(LogLevel::Info, module, text);
}
void Logger::warning(const std::string& module, const std::string& text)
{
    log(LogLevel::Warning, module, text);
}
void Logger::error(const std::string& module, const std::string& text)
{
    log(LogLevel::Error, module, text);
}
const char* toString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug:
        return "Debug";
    case LogLevel::Info:
        return "Info";
    case LogLevel::Warning:
        return "Warning";
    case LogLevel::Error:
        return "Error";
    }
    return "Unknown";
}
} // namespace mllm
