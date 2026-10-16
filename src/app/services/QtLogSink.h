#pragma once
#include "core/Logger.h"
#include <QMetaType>
#include <QObject>
#include <memory>

namespace mllm
{
// 后台日志排入本对象所属线程；注销时也能安全丢弃尚未分发的记录。
class QtLogSink final : public QObject
{
    Q_OBJECT
  public:
    explicit QtLogSink(QObject* parent = nullptr);
    ~QtLogSink() override;
  signals:
    void messageReceived(mllm::LogMessage message);

  private:
    struct State;
    std::shared_ptr<State> state_;
    Logger::SinkId sinkId_ = 0;
};
} // namespace mllm
Q_DECLARE_METATYPE(mllm::LogMessage)
