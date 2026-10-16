#include "app/services/QtLogSink.h"
#include <QMetaObject>
#include <mutex>

namespace mllm
{
struct QtLogSink::State
{
    std::mutex mutex;
    QtLogSink* target = nullptr;
};
QtLogSink::QtLogSink(QObject* parent) : QObject(parent), state_(std::make_shared<State>())
{
    qRegisterMetaType<LogMessage>("mllm::LogMessage");
    state_->target = this;
    sinkId_ = Logger::instance().addSink(
        [state = state_](const LogMessage& message)
        {
            if (message.level == LogLevel::Debug)
                return;
            std::lock_guard<std::mutex> lock(state->mutex);
            if (auto* target = state->target)
                QMetaObject::invokeMethod(
                    target, [target, message] { emit target->messageReceived(message); },
                    Qt::QueuedConnection);
        });
}
QtLogSink::~QtLogSink()
{
    Logger::instance().removeSink(sinkId_);
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->target = nullptr;
}
} // namespace mllm
