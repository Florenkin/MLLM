#include "app/services/TaskRunner.h"
#include <QMetaObject>
#include <QtConcurrent/QtConcurrent>
#include <algorithm>
#include <mutex>

namespace mllm
{
struct TaskRunner::State
{
    std::mutex mutex;
    TaskRunner* target = nullptr;
    std::size_t generation = 0;
};
TaskRunner::TaskRunner(QObject* parent) : QObject(parent), state_(std::make_shared<State>())
{
    qRegisterMetaType<JobResult>("mllm::JobResult");
    state_->target = this;
    connect(&watcher_, &QFutureWatcher<JobResult>::finished, this,
            [this]
            {
                JobResult result;
                result.kind = kind_;
                try
                {
                    result = watcher_.result();
                }
                catch (...)
                {
                    result.message = "无法读取后台任务结果";
                }
                running_ = false;
                {
                    std::lock_guard<std::mutex> lock(state_->mutex);
                    ++state_->generation;
                }
                emit finished(result);
            });
}
TaskRunner::~TaskRunner()
{
    cancel();
    wait();
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->target = nullptr;
}
bool TaskRunner::start(JobKind kind, Work work)
{
    if (running_ || !work)
        return false;
    running_ = true;
    kind_ = kind;
    task_ = TaskContext{};
    std::size_t generation;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        generation = ++state_->generation;
    }
    task_.progress = [state = state_, generation](int percent, const std::string& text)
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (auto* target = state->target)
            QMetaObject::invokeMethod(
                target,
                [target, state, generation, percent, text]
                {
                    {
                        std::lock_guard<std::mutex> guard(state->mutex);
                        if (state->generation != generation)
                            return;
                    }
                    emit target->progress(std::clamp(percent, 0, 100),
                                          QString::fromUtf8(text.data(), static_cast<int>(text.size())));
                },
                Qt::QueuedConnection);
    };
    const auto task = task_;
    try
    {
        watcher_.setFuture(QtConcurrent::run(
            [kind, work = std::move(work), task]
            {
                try
                {
                    return work(task);
                }
                catch (...)
                {
                    return executeJob(kind, task, [&](JobResult&, const TaskContext&) { throw; });
                }
            }));
    }
    catch (...)
    {
        running_ = false;
        const auto failure = executeJob(kind, task, [&](JobResult&, const TaskContext&) { throw; });
        emit finished(failure);
        return false;
    }
    emit started();
    return true;
}
void TaskRunner::cancel()
{
    task_.cancel->store(true);
}
void TaskRunner::wait()
{
    watcher_.waitForFinished();
}
bool TaskRunner::isRunning() const
{
    return running_;
}
} // namespace mllm
