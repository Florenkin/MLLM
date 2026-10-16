#pragma once
#include "app/services/Workflow.h"
#include <QFutureWatcher>
#include <QObject>

namespace mllm
{
// 一个执行器同一时间只接受一个任务，不持有窗口或控件。
class TaskRunner final : public QObject
{
    Q_OBJECT
  public:
    using Work = std::function<JobResult(const TaskContext&)>;
    explicit TaskRunner(QObject* parent = nullptr);
    ~TaskRunner() override;
    bool start(JobKind kind, Work work);
    void cancel();
    void wait();
    bool isRunning() const;
  signals:
    void started();
    void progress(int percent, QString text);
    void finished(mllm::JobResult result);

  private:
    struct State;
    std::shared_ptr<State> state_;
    QFutureWatcher<JobResult> watcher_;
    TaskContext task_;
    bool running_ = false;
    JobKind kind_ = JobKind::CameraCalibration;
};
} // namespace mllm
Q_DECLARE_METATYPE(mllm::JobResult)
