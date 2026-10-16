#pragma once
#include "core/Types.h"
namespace mllm
{
enum class JobKind
{
    CameraCalibration,
    LaserCalibration,
    Reconstruction,
    CloudLoad,
    CloudFit,
    CloudDerived,
    CloudExport,
    CloudSelect
};
enum class JobStatus
{
    Failed,
    Succeeded,
    Cancelled
};
struct JobResult
{
    JobStatus status = JobStatus::Failed;
    JobKind kind = JobKind::CameraCalibration;
    std::string message;
    Calibration calibration;
    std::shared_ptr<const BatchResult> batch;
    std::shared_ptr<const Cloud> cloud;
    std::shared_ptr<const std::vector<std::size_t>> selection;
    std::string documentName;
    std::size_t sourceDocumentId = 0;
    Path directory;
    bool succeeded() const
    {
        return status == JobStatus::Succeeded;
    }
    bool isCancelled() const
    {
        return status == JobStatus::Cancelled;
    }
};
using JobWork = std::function<void(JobResult&, const TaskContext&)>;
const char* jobModule(JobKind kind);
JobResult executeJob(JobKind kind, const TaskContext& task, const JobWork& work);
JobResult cameraWorkflow(const Path& images, const Path& output, const BoardConfig& board,
                         const TaskContext& task);
JobResult laserWorkflow(const Path& off, const Path& on, const Path& output, Calibration calibration,
                        const BoardConfig& board, const LaserConfig& laser, double position,
                        const TaskContext& task);
JobResult reconstructionWorkflow(const Path& images, const Path& output, const Calibration& calibration,
                                 const ReconstructionConfig& config, const TaskContext& task);
JobResult singleFrameWorkflow(const Path& image, const Path& output, const Calibration& calibration,
                              const ReconstructionConfig& config, const TaskContext& task);
} // namespace mllm
