#pragma once
#include "core/Types.h"
namespace mllm
{
struct JobResult
{
    bool success = false, cancelled = false;
    int kind = 0; // camera, plane, reconstruction.
    std::string message;
    Calibration calibration;
    BatchResult batch;
    Path directory;
};
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
