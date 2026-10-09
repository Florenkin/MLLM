#pragma once
#include "core/Types.h"
namespace mllm
{
const LaserPlane& planeAt(const Calibration& calibration, double position);
std::string reconstructionParameters(const ReconstructionConfig& config);
FrameResult reconstructFrame(const cv::Mat& image, const CameraCalibration& camera, const LaserPlane& plane,
                             const ReconstructionConfig& config);
BatchResult reconstructBatch(const Path& images, const Path& output, const Calibration& calibration,
                             const ReconstructionConfig& config, const TaskContext& task = {});
} // namespace mllm
