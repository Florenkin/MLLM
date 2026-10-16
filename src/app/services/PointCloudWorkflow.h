#pragma once
#include "app/services/PointCloudWorkspace.h"
#include "app/services/Workflow.h"
#include "core/PointCloud.h"

namespace mllm
{
enum class CloudOperation
{
    PlaneFit,
    Voxel,
    Crop,
    ExtractSelection,
    Export,
    SelectProjected
};
struct CloudRequest
{
    CloudSnapshot input;
    CloudOperation operation = CloudOperation::PlaneFit;
    double voxelMm = 0.5, minimum = 0, maximum = 1, inlierMm = 0.5;
    int axis = 2;
    Path exportPath;
    CloudProjection projection;
    cv::Rect2d rectangle;
};
JobResult loadCloudWorkflow(const Path& file, const TaskContext& task);
JobResult pointCloudWorkflow(const CloudRequest& request, const TaskContext& task);
} // namespace mllm
