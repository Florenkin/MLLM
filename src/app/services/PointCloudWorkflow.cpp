#include "app/services/PointCloudWorkflow.h"
#include "core/Calibration.h"
#include "core/PointCloud.h"
#include <QSaveFile>
#include <QString>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace mllm
{
JobResult loadCloudWorkflow(const Path& file, const TaskContext& task)
{
    return executeJob(JobKind::CloudLoad, task,
                      [&](JobResult& result, const TaskContext& current)
                      {
                          result.cloud = std::make_shared<const Cloud>(importCloud(file, current));
                          result.documentName = file.filename().u8string();
                          result.message = "点云已加载：" + std::to_string(result.cloud->size()) + " 点";
                      });
}
JobResult pointCloudWorkflow(const CloudRequest& request, const TaskContext& task)
{
    const auto kind = request.operation == CloudOperation::PlaneFit          ? JobKind::CloudFit
                      : request.operation == CloudOperation::Export          ? JobKind::CloudExport
                      : request.operation == CloudOperation::SelectProjected ? JobKind::CloudSelect
                                                                             : JobKind::CloudDerived;
    return executeJob(
        kind, task,
        [&](JobResult& result, const TaskContext& current)
        {
            if (!request.input.points || request.input.points->empty())
                throw std::runtime_error("点云为空");
            Cloud selected;
            const Cloud* points = request.input.points.get();
            if (request.input.selectedOnly || request.operation == CloudOperation::ExtractSelection)
            {
                if (request.input.selection.empty())
                    throw std::runtime_error("请先框选点云");
                selected = selectCloud(*points, request.input.selection, current);
                points = &selected;
            }
            result.sourceDocumentId = request.input.documentId;
            if (request.operation == CloudOperation::SelectProjected)
            {
                result.selection = std::make_shared<const std::vector<std::size_t>>(
                    selectProjectedCloud(*points, request.projection, request.rectangle, current));
                result.message = "框选完成：" + std::to_string(result.selection->size()) + " 个原始点";
            }
            else if (request.operation == CloudOperation::PlaneFit)
            {
                const auto fit = fitPlane(*points, request.inlierMm, current);
                current.check();
                std::ostringstream report;
                report << std::setprecision(6) << "平面 [a b c d] = " << fit.plane << "；内点 " << fit.inliers
                       << " / " << points->size() << "；RMS " << fit.rms << " mm，最大内点残差 "
                       << fit.maxError << " mm（阈值 " << request.inlierMm << " mm）";
                result.message = report.str();
            }
            else if (request.operation == CloudOperation::Export)
            {
                const auto text = cloudText(request.exportPath.extension().u8string(), *points, current);
                current.check();
                QSaveFile output(QString::fromStdWString(request.exportPath.wstring()));
                if (!output.open(QIODevice::WriteOnly))
                    throw std::runtime_error("无法创建点云导出文件");
                for (std::size_t offset = 0; offset < text.size();)
                {
                    current.report(90 + static_cast<int>(9.0 * offset / text.size()), "写入点云");
                    current.check();
                    const auto count =
                        static_cast<qint64>(std::min<std::size_t>(65536, text.size() - offset));
                    if (output.write(text.data() + offset, count) != count)
                        throw std::runtime_error("点云导出写入失败");
                    offset += static_cast<std::size_t>(count);
                }
                current.check();
                if (!output.commit())
                    throw std::runtime_error("点云导出提交失败");
                result.message = "点云已导出：" + request.exportPath.u8string();
            }
            else
            {
                Cloud derived;
                if (request.operation == CloudOperation::Voxel)
                {
                    derived = voxelDownsample(*points, request.voxelMm, current);
                    result.documentName = "体素降采样";
                }
                else if (request.operation == CloudOperation::Crop)
                {
                    derived = cropCloud(*points, request.axis, request.minimum, request.maximum, current);
                    result.documentName = "按轴裁剪";
                }
                else
                {
                    derived = std::move(selected);
                    result.documentName = "选区点云";
                }
                current.check();
                if (derived.empty())
                    throw std::runtime_error("处理后点云为空，原始点云已保留");
                result.cloud = std::make_shared<const Cloud>(std::move(derived));
                result.message = result.documentName + "完成：" + std::to_string(result.cloud->size()) +
                                 " 点，原始点云已保留";
            }
        });
}
} // namespace mllm
