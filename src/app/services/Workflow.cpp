#include "app/services/Workflow.h"
#include "core/Calibration.h"
#include "core/IO.h"
#include "core/PointCloud.h"
#include "core/Reconstruction.h"
#include <algorithm>
#include <sstream>
namespace mllm
{
namespace
{
template <class F> JobResult guarded(int kind, F work)
{
    JobResult result;
    result.kind = kind;
    try
    {
        work(result);
        result.success = true;
    }
    catch (const Cancelled& e)
    {
        result.cancelled = true;
        result.message = e.what();
    }
    catch (const std::exception& e)
    {
        result.message = e.what();
    }
    return result;
}
} // namespace
JobResult cameraWorkflow(const Path& images, const Path& output, const BoardConfig& board,
                         const TaskContext& task)
{
    return guarded(0,
                   [&](JobResult& r)
                   {
                       r.calibration.camera = calibrateCamera(images, board, task);
                       task.check();
                       r.directory = uniqueDirectory(output, "camera_calibration");
                       saveCalibration(r.directory / "calibration.yml", r.calibration);
                       std::ostringstream report;
                       report << "file,rms_px\n";
                       for (std::size_t i = 0; i < r.calibration.camera.files.size(); ++i)
                           report << '"' << r.calibration.camera.files[i] << "\","
                                  << r.calibration.camera.errors[i] << "\n";
                       writeText(r.directory / "errors.csv", report.str());
                       std::ostringstream rejected;
                       for (const auto& message : r.calibration.camera.rejected)
                           rejected << message << "\n";
                       writeText(r.directory / "rejected.txt", rejected.str());
                       r.message = "相机标定完成，RMS = " + std::to_string(r.calibration.camera.rms) + " px";
                   });
}
JobResult laserWorkflow(const Path& off, const Path& on, const Path& output, Calibration calibration,
                        const BoardConfig& board, const LaserConfig& laser, double position,
                        const TaskContext& task)
{
    return guarded(
        1,
        [&](JobResult& r)
        {
            const auto plane = calibrateLaserPlane(off, on, calibration.camera, board, laser, position, task);
            calibration.planes.erase(std::remove_if(calibration.planes.begin(), calibration.planes.end(),
                                                    [&](const LaserPlane& p)
                                                    { return std::abs(p.position - position) < 1e-6; }),
                                     calibration.planes.end());
            calibration.planes.push_back(plane);
            std::sort(calibration.planes.begin(), calibration.planes.end(),
                      [](const LaserPlane& a, const LaserPlane& b) { return a.position < b.position; });
            task.check();
            r.calibration = calibration;
            r.directory = uniqueDirectory(output, "laser_calibration");
            saveCalibration(r.directory / "calibration.yml", r.calibration);
            r.message = "激光面标定完成，RMS = " + std::to_string(plane.rmsMm) + " mm，有效图像对 " +
                        std::to_string(plane.pairs);
        });
}
JobResult reconstructionWorkflow(const Path& images, const Path& output, const Calibration& calibration,
                                 const ReconstructionConfig& config, const TaskContext& task)
{
    return guarded(2,
                   [&](JobResult& r)
                   {
                       r.batch = reconstructBatch(images, output, calibration, config, task);
                       r.directory = r.batch.directory;
                       r.message = "重建完成：" + std::to_string(r.batch.frames) + " 帧，" +
                                   std::to_string(r.batch.points.size()) + " 点；失败 " +
                                   std::to_string(r.batch.failed) + " 帧";
                   });
}
JobResult singleFrameWorkflow(const Path& image, const Path& output, const Calibration& calibration,
                              const ReconstructionConfig& config, const TaskContext& task)
{
    return guarded(
        2,
        [&](JobResult& result)
        {
            task.check();
            const auto frame = reconstructFrame(readImage(image), calibration.camera,
                                                planeAt(calibration, config.positionStart), config);
            if (frame.points.empty())
                throw std::runtime_error("未获得有效激光交点");
            task.check();
            result.directory = uniqueDirectory(output, "single_frame");
            result.batch.directory = result.directory;
            result.batch.points = frame.points;
            result.batch.frames = 1;
            saveCalibration(result.directory / "calibration.yml", calibration);
            exportCloud(result.directory / "cloud.xyz", frame.points);
            exportCloud(result.directory / "cloud.pcd", frame.points);
            exportCloud(result.directory / "cloud.ply", frame.points);
            writeText(result.directory / "parameters.txt",
                      reconstructionParameters(config) + "source=" + image.u8string() + "\n");
            if (config.saveDiagnostics)
            {
                const auto diagnostic = result.directory / "laser_lines/line.png";
                writeImage(diagnostic, frame.line.preview);
                result.batch.diagnostics.push_back(diagnostic);
            }
            task.check();
            writeText(result.directory / "completed.txt", "Single frame reconstruction completed.\n");
            result.message = "单帧重建完成：" + std::to_string(frame.points.size()) + " 点";
        });
}
} // namespace mllm
