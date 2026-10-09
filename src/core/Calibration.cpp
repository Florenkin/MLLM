#include "core/Calibration.h"
#include "core/IO.h"
#include "core/Laser.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

namespace mllm
{
std::vector<cv::Point3f> boardPoints(const BoardConfig& b)
{
    if (b.corners.width < 3 || b.corners.height < 3 || b.corners.area() > 2000 ||
        !std::isfinite(b.squareMm) || b.squareMm <= 0)
        throw std::runtime_error("棋盘格内角点至少 3×3，格长必须为正数");
    std::vector<cv::Point3f> points;
    for (int y = 0; y < b.corners.height; ++y)
        for (int x = 0; x < b.corners.width; ++x)
            points.emplace_back(static_cast<float>(x * b.squareMm), static_cast<float>(y * b.squareMm), 0.0F);
    return points;
}
bool detectBoard(const cv::Mat& image, const BoardConfig& b, std::vector<cv::Point2f>& corners)
{
    boardPoints(b);
    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image;
    if (!cv::findChessboardCorners(gray, b.corners, corners,
                                   cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE))
        return false;
    cv::cornerSubPix(gray, corners, {5, 5}, {-1, -1},
                     {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 40, 0.001});
    return true;
}
CameraCalibration calibrateCamera(const Path& directory, const BoardConfig& b, const TaskContext& task)
{
    const auto objects = boardPoints(b);
    const auto files = imageFiles(directory);
    CameraCalibration result;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    for (std::size_t i = 0; i < files.size(); ++i)
    {
        task.report(static_cast<int>(80 * i / files.size()), "检测棋盘格：" + files[i].filename().u8string());
        try
        {
            const auto image = readImage(files[i]);
            std::vector<cv::Point2f> corners;
            if (!detectBoard(image, b, corners))
            {
                result.rejected.push_back(files[i].filename().u8string() + ": 未检测到完整角点");
                continue;
            }
            if (result.imageSize.area() == 0)
                result.imageSize = image.size();
            if (image.size() != result.imageSize)
            {
                result.rejected.push_back(files[i].filename().u8string() + ": 图像尺寸不一致");
                continue;
            }
            imagePoints.push_back(corners);
            objectPoints.push_back(objects);
            result.files.push_back(files[i].filename().u8string());
        }
        catch (const cv::Exception& error)
        {
            result.rejected.push_back(files[i].filename().u8string() + ": " + error.what());
        }
        catch (const std::runtime_error& error)
        {
            result.rejected.push_back(files[i].filename().u8string() + ": " + error.what());
        }
    }
    if (imagePoints.size() < 5)
        throw std::runtime_error("有效棋盘图不足 5 张；建议使用 15–25 张不同位置和倾角的图像");
    task.report(85, "求解相机内参和畸变");
    std::vector<cv::Mat> rvecs, tvecs;
    result.rms =
        cv::calibrateCamera(objectPoints, imagePoints, result.imageSize, result.K, result.D, rvecs, tvecs);
    task.check();
    if (!result.valid() || !std::isfinite(result.rms))
        throw std::runtime_error("相机标定失败或退化");
    for (std::size_t i = 0; i < imagePoints.size(); ++i)
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objects, rvecs[i], tvecs[i], result.K, result.D, projected);
        result.errors.push_back(cv::norm(imagePoints[i], projected, cv::NORM_L2) /
                                std::sqrt(static_cast<double>(objects.size())));
    }
    task.report(100, "相机标定完成");
    return result;
}
namespace
{
PlaneFit leastSquaresPlane(const Cloud& points)
{
    cv::Vec3d center(0, 0, 0);
    for (const auto& p : points)
        center += cv::Vec3d(p.x, p.y, p.z);
    center /= static_cast<double>(points.size());
    cv::Mat covariance = cv::Mat::zeros(3, 3, CV_64F);
    for (const auto& p : points)
    {
        const cv::Mat v = cv::Mat(cv::Vec3d(p.x, p.y, p.z) - center);
        covariance += v * v.t();
    }
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covariance, eigenvalues, eigenvectors);
    if (eigenvalues.at<double>(1) < 1e-8 ||
        eigenvalues.at<double>(1) / std::max(eigenvalues.at<double>(0), 1e-12) < 1e-6)
        throw std::runtime_error("激光点分布近似一条直线，无法确定平面；请改变棋盘深度及倾角");
    cv::Vec3d n(eigenvectors.at<double>(2, 0), eigenvectors.at<double>(2, 1), eigenvectors.at<double>(2, 2));
    if (n[2] < 0)
        n = -n;
    PlaneFit fit;
    fit.plane = {n[0], n[1], n[2], -n.dot(center)};
    fit.inliers = points.size();
    for (const auto& p : points)
    {
        const double d = std::abs(n.dot(cv::Vec3d(p.x, p.y, p.z)) + fit.plane[3]);
        fit.rms += d * d;
        fit.maxError = std::max(fit.maxError, d);
    }
    fit.rms = std::sqrt(fit.rms / points.size());
    return fit;
}
double distance(const cv::Vec4d& plane, const cv::Point3d& p)
{
    return std::abs(plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]);
}
} // namespace
PlaneFit fitPlane(const Cloud& points, double threshold)
{
    if (points.size() < 3 || !std::isfinite(threshold) || threshold <= 0)
        throw std::runtime_error("平面拟合需要至少三个点和正数阈值");
    for (const auto& p : points)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            throw std::runtime_error("点云包含非有限坐标");
    if (points.size() == 3)
        return leastSquaresPlane(points);
    std::mt19937 rng(42);
    std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
    cv::Vec4d best;
    std::size_t count = 0;
    for (int iteration = 0; iteration < 256; ++iteration)
    {
        const auto& p = points[pick(rng)];
        const auto& q = points[pick(rng)];
        const auto& r = points[pick(rng)];
        cv::Vec3d n =
            cv::Vec3d(q.x - p.x, q.y - p.y, q.z - p.z).cross(cv::Vec3d(r.x - p.x, r.y - p.y, r.z - p.z));
        const double norm = cv::norm(n);
        if (norm < 1e-8)
            continue;
        n /= norm;
        const cv::Vec4d plane(n[0], n[1], n[2], -n.dot(cv::Vec3d(p.x, p.y, p.z)));
        std::size_t inliers = 0;
        for (const auto& pt : points)
            if (distance(plane, pt) <= threshold)
                ++inliers;
        if (inliers > count)
        {
            count = inliers;
            best = plane;
        }
        if (count == points.size())
            break;
    }
    if (count < 3 || count < points.size() / 2)
        throw std::runtime_error("无法找到一致平面；请检查配对、中心线和误差阈值");
    Cloud inliers;
    for (const auto& p : points)
        if (distance(best, p) <= threshold)
            inliers.push_back(p);
    return leastSquaresPlane(inliers);
}
LaserPlane calibrateLaserPlane(const Path& off, const Path& on, const CameraCalibration& camera,
                               const BoardConfig& board, const LaserConfig& laser, double position,
                               const TaskContext& task)
{
    if (!camera.valid() || !std::isfinite(position))
        throw std::runtime_error("请先完成或导入相机标定");
    const auto offFiles = imageFiles(off), onFiles = imageFiles(on);
    std::map<std::string, Path> pairs;
    for (const auto& p : onFiles)
        if (!pairs.emplace(p.stem().u8string(), p).second)
            throw std::runtime_error("激光开启目录存在重复文件名主体");
    if (offFiles.size() != onFiles.size())
        throw std::runtime_error("激光开关图像数量不相等；需同名一一配对");
    std::map<std::string, bool> seen;
    Cloud samples;
    int used = 0;
    const auto objects = boardPoints(board);
    for (std::size_t i = 0; i < offFiles.size(); ++i)
    {
        task.report(static_cast<int>(85 * i / offFiles.size()),
                    "恢复棋盘激光点：" + offFiles[i].filename().u8string());
        const auto stem = offFiles[i].stem().u8string();
        if (!seen.emplace(stem, true).second || !pairs.count(stem))
            throw std::runtime_error("开关图像必须具有唯一且相同的文件名主体");
        const auto a = readImage(offFiles[i]), b = readImage(pairs.at(stem));
        if (a.size() != camera.imageSize || b.size() != camera.imageSize)
            throw std::runtime_error("标定图像尺寸必须与相机内参一致");
        std::vector<cv::Point2f> corners;
        if (!detectBoard(a, board, corners))
            continue;
        cv::Mat rvec, tvec, R;
        if (!cv::solvePnP(objects, corners, camera.K, camera.D, rvec, tvec))
            continue;
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objects, rvec, tvec, camera.K, camera.D, projected);
        if (cv::norm(projected, corners) / std::sqrt(static_cast<double>(corners.size())) > 2)
            continue;
        cv::Rodrigues(rvec, R);
        const cv::Vec3d n(R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));
        const cv::Vec3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        cv::Mat difference;
        cv::subtract(b, a, difference); // only added laser intensity.
        const auto line = extractLaser(difference, laser);
        if (line.pixels.empty())
            continue;
        std::vector<cv::Point2d> rays;
        cv::undistortPoints(line.pixels, rays, camera.K, camera.D);
        std::vector<cv::Point2f> hull;
        cv::convexHull(corners, hull);
        Cloud pairSamples;
        for (std::size_t j = 0; j < rays.size(); ++j)
        {
            if (cv::pointPolygonTest(hull, line.pixels[j], false) < 0)
                continue;
            const cv::Vec3d v(rays[j].x, rays[j].y, 1);
            const double denominator = n.dot(v);
            if (std::abs(denominator) < 1e-9)
                continue;
            const double z = n.dot(t) / denominator;
            if (z <= 0 || !std::isfinite(z))
                continue;
            pairSamples.emplace_back(z * v[0], z * v[1], z);
        }
        if (pairSamples.size() >= 10)
        {
            samples.insert(samples.end(), pairSamples.begin(), pairSamples.end());
            ++used;
        }
    }
    if (used < 3 || samples.size() < 30)
        throw std::runtime_error("至少需要 3 组不同棋盘位置的有效激光开关图像，每组至少 10 个激光点");
    task.report(90, "拟合激光平面并剔除异常点（0.5 mm 阈值）");
    const auto fit = fitPlane(samples, 0.5);
    task.check();
    LaserPlane result;
    result.position = position;
    result.coefficients = fit.plane;
    result.rmsMm = fit.rms;
    result.maxErrorMm = fit.maxError;
    result.samples = static_cast<int>(fit.inliers);
    result.pairs = used;
    if (!result.valid() || std::abs(result.coefficients[3]) < 1e-6)
        throw std::runtime_error("激光平面经过相机中心，不能进行三角测量");
    task.report(100, "激光平面标定完成");
    return result;
}
} // namespace mllm
