#include "core/PointCloud.h"
#include "core/IO.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <tuple>

namespace mllm
{
cv::Point2d projectCloudPoint(const cv::Point3d& point, const CloudProjection& projection)
{
    const auto delta = point - projection.center;
    const cv::Vec3d vector(delta.x, delta.y, delta.z);
    return {projection.offset.x + projection.horizontal.dot(vector) * projection.scale,
            projection.offset.y - projection.vertical.dot(vector) * projection.scale};
}
std::vector<std::size_t> selectProjectedCloud(const Cloud& cloud, const CloudProjection& projection,
                                              const cv::Rect2d& rectangle, const TaskContext& task)
{
    task.check();
    if (!std::isfinite(projection.scale) || projection.scale <= 0 || !std::isfinite(projection.center.x) ||
        !std::isfinite(projection.center.y) || !std::isfinite(projection.center.z) ||
        !cv::checkRange(cv::Mat(projection.horizontal)) || !cv::checkRange(cv::Mat(projection.vertical)) ||
        !std::isfinite(projection.offset.x) || !std::isfinite(projection.offset.y) ||
        !std::isfinite(rectangle.x) || !std::isfinite(rectangle.y) || !std::isfinite(rectangle.width) ||
        !std::isfinite(rectangle.height) || rectangle.width < 0 || rectangle.height < 0)
        throw std::runtime_error("点云投影或框选范围无效");
    std::vector<std::size_t> result;
    for (std::size_t i = 0; i < cloud.size(); ++i)
    {
        if (i % 1024 == 0)
            task.report(static_cast<int>(90 * i / cloud.size()), "框选原始点云");
        const auto point = projectCloudPoint(cloud[i], projection);
        if (point.x >= rectangle.x && point.x <= rectangle.x + rectangle.width && point.y >= rectangle.y &&
            point.y <= rectangle.y + rectangle.height)
            result.push_back(i);
    }
    task.check();
    return result;
}
std::string cloudText(const std::string& extension, const Cloud& cloud, const TaskContext& task)
{
    task.check();
    auto ext = extension;
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (cloud.empty())
        throw std::runtime_error("点云为空");
    std::ostringstream text;
    text << std::setprecision(12);
    if (ext == ".pcd")
        text << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
             << cloud.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << cloud.size()
             << "\nDATA ascii\n";
    else if (ext == ".ply")
        text << "ply\nformat ascii 1.0\ncomment units mm\nelement vertex " << cloud.size()
             << "\nproperty double x\nproperty double y\nproperty double z\nend_header\n";
    else if (ext != ".xyz" && ext != ".txt")
        throw std::runtime_error("导出格式应为 XYZ、TXT、PCD 或 PLY");
    std::size_t processed = 0;
    for (const auto& p : cloud)
    {
        if (processed++ % 1024 == 0)
            task.report(static_cast<int>(90 * processed / cloud.size()), "编码点云");
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            throw std::runtime_error("点云包含无效坐标");
        text << p.x << " " << p.y << " " << p.z << "\n";
    }
    task.check();
    return text.str();
}
void exportCloud(const Path& path, const Cloud& cloud, const TaskContext& task)
{
    writeText(path, cloudText(path.extension().u8string(), cloud, task));
}
Cloud importCloud(const Path& path, const TaskContext& task)
{
    task.check();
    std::ifstream data(path, std::ios::binary);
    if (!data)
        throw std::runtime_error("无法读取点云：" + path.u8string());
    std::string line;
    auto ext = path.extension().u8string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    bool header = ext == ".pcd" || ext == ".ply";
    bool formatOk = !header;
    Cloud cloud;
    std::size_t expected = 0;
    std::vector<std::string> plyProperties;
    bool vertexProperties = false;
    bool pcdFields = false;
    std::size_t lines = 0;
    std::error_code sizeError;
    const auto totalBytes = std::filesystem::file_size(path, sizeError);
    while (std::getline(data, line))
    {
        if (lines++ % 1024 == 0)
        {
            const auto position = data.tellg();
            const auto percent =
                !sizeError && totalBytes && position >= 0
                    ? static_cast<int>(90.0 * static_cast<double>(position) / static_cast<double>(totalBytes))
                    : 0;
            task.report(percent, "读取点云");
        }
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (header)
        {
            if (line == "format ascii 1.0" || line == "DATA ascii")
                formatOk = true;
            if (line.rfind("FIELDS ", 0) == 0 && line != "FIELDS x y z")
                throw std::runtime_error("PCD 只支持 x y z 三字段 ASCII 数据");
            if (line == "FIELDS x y z")
                pcdFields = true;
            if (line.rfind("POINTS ", 0) == 0)
                expected = std::stoull(line.substr(7));
            if (line.rfind("element vertex ", 0) == 0)
            {
                expected = std::stoull(line.substr(15));
                vertexProperties = true;
            }
            else if (line.rfind("element ", 0) == 0)
                vertexProperties = false;
            if (vertexProperties && line.rfind("property ", 0) == 0)
            {
                std::istringstream property(line);
                std::string keyword, type, name;
                property >> keyword >> type >> name;
                plyProperties.push_back(name);
            }
            if (line == "end_header" || line.rfind("DATA ", 0) == 0)
            {
                header = false;
                if (!formatOk)
                    throw std::runtime_error("只支持 ASCII PCD/PLY 点云");
                if (ext == ".pcd" && !pcdFields)
                    throw std::runtime_error("PCD 缺少 x y z 字段");
                if (ext == ".ply" && plyProperties != std::vector<std::string>{"x", "y", "z"})
                    throw std::runtime_error("PLY 只支持顺序为 x y z 的三个顶点属性");
            }
            continue;
        }
        if (line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        cv::Point3d point;
        if (!(row >> point.x >> point.y >> point.z) || !std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
            throw std::runtime_error("点云数据行无效");
        cloud.push_back(point);
    }
    task.check();
    if (data.bad())
        throw std::runtime_error("点云文件读取未完成：" + path.u8string());
    if (header || cloud.empty() || (expected && expected != cloud.size()))
        throw std::runtime_error("点云为空、头部不完整或点数不匹配");
    return cloud;
}
Cloud voxelDownsample(const Cloud& cloud, double size, const TaskContext& task)
{
    task.check();
    if (!std::isfinite(size) || size <= 0)
        throw std::runtime_error("体素尺寸必须大于零");
    struct Bin
    {
        cv::Point3d sum{0, 0, 0};
        std::size_t count = 0;
    };
    std::map<std::tuple<long long, long long, long long>, Bin> bins;
    std::size_t processed = 0;
    for (const auto& p : cloud)
    {
        if (processed++ % 1024 == 0)
            task.report(static_cast<int>(85 * processed / cloud.size()), "体素降采样");
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            std::max({std::abs(p.x / size), std::abs(p.y / size), std::abs(p.z / size)}) > 9e18)
            throw std::runtime_error("体素索引超出范围");
        auto& bin = bins[{static_cast<long long>(std::floor(p.x / size)),
                          static_cast<long long>(std::floor(p.y / size)),
                          static_cast<long long>(std::floor(p.z / size))}];
        bin.sum += p;
        ++bin.count;
    }
    Cloud out;
    processed = 0;
    for (const auto& entry : bins)
    {
        if (processed++ % 1024 == 0)
            task.check();
        out.push_back(entry.second.sum * (1.0 / entry.second.count));
    }
    task.check();
    return out;
}
Cloud selectCloud(const Cloud& cloud, const std::vector<std::size_t>& indices, const TaskContext& task)
{
    task.check();
    Cloud result;
    result.reserve(indices.size());
    std::size_t processed = 0;
    for (const auto index : indices)
    {
        if (processed++ % 1024 == 0)
            task.check();
        if (index >= cloud.size())
            throw std::runtime_error("点云选区索引超出范围");
        result.push_back(cloud[index]);
    }
    task.check();
    return result;
}
Cloud cropCloud(const Cloud& cloud, int axis, double minimum, double maximum, const TaskContext& task)
{
    task.check();
    if (axis < 0 || axis > 2 || !std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum)
        throw std::runtime_error("裁剪轴或范围无效");
    Cloud result;
    std::size_t processed = 0;
    for (const auto& point : cloud)
    {
        if (processed++ % 1024 == 0)
            task.report(static_cast<int>(90 * processed / cloud.size()), "按轴裁剪点云");
        const double value = axis == 0 ? point.x : axis == 1 ? point.y : point.z;
        if (value >= minimum && value <= maximum)
            result.push_back(point);
    }
    task.check();
    return result;
}
std::pair<cv::Point3d, cv::Point3d> cloudBounds(const Cloud& cloud)
{
    if (cloud.empty())
        throw std::runtime_error("点云为空");
    auto lo = cloud.front(), hi = lo;
    for (const auto& p : cloud)
    {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
        hi.z = std::max(hi.z, p.z);
    }
    return {lo, hi};
}
} // namespace mllm
