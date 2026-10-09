#include "core/PointCloud.h"
#include "core/IO.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <tuple>

namespace mllm
{
std::string cloudText(const std::string& extension, const Cloud& cloud)
{
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
    for (const auto& p : cloud)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            throw std::runtime_error("点云包含无效坐标");
        text << p.x << " " << p.y << " " << p.z << "\n";
    }
    return text.str();
}
void exportCloud(const Path& path, const Cloud& cloud)
{
    writeText(path, cloudText(path.extension().u8string(), cloud));
}
Cloud importCloud(const Path& path)
{
    std::istringstream data(readText(path));
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
    while (std::getline(data, line))
    {
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
    if (header || cloud.empty() || (expected && expected != cloud.size()))
        throw std::runtime_error("点云为空、头部不完整或点数不匹配");
    return cloud;
}
Cloud voxelDownsample(const Cloud& cloud, double size)
{
    if (!std::isfinite(size) || size <= 0)
        throw std::runtime_error("体素尺寸必须大于零");
    struct Bin
    {
        cv::Point3d sum{0, 0, 0};
        std::size_t count = 0;
    };
    std::map<std::tuple<long long, long long, long long>, Bin> bins;
    for (const auto& p : cloud)
    {
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
    for (const auto& entry : bins)
        out.push_back(entry.second.sum * (1.0 / entry.second.count));
    return out;
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
