#include "core/IO.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <sstream>

namespace mllm
{
bool CameraCalibration::valid() const
{
    return K.rows == 3 && K.cols == 3 && K.type() == CV_64F && cv::checkRange(K) && K.at<double>(0, 0) > 0 &&
           K.at<double>(1, 1) > 0 && std::abs(cv::determinant(K)) > 1e-12 && !D.empty() &&
           D.type() == CV_64F && (D.rows == 1 || D.cols == 1) && std::abs(K.at<double>(2, 0)) < 1e-12 &&
           std::abs(K.at<double>(2, 1)) < 1e-12 && std::abs(K.at<double>(2, 2) - 1) < 1e-12 &&
           (D.total() == 4 || D.total() == 5 || D.total() == 8 || D.total() == 12 || D.total() == 14) &&
           cv::checkRange(D) && imageSize.width > 0 && imageSize.height > 0;
}
bool LaserPlane::valid() const
{
    const double n = cv::norm(cv::Vec3d(coefficients[0], coefficients[1], coefficients[2]));
    return std::isfinite(n) && n > 1e-9 && std::isfinite(coefficients[3]);
}
namespace
{
bool naturalLess(const Path& a, const Path& b)
{
    const auto x = a.filename().u8string(), y = b.filename().u8string();
    std::size_t i = 0, j = 0;
    while (i < x.size() && j < y.size())
    {
        if (std::isdigit(static_cast<unsigned char>(x[i])) && std::isdigit(static_cast<unsigned char>(y[j])))
        {
            auto ei = i, ej = j;
            while (ei < x.size() && std::isdigit(static_cast<unsigned char>(x[ei])))
                ++ei;
            while (ej < y.size() && std::isdigit(static_cast<unsigned char>(y[ej])))
                ++ej;
            auto ni = i, nj = j;
            while (ni < ei && x[ni] == '0')
                ++ni;
            while (nj < ej && y[nj] == '0')
                ++nj;
            if (ei - ni != ej - nj)
                return ei - ni < ej - nj;
            const int cmp = x.compare(ni, ei - ni, y, nj, ej - nj);
            if (cmp)
                return cmp < 0;
            i = ei;
            j = ej;
        }
        else
        {
            const auto cx = std::tolower(static_cast<unsigned char>(x[i])),
                       cy = std::tolower(static_cast<unsigned char>(y[j]));
            if (cx != cy)
                return cx < cy;
            ++i;
            ++j;
        }
    }
    return i == x.size() && j != y.size() ? true : (i == x.size() && j == y.size() && x < y);
}
} // namespace
std::vector<Path> imageFiles(const Path& directory)
{
    if (!std::filesystem::is_directory(directory))
        throw std::runtime_error("图像目录不存在：" + directory.u8string());
    std::vector<Path> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
    {
        auto ext = entry.path().extension().u8string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (entry.is_regular_file() && (ext == ".png" || ext == ".bmp" || ext == ".jpg" || ext == ".jpeg" ||
                                        ext == ".tif" || ext == ".tiff"))
            files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end(), naturalLess);
    if (files.empty())
        throw std::runtime_error("目录中没有支持的图像");
    return files;
}
std::string readText(const Path& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw std::runtime_error("无法读取：" + path.u8string());
    return std::string(std::istreambuf_iterator<char>(file), {});
}
void writeText(const Path& path, const std::string& text)
{
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    if (std::filesystem::exists(path))
        throw std::runtime_error("目标文件已存在：" + path.u8string());
    std::ofstream file(path, std::ios::binary);
    if (!file || !file.write(text.data(), static_cast<std::streamsize>(text.size())))
        throw std::runtime_error("无法写入：" + path.u8string());
    file.close();
    if (!file)
        throw std::runtime_error("写入未完成：" + path.u8string());
}
cv::Mat readImage(const Path& path, int mode)
{
    const auto data = readText(path);
    const std::vector<uchar> bytes(data.begin(), data.end());
    cv::Mat result = cv::imdecode(bytes, mode);
    if (result.empty())
        throw std::runtime_error("无效图像：" + path.u8string());
    return result;
}
void writeImage(const Path& path, const cv::Mat& image)
{
    std::vector<uchar> bytes;
    if (!cv::imencode(path.extension().u8string(), image, bytes))
        throw std::runtime_error("图像编码失败");
    writeText(path, std::string(bytes.begin(), bytes.end()));
}
Path uniqueDirectory(const Path& root, const std::string& prefix)
{
    std::filesystem::create_directories(root);
    const auto tick = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
    for (int i = 0; i < 1000; ++i)
    {
        const auto path = root / (prefix + "_" + std::to_string(tick) + "_" + std::to_string(i));
        if (std::filesystem::create_directory(path))
            return path;
    }
    throw std::runtime_error("无法创建独立结果目录");
}
std::string calibrationYaml(const Calibration& c)
{
    if (!c.camera.valid())
        throw std::runtime_error("相机标定无效");
    cv::FileStorage f(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
    f << "schema" << 1 << "medium" << "air" << "units" << "mm";
    f << "image_width" << c.camera.imageSize.width << "image_height" << c.camera.imageSize.height;
    f << "camera_matrix" << c.camera.K << "distortion" << c.camera.D << "camera_rms_px" << c.camera.rms;
    f << "image_errors_px" << c.camera.errors << "camera_images" << c.camera.files << "rejected_images"
      << c.camera.rejected;
    f << "laser_planes" << "[";
    for (const auto& p : c.planes)
    {
        if (!p.valid())
            throw std::runtime_error("激光平面无效");
        f << "{" << "position" << p.position << "coefficients" << cv::Mat(p.coefficients) << "rms_mm"
          << p.rmsMm << "max_error_mm" << p.maxErrorMm << "samples" << p.samples << "pairs" << p.pairs << "}";
    }
    f << "]";
    return f.releaseAndGetString();
}
Calibration loadCalibration(const Path& path)
{
    cv::FileStorage f(readText(path), cv::FileStorage::READ | cv::FileStorage::MEMORY);
    Calibration c;
    if (!f["camera_matrix"].empty())
    {
        int schema = 0;
        std::string medium, units;
        f["schema"] >> schema;
        f["medium"] >> medium;
        f["units"] >> units;
        if (schema != 1 || medium != "air" || units != "mm")
            throw std::runtime_error("不支持的标定版本、介质或单位");
        f["camera_matrix"] >> c.camera.K;
        f["distortion"] >> c.camera.D;
        f["image_width"] >> c.camera.imageSize.width;
        f["image_height"] >> c.camera.imageSize.height;
        f["camera_rms_px"] >> c.camera.rms;
        f["image_errors_px"] >> c.camera.errors;
        f["camera_images"] >> c.camera.files;
        f["rejected_images"] >> c.camera.rejected;
        for (const auto& node : f["laser_planes"])
        {
            LaserPlane p;
            cv::Mat m;
            node["position"] >> p.position;
            node["coefficients"] >> m;
            if (m.total() != 4)
                throw std::runtime_error("平面参数数量必须为 4");
            m.reshape(1, 1).convertTo(m, CV_64F);
            for (int i = 0; i < 4; ++i)
                p.coefficients[i] = m.at<double>(0, i);
            const double n = cv::norm(cv::Vec3d(p.coefficients[0], p.coefficients[1], p.coefficients[2]));
            if (!p.valid() || !std::isfinite(p.position))
                throw std::runtime_error("无效平面参数");
            p.coefficients /= n;
            node["rms_mm"] >> p.rmsMm;
            node["max_error_mm"] >> p.maxErrorMm;
            node["samples"] >> p.samples;
            node["pairs"] >> p.pairs;
            for (const auto& other : c.planes)
                if (std::abs(other.position - p.position) < 1e-6)
                    throw std::runtime_error("平面表位置重复");
            c.planes.push_back(p);
        }
    }
    else
    {
        // UWScan2 camera compatibility only: its underwater surfaces are not air planes.
        f["CameraMatrix"] >> c.camera.K;
        f["DistCoeffs"] >> c.camera.D;
        f["picSize"] >> c.camera.imageSize;
    }
    c.camera.K.convertTo(c.camera.K, CV_64F);
    c.camera.D.convertTo(c.camera.D, CV_64F);
    if (!c.camera.valid())
        throw std::runtime_error("相机内参、畸变或图像尺寸无效");
    return c;
}
void saveCalibration(const Path& path, const Calibration& c)
{
    writeText(path, calibrationYaml(c));
}
} // namespace mllm
