#include "core/CaptureManifest.h"
#include "core/IO.h"
#include <algorithm>
#include <charconv>
#include <map>
#include <set>

namespace mllm
{
namespace
{
std::uint64_t number(const std::string& text)
{
    std::uint64_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != std::errc() || parsed.ptr != text.data() + text.size())
        throw std::runtime_error("采集记录中的帧号或索引无效");
    return value;
}
void validateRecord(const CaptureRecord& record)
{
    const Path relative = std::filesystem::u8path(record.file);
    if (relative.empty() || relative.has_root_path() ||
        relative.parent_path().generic_u8string() != captureFolder(record.category) ||
        relative.filename().empty() || !record.captureIndex || record.frame.deviceId.empty() ||
        record.frame.hostTimeMs <= 0 || record.frame.lostPackets || record.scanMode < 0 ||
        record.scanMode > 2 || record.axis < 0 || record.axis > 2 || record.frame.streamId.empty() ||
        !std::isfinite(record.frame.requestedExposureUs) || !std::isfinite(record.frame.requestedGain) ||
        !std::isfinite(record.planePosition) || !std::isfinite(record.translationMm) ||
        (record.positionSource != "planned" && record.positionSource != "manual"))
        throw std::runtime_error("采集记录中的路径、元数据或扫描位置无效");
    if ((record.category == CaptureCategory::LaserOff || record.category == CaptureCategory::LaserOn) &&
        !record.pairId)
        throw std::runtime_error("激光图像缺少配对号");
}
CaptureCategory parseCategory(const std::string& text)
{
    for (auto category : {CaptureCategory::Camera, CaptureCategory::LaserOff, CaptureCategory::LaserOn,
                          CaptureCategory::Reconstruction})
        if (text == captureFolder(category))
            return category;
    throw std::runtime_error("未知采集类别");
}
std::optional<std::uint64_t> trailingIndex(const Path& file)
{
    const auto stem = file.stem().u8string();
    auto first = stem.size();
    while (first && stem[first - 1] >= '0' && stem[first - 1] <= '9')
        --first;
    if (first == stem.size())
        return {};
    return number(stem.substr(first));
}
} // namespace
Path findCaptureManifest(const Path& images)
{
    auto directory = std::filesystem::weakly_canonical(images);
    if (directory.filename().empty())
        directory = directory.parent_path();
    const auto candidate = directory.parent_path() / "capture_manifest.yml";
    return std::filesystem::exists(candidate) ? candidate : Path{};
}
const char* captureFolder(CaptureCategory category)
{
    switch (category)
    {
    case CaptureCategory::Camera:
        return "camera";
    case CaptureCategory::LaserOff:
        return "off";
    case CaptureCategory::LaserOn:
        return "on";
    case CaptureCategory::Reconstruction:
        return "reconstruction";
    }
    throw std::runtime_error("未知采集类别");
}
std::string captureManifestText(const CaptureManifest& manifest)
{
    cv::FileStorage file(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
    file << "schema" << 1 << "units" << "mm" << "session_id" << manifest.sessionId << "device_id"
         << manifest.deviceId;
    file << "image_width" << manifest.imageSize.width << "image_height" << manifest.imageSize.height
         << "records" << "[";
    for (const auto& record : manifest.records)
    {
        validateRecord(record);
        file << "{" << "file" << record.file << "category" << captureFolder(record.category)
             << "capture_index" << std::to_string(record.captureIndex) << "pair_id"
             << std::to_string(record.pairId) << "frame_number" << std::to_string(record.frame.frameNumber)
             << "host_time_ms" << std::to_string(record.frame.hostTimeMs) << "device_frame_number"
             << static_cast<int>(record.frame.deviceFrameNumber) << "device_id" << record.frame.deviceId
             << "stream_id" << record.frame.streamId << "lost_packets"
             << static_cast<int>(record.frame.lostPackets) << "requested_exposure_us"
             << record.frame.requestedExposureUs << "requested_gain" << record.frame.requestedGain
             << "scan_mode" << record.scanMode << "axis" << record.axis << "plane_position"
             << record.planePosition << "translation_mm" << record.translationMm << "position_source"
             << record.positionSource << "}";
    }
    file << "]";
    return file.releaseAndGetString();
}
CaptureManifest loadCaptureManifest(const Path& path)
{
    cv::FileStorage file(readText(path), cv::FileStorage::READ | cv::FileStorage::MEMORY);
    int schema = 0;
    std::string units;
    file["schema"] >> schema;
    file["units"] >> units;
    if (schema != 1 || units != "mm" || !file["records"].isSeq())
        throw std::runtime_error("采集记录格式无效");
    CaptureManifest result;
    file["session_id"] >> result.sessionId;
    file["device_id"] >> result.deviceId;
    file["image_width"] >> result.imageSize.width;
    file["image_height"] >> result.imageSize.height;
    if (result.sessionId.empty() || result.deviceId.empty() || result.imageSize.width <= 0 ||
        result.imageSize.height <= 0)
        throw std::runtime_error("采集记录缺少会话或相机信息");
    std::set<std::string> files;
    std::set<std::pair<CaptureCategory, std::size_t>> indices;
    for (const auto& node : file["records"])
    {
        CaptureRecord record;
        std::string category, index, pair, frame, time;
        node["file"] >> record.file;
        node["category"] >> category;
        record.category = parseCategory(category);
        node["capture_index"] >> index;
        node["pair_id"] >> pair;
        node["frame_number"] >> frame;
        node["host_time_ms"] >> time;
        record.captureIndex = static_cast<std::size_t>(number(index));
        record.pairId = static_cast<std::size_t>(number(pair));
        record.frame.frameNumber = number(frame);
        const auto timestamp = number(time);
        if (timestamp > static_cast<std::uint64_t>(INT64_MAX))
            throw std::runtime_error("采集时间超出范围");
        record.frame.hostTimeMs = static_cast<std::int64_t>(timestamp);
        int deviceNumber = 0;
        node["device_frame_number"] >> deviceNumber;
        record.frame.deviceFrameNumber = deviceNumber != 0;
        node["device_id"] >> record.frame.deviceId;
        node["requested_exposure_us"] >> record.frame.requestedExposureUs;
        node["stream_id"] >> record.frame.streamId;
        int lost = 0;
        node["lost_packets"] >> lost;
        if (lost < 0)
            throw std::runtime_error("采集记录丢包数无效");
        record.frame.lostPackets = static_cast<unsigned int>(lost);
        node["requested_gain"] >> record.frame.requestedGain;
        node["scan_mode"] >> record.scanMode;
        node["axis"] >> record.axis;
        node["plane_position"] >> record.planePosition;
        node["translation_mm"] >> record.translationMm;
        node["position_source"] >> record.positionSource;
        validateRecord(record);
        if (record.frame.deviceId != result.deviceId || !files.insert(record.file).second ||
            !indices.insert({record.category, record.captureIndex}).second)
            throw std::runtime_error("采集记录包含相机不一致或重复帧");
        result.records.push_back(std::move(record));
    }
    return result;
}
std::vector<FramePlan> reconstructionFrames(const Path& images, const ReconstructionConfig& config,
                                            std::vector<std::string>& warnings, const TaskContext& task)
{
    const auto files = imageFiles(images);
    std::vector<FramePlan> result;
    const auto manifestPath = findCaptureManifest(images);
    if (!manifestPath.empty())
    {
        if (!std::filesystem::equivalent(images, manifestPath.parent_path() / "reconstruction"))
            throw std::runtime_error("请选择本会话的 reconstruction 图像目录");
        const auto manifest = loadCaptureManifest(manifestPath);
        std::map<std::string, Path> available;
        for (const auto& file : files)
            available[file.filename().u8string()] = file;
        std::set<std::size_t> sourceIndices;
        for (const auto& record : manifest.records)
        {
            task.check();
            if (record.category != CaptureCategory::Reconstruction)
                continue;
            const auto name = std::filesystem::u8path(record.file).filename().u8string();
            if (!available.count(name))
                throw std::runtime_error("采集图像缺失：" + name);
            if (record.scanMode != config.mode || (config.mode == 1 && record.axis != config.axis))
                throw std::runtime_error("重建扫描模型或坐标轴与采集记录不一致");
            sourceIndices.insert(record.captureIndex);
            result.push_back(
                {available.at(name), record.captureIndex, record.planePosition, record.translationMm});
            available.erase(name);
        }
        if (result.empty() || !available.empty())
            throw std::runtime_error("图像与采集记录不一致，存在未登记文件");
        for (std::size_t i = 1; i <= result.size(); ++i)
            if (!sourceIndices.count(i))
                throw std::runtime_error("采集记录的保存序号不连续");
        std::sort(result.begin(), result.end(),
                  [](const auto& a, const auto& b) { return a.sourceIndex < b.sourceIndex; });
    }
    else
    {
        if (config.mode == 1 && std::abs(config.stepMm) < 1e-12)
            throw std::runtime_error("没有采集记录时，平移扫描的有符号步距不能为零");
        if (config.mode != 0)
        {
            warnings.push_back("输入没有采集记录，扫描位置按文件顺序推算；请确认运动位置对应关系。");
            std::optional<std::uint64_t> previous;
            for (const auto& file : files)
            {
                const auto index = trailingIndex(file);
                if (index && previous && (*index <= *previous || *index - *previous != 1))
                    throw std::runtime_error("扫描图像序号存在缺帧或重复，请补充采集记录后重建");
                previous = index;
            }
        }
        for (std::size_t i = 0; i < files.size(); ++i)
            result.push_back(
                {files[i], i,
                 config.positionStart + (config.mode == 2 ? static_cast<double>(i) * config.positionStep : 0),
                 config.mode == 1 ? static_cast<double>(i) * config.stepMm : 0});
    }
    return result;
}
void validateLaserPairManifest(const Path& off, const Path& on)
{
    const auto manifestPath = findCaptureManifest(off);
    const auto other = findCaptureManifest(on);
    if (manifestPath.empty() && other.empty())
        return;
    if (manifestPath.empty() || other.empty() || !std::filesystem::equivalent(manifestPath, other))
        throw std::runtime_error("激光开关图像必须来自同一采集会话");
    const auto manifest = loadCaptureManifest(manifestPath);
    if (!std::filesystem::equivalent(off, manifestPath.parent_path() / "off") ||
        !std::filesystem::equivalent(on, manifestPath.parent_path() / "on"))
        throw std::runtime_error("激光开关目录与采集记录不一致");
    std::map<std::size_t, std::pair<std::string, std::string>> pairs;
    for (const auto& record : manifest.records)
    {
        if (record.category == CaptureCategory::LaserOff || record.category == CaptureCategory::LaserOn)
        {
            auto& entry = record.category == CaptureCategory::LaserOff ? pairs[record.pairId].first
                                                                       : pairs[record.pairId].second;
            if (!entry.empty())
                throw std::runtime_error("激光配对号重复");
            entry = std::filesystem::u8path(record.file).filename().u8string();
        }
    }
    std::set<std::string> expected;
    for (const auto& pair : pairs)
    {
        if (pair.second.first.empty() || pair.second.first != pair.second.second)
            throw std::runtime_error("激光采集记录存在未完成或错误配对");
        expected.insert(pair.second.first);
    }
    const auto a = imageFiles(off), b = imageFiles(on);
    if (expected.empty() || a.size() != expected.size() || b.size() != expected.size())
        throw std::runtime_error("激光图像与采集记录不一致");
    for (const auto& file : a)
        if (!expected.count(file.filename().u8string()))
            throw std::runtime_error("存在未登记的激光关闭图像");
    for (const auto& file : b)
        if (!expected.count(file.filename().u8string()))
            throw std::runtime_error("存在未登记的激光开启图像");
}
} // namespace mllm
