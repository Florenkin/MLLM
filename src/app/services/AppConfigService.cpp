#include "app/services/AppConfigService.h"
#include "app/services/ConfigFile.h"
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <cmath>
#include <map>

namespace mllm
{
namespace
{
bool convert(const ParameterSpec& spec, const QVariant& source, QVariant& output)
{
    bool valid = true;
    switch (spec.defaultValue.type())
    {
    case QVariant::Int:
    {
        const auto number = source.toString().toLongLong(&valid);
        valid = valid && number >= spec.minimum && number <= spec.maximum;
        if (valid)
            output = static_cast<int>(number);
        break;
    }
    case QVariant::Double:
    {
        const auto number = source.toString().toDouble(&valid);
        valid = valid && std::isfinite(number) && number >= spec.minimum && number <= spec.maximum;
        if (valid)
            output = number;
        break;
    }
    case QVariant::Bool:
    {
        const auto text = source.toString().toLower();
        valid = text == "true" || text == "false" || text == "1" || text == "0";
        if (valid)
            output = text == "true" || text == "1";
        break;
    }
    default:
        output = source.toString();
        valid = !output.toString().contains('\n') && !output.toString().contains('\r');
        break;
    }
    return valid;
}
QString category(const QString& key)
{
    return key.section('/', 0, 0) + ".ini";
}
QVariant legacyValue(const ConfigFile& file, const QSettings& decoded, const ParameterSpec& spec)
{
    if (spec.defaultValue.type() == QVariant::String)
        return legacyIniString(file.value(spec.key), decoded.value(spec.key).toString());
    return decoded.value(spec.key);
}
} // namespace
const std::vector<ParameterSpec>& AppConfigService::specifications()
{
    static const std::vector<ParameterSpec> specs = {
        {"calibration/columns", 11, 3, 30, "棋盘格内角点列数"},
        {"calibration/rows", 8, 3, 30, "棋盘格内角点行数"},
        {"calibration/square_mm", 15.0, 0.001, 1000, "实测方格边长，mm"},
        {"calibration/position", 0.0, -100000, 100000, "当前激光平面位置"},
        {"laser/method", 0, 0, 1, "0 灰度重心，1 Steger"},
        {"laser/channel", 0, 0, 3, "0 灰度，1 红，2 绿，3 蓝"},
        {"laser/direction", 0, 0, 1, "0 垂直，1 水平"},
        {"laser/threshold", 100, 0, 255, "激光强度阈值"},
        {"laser/width", 3.0, 0.5, 30, "Steger 条纹宽度，px"},
        {"laser/roi_x", 0, 0, 100000, "ROI 左上角 X，px"},
        {"laser/roi_y", 0, 0, 100000, "ROI 左上角 Y，px"},
        {"laser/roi_w", 0, 0, 100000, "ROI 宽，0 使用剩余范围"},
        {"laser/roi_h", 0, 0, 100000, "ROI 高，0 使用剩余范围"},
        {"reconstruction/mode", 0, 0, 2, "0 固定平面，1 平移，2 逐位置平面"},
        {"reconstruction/axis", 1, 0, 2, "平移坐标轴，0 X，1 Y，2 Z"},
        {"reconstruction/step_mm", 0.0, -10000, 10000, "有符号平移步距，mm"},
        {"reconstruction/position_start", 0.0, -100000, 100000, "起始激光位置"},
        {"reconstruction/position_step", 1.0, -100000, 100000, "逐帧激光位置增量"},
        {"reconstruction/min_depth", 1.0, 0.001, 1000000, "最小深度，mm"},
        {"reconstruction/max_depth", 10000.0, 0.001, 1000000, "最大深度，mm"},
        {"reconstruction/save_diagnostics", true, 0, 0, "是否保存激光线诊断图"},
        {"acquisition/exposure_us", 3000.0, 1, 10000000, "请求曝光时间，us；USB 后端不设置曝光"},
        {"acquisition/gain", 0.0, 0, 60, "请求增益，dB；USB 后端不设置增益"},
        {"paths/output", QString("output"), 0, 0, "输出根目录，相对路径基于 config 的父目录"},
        {"paths/logDirectory", QString("C:/Project/MLLM/log"), 0, 0, "运行日志目录"},
        {"paths/camera_images", QString(), 0, 0, "相机标定图像目录"},
        {"paths/laser_off", QString(), 0, 0, "激光关闭图像目录"},
        {"paths/laser_on", QString(), 0, 0, "激光开启图像目录"},
        {"paths/reconstruction", QString(), 0, 0, "重建图像目录"},
        {"paths/calibration_file", QString(), 0, 0, "标定 YAML 路径"},
        {"paths/restoreInputPaths", true, 0, 0, "启动时恢复输入路径"}};
    return specs;
}
AppConfigService::AppConfigService(QString directory) : directory_(QDir(directory).absolutePath()) {}
void AppConfigService::load()
{
    values_.clear();
    warnings_.clear();
    error_.clear();
    ConfigFile legacy(directory_ + "/settings.ini"), legacyDefaults(directory_ + "/defaults.ini");
    // 旧文件由 QSettings 写入；只读解码引号、逗号及反斜杠，格式保护仍由 ConfigFile 负责。
    QSettings legacyValues(directory_ + "/settings.ini", QSettings::IniFormat),
        legacyDefaultValues(directory_ + "/defaults.ini", QSettings::IniFormat);
    legacyValues.setIniCodec("UTF-8");
    legacyDefaultValues.setIniCodec("UTF-8");
    QMap<QString, QList<ConfigEntry>> initialize;
    std::map<QString, ConfigFile> categoryFiles, defaultFiles;
    for (const auto& spec : specifications())
    {
        const auto fileName = category(spec.key);
        const auto& file = categoryFiles.try_emplace(fileName, directory_ + '/' + fileName).first->second;
        const auto& defaults =
            defaultFiles.try_emplace(fileName, directory_ + "/defaults/" + fileName).first->second;
        QVariant source = spec.defaultValue;
        if (defaults.error().isEmpty() && defaults.contains(spec.key))
            source = defaults.value(spec.key);
        else if (legacyDefaults.error().isEmpty() && legacyDefaults.contains(spec.key))
            source = legacyValue(legacyDefaults, legacyDefaultValues, spec);
        QVariant validDefault;
        if (!convert(spec, source, validDefault))
            validDefault = spec.defaultValue;
        source = validDefault;
        if (file.exists())
        {
            if (!file.error().isEmpty())
            {
                if (!warnings_.contains(file.error()))
                    warnings_.append(file.error());
            }
            else if (file.contains(spec.key))
                source = file.value(spec.key);
        }
        else if (legacy.error().isEmpty() && legacy.contains(spec.key))
            source = legacyValue(legacy, legacyValues, spec);
        QVariant parsed;
        if (!convert(spec, source, parsed))
        {
            parsed = validDefault;
            warnings_.append("配置值无效，使用默认值：" + spec.key + " = " + source.toString());
        }
        values_[spec.key] = parsed;
        if (!file.exists())
            initialize[fileName].append({spec.key, parsed, spec.comment});
    }
    if (!legacy.error().isEmpty())
        warnings_.append(legacy.error());
    if (values_["reconstruction/min_depth"].toDouble() >= values_["reconstruction/max_depth"].toDouble())
    {
        values_["reconstruction/min_depth"] = 1.0;
        values_["reconstruction/max_depth"] = 10000.0;
        warnings_.append("深度上下限无效，已使用默认值");
    }
    // 旧设置只用于迁移缺失文件；损坏的旧文件保留，避免被默认值覆盖。
    if (legacy.error().isEmpty())
        for (auto group = initialize.begin(); group != initialize.end(); ++group)
        {
            for (auto& entry : group.value())
                entry.value = values_.value(entry.key);
            ConfigFile file(directory_ + '/' + group.key());
            if (!file.save(group.value()))
                warnings_.append(file.error());
        }
    if (!values_["paths/restoreInputPaths"].toBool())
        for (const auto& key : {"paths/camera_images", "paths/laser_off", "paths/laser_on",
                                "paths/reconstruction", "paths/calibration_file"})
            values_[key] = QString();
}
QVariant AppConfigService::value(const QString& key, const QVariant& fallback) const
{
    return values_.value(key, fallback);
}
bool AppConfigService::update(const QVariantMap& values)
{
    error_.clear();
    auto updated = values_;
    for (const auto& spec : specifications())
        if (values.contains(spec.key))
        {
            QVariant parsed;
            if (!convert(spec, values.value(spec.key), parsed))
            {
                error_ = "参数无效：" + spec.key;
                return false;
            }
            updated[spec.key] = parsed;
        }
    // 编辑期间允许临时上下限交叉，真正启动任务时由算法检查；两项不被静默改写。
    values_ = std::move(updated);
    return true;
}
bool AppConfigService::save()
{
    error_.clear();
    ConfigFile legacy(directory_ + "/settings.ini");
    if (!legacy.error().isEmpty())
    {
        error_ = legacy.error();
        return false;
    }
    QMap<QString, QList<ConfigEntry>> groups;
    for (const auto& spec : specifications())
        groups[category(spec.key)].append({spec.key, values_.value(spec.key), spec.comment});
    QStringList failures;
    for (auto group = groups.cbegin(); group != groups.cend(); ++group)
    {
        ConfigFile file(directory_ + '/' + group.key());
        if (!file.save(group.value()))
            failures.append(file.error());
    }
    error_ = failures.join('\n');
    return failures.isEmpty();
}
QStringList AppConfigService::warnings() const
{
    return warnings_;
}
QString AppConfigService::lastError() const
{
    return error_;
}
BoardConfig AppConfigService::board() const
{
    return {{value("calibration/columns").toInt(), value("calibration/rows").toInt()},
            value("calibration/square_mm").toDouble()};
}
LaserConfig AppConfigService::laser() const
{
    LaserConfig result;
    result.method = value("laser/method").toInt();
    result.channel = value("laser/channel").toInt();
    result.horizontal = value("laser/direction").toInt() == 1;
    result.threshold = value("laser/threshold").toInt();
    result.width = value("laser/width").toDouble();
    result.roi = {value("laser/roi_x").toInt(), value("laser/roi_y").toInt(), value("laser/roi_w").toInt(),
                  value("laser/roi_h").toInt()};
    return result;
}
ReconstructionConfig AppConfigService::reconstruction() const
{
    ReconstructionConfig result;
    result.laser = laser();
    result.mode = value("reconstruction/mode").toInt();
    result.axis = value("reconstruction/axis").toInt();
    result.stepMm = value("reconstruction/step_mm").toDouble();
    result.positionStart = value("reconstruction/position_start").toDouble();
    result.positionStep = value("reconstruction/position_step").toDouble();
    result.minDepth = value("reconstruction/min_depth").toDouble();
    result.maxDepth = value("reconstruction/max_depth").toDouble();
    result.saveDiagnostics = value("reconstruction/save_diagnostics").toBool();
    return result;
}
Path AppConfigService::resolvedPath(const QString& key) const
{
    const auto text = value(key).toString().trimmed();
    if (text.isEmpty())
        return {};
    return Path(QDir(QFileInfo(directory_).absolutePath()).absoluteFilePath(text).toStdWString());
}
} // namespace mllm
