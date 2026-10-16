#include "app/services/ConfigFile.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSet>

namespace mllm
{
QString legacyIniString(QString raw, const QString& decoded)
{
    if (raw.startsWith('"') && raw.endsWith('"') && raw.size() >= 2)
        raw = raw.mid(1, raw.size() - 2);
    auto unpaired = raw;
    unpaired.replace("\\\\", "");
    // 手写 Windows 路径使用单个分隔符；QSettings 写出的反斜杠成对转义。
    return unpaired.contains('\\') ? raw : decoded;
}
ConfigFile::ConfigFile(QString fileName) : fileName_(std::move(fileName))
{
    QFile file(fileName_);
    if (!file.exists())
        return;
    validSource_ = false;
    if (!file.open(QIODevice::ReadOnly))
    {
        error_ = "无法读取配置：" + fileName_;
        return;
    }
    const auto bytes = file.readAll();
    if (file.error() != QFile::NoError)
    {
        error_ = "配置读取未完成：" + fileName_;
        return;
    }
    auto text = QString::fromUtf8(bytes);
    if (text.toUtf8() != bytes)
    {
        error_ = "配置必须使用 UTF-8 编码：" + fileName_;
        return;
    }
    if (text.startsWith(QChar(0xfeff)))
        text.remove(0, 1);
    text.replace("\r\n", "\n");
    lines_ = text.split('\n');
    QString section;
    for (int i = 0; i < lines_.size(); ++i)
    {
        const auto line = lines_[i].trimmed();
        if (line.isEmpty() || line.startsWith(';') || line.startsWith('#'))
            continue;
        if (line.startsWith('[') && line.endsWith(']') && line.size() > 2)
        {
            section = line.mid(1, line.size() - 2).trimmed();
            continue;
        }
        const auto separator = line.indexOf('=');
        if (separator <= 0 || section.isEmpty())
        {
            error_ = QString("配置 %1 第 %2 行格式无效").arg(fileName_).arg(i + 1);
            return;
        }
        const auto key = section + '/' + line.left(separator).trimmed();
        if (values_.contains(key))
        {
            error_ = "配置参数重复：" + fileName_ + " / " + key;
            return;
        }
        values_[key] = line.mid(separator + 1).trimmed();
        lineIndices_[key] = i;
    }
    validSource_ = true;
}
bool ConfigFile::contains(const QString& key) const
{
    return values_.contains(key);
}
QString ConfigFile::value(const QString& key) const
{
    return values_.value(key);
}
QString ConfigFile::error() const
{
    return error_;
}
bool ConfigFile::exists() const
{
    return QFileInfo::exists(fileName_);
}
bool ConfigFile::save(const QList<ConfigEntry>& entries)
{
    if (!validSource_)
        return false;
    error_.clear();
    auto lines = lines_;
    auto indices = lineIndices_;
    auto values = values_;
    QSet<QString> seen;
    QMap<QString, QList<ConfigEntry>> additions;
    for (const auto& entry : entries)
    {
        if (seen.contains(entry.key) || entry.key.count('/') != 1 ||
            entry.key.section('/', 0, 0).trimmed().isEmpty() ||
            entry.key.section('/', 1).trimmed().isEmpty() || entry.key.contains('\n') ||
            entry.key.contains('\r') || entry.key.contains('=') || entry.comment.contains('\n') ||
            entry.comment.contains('\r'))
        {
            error_ = "配置字段或注释无效：" + entry.key;
            return false;
        }
        seen.insert(entry.key);
        QString text;
        if (entry.value.type() == QVariant::Bool)
            text = entry.value.toBool() ? "true" : "false";
        else if (entry.value.type() == QVariant::Double)
            text = QString::number(entry.value.toDouble(), 'g', 17);
        else
            text = entry.value.toString();
        if (text.contains('\n') || text.contains('\r'))
        {
            error_ = "配置值不能包含换行：" + entry.key;
            return false;
        }
        if (lineIndices_.contains(entry.key))
            lines[lineIndices_[entry.key]] = entry.key.section('/', 1) + '=' + text;
        else
            additions[entry.key.section('/', 0, 0)].append({entry.key, text, entry.comment});
        values[entry.key] = text;
    }
    // 新字段追加到对应节，原来的行、注释和未知参数保持原样。
    for (auto group = additions.cbegin(); group != additions.cend(); ++group)
    {
        lines.append(QString());
        lines.append('[' + group.key() + ']');
        for (const auto& entry : group.value())
        {
            if (!entry.comment.isEmpty())
                lines.append("; " + entry.comment);
            indices[entry.key] = lines.size();
            lines.append(entry.key.section('/', 1) + '=' + entry.value.toString());
        }
    }
    if (!QDir().mkpath(QFileInfo(fileName_).absolutePath()))
    {
        error_ = "无法创建配置目录：" + fileName_;
        return false;
    }
    QSaveFile file(fileName_);
    const auto bytes = lines.join('\n').toUtf8();
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    {
        error_ = "配置保存失败：" + fileName_ + " / " + file.errorString();
        return false;
    }
    lines_ = std::move(lines);
    lineIndices_ = std::move(indices);
    values_ = std::move(values);
    return true;
}
} // namespace mllm
