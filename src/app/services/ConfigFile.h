#pragma once
#include <QMap>
#include <QStringList>
#include <QVariant>

namespace mllm
{
QString legacyIniString(QString raw, const QString& decoded);
struct ConfigEntry
{
    QString key;
    QVariant value;
    QString comment;
};
// 严格解析 UTF-8 INI，保留注释和未知键；格式损坏时禁止保存。
class ConfigFile final
{
  public:
    explicit ConfigFile(QString fileName);
    bool contains(const QString& key) const;
    QString value(const QString& key) const;
    QString error() const;
    bool save(const QList<ConfigEntry>& entries);
    bool exists() const;

  private:
    QString fileName_, error_;
    QStringList lines_;
    QMap<QString, QString> values_;
    QMap<QString, int> lineIndices_;
    bool validSource_ = true;
};
} // namespace mllm
