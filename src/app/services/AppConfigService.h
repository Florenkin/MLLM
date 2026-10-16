#pragma once
#include "core/Types.h"
#include <QString>
#include <QStringList>
#include <QVariantMap>

namespace mllm
{
struct ParameterSpec
{
    QString key;
    QVariant defaultValue;
    double minimum = 0, maximum = 0;
    QString comment;
};
class AppConfigService final
{
  public:
    explicit AppConfigService(QString directory);
    void load();
    QVariant value(const QString& key, const QVariant& fallback = {}) const;
    bool update(const QVariantMap& values);
    bool save();
    QStringList warnings() const;
    QString lastError() const;
    BoardConfig board() const;
    LaserConfig laser() const;
    ReconstructionConfig reconstruction() const;
    Path resolvedPath(const QString& key) const;
    static const std::vector<ParameterSpec>& specifications();

  private:
    QString directory_, error_;
    QVariantMap values_;
    QStringList warnings_;
};
} // namespace mllm
