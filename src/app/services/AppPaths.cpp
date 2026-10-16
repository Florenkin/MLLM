#include "app/services/AppPaths.h"
#include "app/services/ConfigFile.h"
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace mllm
{
QString applicationConfigDirectory()
{
    const auto override = qEnvironmentVariable("MLLM_CONFIG_DIR");
    return QDir(override.isEmpty() ? QCoreApplication::applicationDirPath() + "/config" : override)
        .absolutePath();
}
QString applicationLogDirectory()
{
    const auto config = applicationConfigDirectory();
    QSettings defaults(config + "/defaults.ini", QSettings::IniFormat);
    QSettings settings(config + "/settings.ini", QSettings::IniFormat);
    defaults.setIniCodec("UTF-8");
    settings.setIniCodec("UTF-8");
    ConfigFile defaultFile(config + "/defaults.ini"), settingsFile(config + "/settings.ini");
    const QString key = "paths/logDirectory";
    QString directory = "log";
    if (defaultFile.error().isEmpty() && defaultFile.contains(key))
        directory = legacyIniString(defaultFile.value(key), defaults.value(key).toString());
    if (settingsFile.error().isEmpty() && settingsFile.contains(key))
        directory = legacyIniString(settingsFile.value(key), settings.value(key).toString());
    directory = directory.trimmed();
    ConfigFile classified(config + "/paths.ini");
    if (classified.error().isEmpty() && classified.contains("paths/logDirectory"))
        directory = classified.value("paths/logDirectory").trimmed();
    if (directory.isEmpty())
        directory = "log";
    return QDir(QFileInfo(config).absolutePath()).absoluteFilePath(directory);
}
} // namespace mllm
