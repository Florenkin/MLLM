#include "app/services/ResultExportService.h"
#include "core/IO.h"
#include <QSaveFile>
namespace mllm
{
void exportCalibrationAtomically(const Path& path, const Calibration& calibration)
{
    const auto text = calibrationYaml(calibration);
    QSaveFile output(QString::fromStdWString(path.wstring()));
    if (!output.open(QIODevice::WriteOnly) ||
        output.write(text.data(), static_cast<qint64>(text.size())) != static_cast<qint64>(text.size()) ||
        !output.commit())
        throw std::runtime_error("标定导出失败：" + path.u8string());
}
} // namespace mllm
