#include "app/services/ResultExportService.h"
#include "app/services/TaskRunner.h"
#include "app/ui/MainWindow.h"
#include "app/ui/MainWindowHelpers.h"
#include "app/ui/PointCloudPanel.h"
#include "core/IO.h"
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>

namespace mllm
{
using namespace ui_detail;
void MainWindow::startCameraCalibration()
{
    if (closing_ || runner_->isRunning() || !syncConfig())
        return;
    if (calibration_.camera.valid() && !calibration_.planes.empty() &&
        QMessageBox::question(this, "重新标定相机",
                              "重新计算相机内参会清除当前激光面表，之后需重新标定激光面。继续？") !=
            QMessageBox::Yes)
        return;
    const auto input = fsPath("paths/camera_images"), output = fsPath("paths/output");
    const auto board = boardConfig();
    startJob(JobKind::CameraCalibration,
             [=](const TaskContext& task) { return cameraWorkflow(input, output, board, task); });
}

void MainWindow::startLaserCalibration()
{
    if (closing_ || runner_->isRunning() || !syncConfig())
        return;
    const auto off = fsPath("paths/laser_off"), on = fsPath("paths/laser_on"),
               output = fsPath("paths/output");
    const auto cal = calibration_;
    const auto board = boardConfig();
    const auto laser = laserConfig();
    const double position = realValue("calibration/position");
    startJob(JobKind::LaserCalibration, [=](const TaskContext& task)
             { return laserWorkflow(off, on, output, cal, board, laser, position, task); });
}

void MainWindow::startReconstruction(bool single)
{
    if (closing_ || runner_->isRunning() || !syncConfig())
        return;
    QString singleFile;
    if (single)
    {
        singleFile = QFileDialog::getOpenFileName(this, "单帧重建", display(fsPath("paths/reconstruction")),
                                                  "图像 (*.png *.bmp *.jpg *.jpeg *.tif *.tiff)");
        if (singleFile.isEmpty())
            return;
    }
    const auto input = fsPath("paths/reconstruction"), output = fsPath("paths/output");
    const auto cal = calibration_;
    const auto c = reconstructionConfig();
    startJob(JobKind::Reconstruction,
             [=](const TaskContext& task)
             {
                 if (!single)
                     return reconstructionWorkflow(input, output, cal, c, task);
                 return singleFrameWorkflow(path(singleFile), output, cal, c, task);
             });
}

void MainWindow::startJob(JobKind kind, std::function<JobResult(const TaskContext&)> work)
{
    if (closing_)
        return;
    runner_->start(kind, std::move(work));
}

void MainWindow::finishJob(const JobResult& result)
{
    busy_ = false;
    panel_->setEnabled(!closing_);
    centralTabs_->setEnabled(!closing_);
    cancelButton_->setEnabled(false);
    progress_->setValue(result.succeeded() ? 100 : 0);
    updateCaptureState();
    if (!result.succeeded() || closing_)
        return;
    try
    {
        if (result.kind == JobKind::CameraCalibration || result.kind == JobKind::LaserCalibration)
        {
            calibration_ = result.calibration;
            qobject_cast<QLineEdit*>(controls_.value("paths/calibration_file"))
                ->setText(display(result.directory / "calibration.yml"));
            updateCalibration();
        }
        if (result.kind == JobKind::Reconstruction && result.batch)
        {
            const auto points = std::shared_ptr<const Cloud>(result.batch, &result.batch->points);
            cloudPanel_->addCloud("重建结果", points);
            centralTabs_->setCurrentIndex(1);
            if (!result.batch->diagnostics.empty())
                loadImages(display(result.directory / "laser_lines"), true);
        }
    }
    catch (const std::exception& error)
    {
        log(q(error.what()), true, "Workflow");
    }
}

void MainWindow::updateCalibration()
{
    if (calibration_.camera.valid())
        calibrationStatus_->setText(
            QString("相机已标定：%1×%2；RMS %3 px\n激光面 %4 个。重新标定相机后需重新标定激光面。")
                .arg(calibration_.camera.imageSize.width)
                .arg(calibration_.camera.imageSize.height)
                .arg(calibration_.camera.rms, 0, 'f', 4)
                .arg(calibration_.planes.size()));
    else
        calibrationStatus_->setText("相机未标定；三维重建还需要激光面标定。");
    planeTable_->setRowCount(static_cast<int>(calibration_.planes.size()));
    for (int i = 0; i < planeTable_->rowCount(); ++i)
    {
        const auto& p = calibration_.planes[i];
        planeTable_->setItem(i, 0, new QTableWidgetItem(QString::number(p.position, 'g', 10)));
        planeTable_->setItem(i, 1, new QTableWidgetItem(QString::number(p.rmsMm, 'f', 4)));
        planeTable_->setItem(i, 2, new QTableWidgetItem(QString::number(p.samples)));
    }
}

void MainWindow::loadCalibrationFile()
{
    try
    {
        const auto loaded = loadCalibration(fsPath("paths/calibration_file"));
        calibration_ = loaded;
        updateCalibration();
        log(calibration_.planes.empty()
                ? "已加载相机内参；请继续标定激光面。UWScan 水下曲面不会作为空气平面导入。"
                : "已加载完整标定及激光面表。",
            false, "Calibration");
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}

void MainWindow::exportCalibrationFile()
{
    if (!calibration_.camera.valid())
    {
        log("没有可导出的相机标定", true, "Calibration");
        return;
    }
    const auto file =
        QFileDialog::getSaveFileName(this, "导出标定", baseDir_ + "/calibration.yml", "YAML (*.yml)");
    if (file.isEmpty())
        return;
    try
    {
        exportCalibrationAtomically(path(file), calibration_);
        log("标定已导出：" + file, false, "Calibration");
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}

void MainWindow::loadImages(const QString& directory, bool diagnostics)
{
    try
    {
        const auto files = imageFiles(path(directory));
        auto* list = diagnostics ? laserList_ : captureList_;
        list->clear();
        for (const auto& file : files)
        {
            auto* item = new QListWidgetItem(q(file.filename().u8string()), list);
            item->setData(Qt::UserRole, display(file));
        }
        list->setCurrentRow(0);
        if (!diagnostics)
            centralTabs_->setCurrentIndex(2);
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}

void MainWindow::loadDemo(const QString& directory, bool reconstruct)
{
    const auto set = [this, &directory](const QString& key, const QString& relative)
    { qobject_cast<QLineEdit*>(controls_.value(key))->setText(QDir(directory).absoluteFilePath(relative)); };
    set("paths/camera_images", "camera");
    set("paths/laser_off", "off");
    set("paths/laser_on", "on");
    set("paths/reconstruction", "scan");
    set("paths/calibration_file", "calibration.yml");
    qobject_cast<QSpinBox*>(controls_.value("calibration/columns"))->setValue(9);
    qobject_cast<QSpinBox*>(controls_.value("calibration/rows"))->setValue(6);
    qobject_cast<QDoubleSpinBox*>(controls_.value("calibration/square_mm"))->setValue(20);
    qobject_cast<QSpinBox*>(controls_.value("laser/threshold"))->setValue(60);
    loadCalibrationFile();
    loadImages(QDir(directory).absoluteFilePath("scan"));
    log("已加载合成演示数据；演示内参和激光面不用于真实设备。", false, "Demo");
    if (reconstruct)
        startReconstruction(false);
}
} // namespace mllm
