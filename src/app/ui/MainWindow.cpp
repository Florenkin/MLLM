#include "app/ui/MainWindow.h"
#include "app/ui/Views.h"
#include "core/Calibration.h"
#include "core/IO.h"
#include "core/Laser.h"
#include "core/PointCloud.h"
#include "core/Reconstruction.h"
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace mllm
{
namespace
{
QString q(const std::string& text)
{
    return QString::fromUtf8(text.data(), static_cast<int>(text.size()));
}
Path path(const QString& text)
{
    return Path(text.toStdWString());
}
QString display(const Path& p)
{
    return QString::fromStdWString(p.wstring());
}
QScrollArea* wrapScroll(QWidget* widget)
{
    auto* area = new QScrollArea;
    area->setWidgetResizable(true);
    area->setWidget(widget);
    area->setFrameShape(QFrame::NoFrame);
    return area;
}
QPushButton* button(QVBoxLayout* layout, const QString& text, const QString& name)
{
    auto* b = new QPushButton(text);
    b->setObjectName(name);
    layout->addWidget(b);
    return b;
}
void atomicSave(const QString& file, const std::string& text)
{
    QSaveFile output(file);
    if (!output.open(QIODevice::WriteOnly) ||
        output.write(text.data(), static_cast<qint64>(text.size())) != static_cast<qint64>(text.size()) ||
        !output.commit())
        throw std::runtime_error("保存失败：" + file.toStdString());
}
} // namespace
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    configDir_ = qEnvironmentVariable("MLLM_CONFIG_DIR");
    if (configDir_.isEmpty())
        configDir_ = QCoreApplication::applicationDirPath() + "/config";
    configDir_ = QDir(configDir_).absolutePath();
    baseDir_ = QFileInfo(configDir_).absolutePath();
    QDir().mkpath(configDir_);
    settings_ = std::make_unique<QSettings>(configDir_ + "/settings.ini", QSettings::IniFormat);
    settings_->setIniCodec("UTF-8");
    settings_->setAtomicSyncRequired(true);
    QDir().mkpath(baseDir_ + "/log");
    logPath_ = baseDir_ + "/log/MLLM_" + QDate::currentDate().toString("yyyyMMdd") + ".log";
    qRegisterMetaType<cv::Mat>("cv::Mat");
    setWindowTitle("MLLM — 单目线激光测量");
    resize(1440, 920);
    setStyleSheet(
        "QMainWindow {background:#d9d9d9;} QDockWidget::title {background:#d6d6d6;padding:4px;border:1px "
        "solid #b8b8b8;} QTabWidget::pane {border:1px solid #b8b8b8;background:#f4f4f4;} QTabBar::tab "
        "{background:#e8e8e8;padding:4px 12px;border:1px solid #b8b8b8;} QTabBar::tab:selected "
        "{background:white;} QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox {background:white;border:1px solid "
        "#c0c0c0;padding:2px;} QPushButton {padding:4px 8px;}");
    settingsTimer_ = new QTimer(this);
    settingsTimer_->setSingleShot(true);
    settingsTimer_->setInterval(500);
    connect(settingsTimer_, &QTimer::timeout, this, &MainWindow::saveSettings);
    buildUi();
    connect(&watcher_, &QFutureWatcher<JobResult>::finished, this, &MainWindow::finishJob);
    refreshDevices();
    if (!pathValue("paths/calibration_file").isEmpty())
        try
        {
            calibration_ = loadCalibration(fsPath("paths/calibration_file"));
        }
        catch (const std::exception& e)
        {
            log(q(e.what()), true);
        }
    updateCalibration();
    log("MLLM 已启动。流程：相机标定 → 激光平面标定 → 重建。坐标单位 mm，介质为空气。");
}
MainWindow::~MainWindow()
{
    task_.cancel->store(true);
    if (camera_)
    {
        camera_->stop();
        camera_->wait();
    }
    watcher_.waitForFinished();
}
QVariant MainWindow::saved(const QString& key, const QVariant& fallback) const
{
    if (settings_->contains(key))
        return settings_->value(key);
    QSettings defaults(configDir_ + "/defaults.ini", QSettings::IniFormat);
    defaults.setIniCodec("UTF-8");
    return defaults.value(key, fallback);
}
QLineEdit* MainWindow::pathRow(QFormLayout* form, const QString& title, const QString& key, bool file)
{
    auto* edit = new QLineEdit(saved(key, key == "paths/output" ? "output" : "").toString());
    edit->setObjectName(key);
    controls_[key] = edit;
    auto* row = new QWidget;
    auto* layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    auto* browse = new QPushButton("…");
    browse->setMaximumWidth(32);
    layout->addWidget(edit);
    layout->addWidget(browse);
    form->addRow(title, row);
    connect(browse, &QPushButton::clicked, this,
            [=]
            {
                const QString selected =
                    file
                        ? QFileDialog::getOpenFileName(this, title, pathValue(key), "标定文件 (*.yml *.yaml)")
                        : QFileDialog::getExistingDirectory(this, title, pathValue(key));
                if (!selected.isEmpty())
                    edit->setText(selected);
            });
    connect(edit, &QLineEdit::textChanged, this, [this] { settingsTimer_->start(); });
    return edit;
}
QSpinBox* MainWindow::integer(QFormLayout* form, const QString& title, const QString& key, int value, int lo,
                              int hi)
{
    auto* spin = new QSpinBox;
    spin->setRange(lo, hi);
    spin->setValue(saved(key, value).toInt());
    controls_[key] = spin;
    spin->setObjectName(key);
    form->addRow(title, spin);
    connect(spin, qOverload<int>(&QSpinBox::valueChanged), this, [this] { settingsTimer_->start(); });
    return spin;
}
QDoubleSpinBox* MainWindow::real(QFormLayout* form, const QString& title, const QString& key, double value,
                                 double lo, double hi, int decimals)
{
    auto* spin = new QDoubleSpinBox;
    spin->setRange(lo, hi);
    spin->setDecimals(decimals);
    spin->setValue(saved(key, value).toDouble());
    controls_[key] = spin;
    spin->setObjectName(key);
    form->addRow(title, spin);
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this] { settingsTimer_->start(); });
    return spin;
}
QComboBox* MainWindow::choice(QFormLayout* form, const QString& title, const QString& key,
                              const QStringList& names)
{
    auto* combo = new QComboBox;
    combo->addItems(names);
    combo->setCurrentIndex(std::clamp(saved(key, 0).toInt(), 0, names.size() - 1));
    combo->setObjectName(key);
    controls_[key] = combo;
    form->addRow(title, combo);
    connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this] { settingsTimer_->start(); });
    return combo;
}
void MainWindow::buildUi()
{
    centralTabs_ = new QTabWidget;
    centralTabs_->setObjectName("centralTabs");
    liveView_ = new ImageView;
    centralTabs_->addTab(liveView_, "单目");
    auto* cloudPage = new QWidget;
    auto* cloudLayout = new QVBoxLayout(cloudPage);
    auto* cloudButtons = new QHBoxLayout;
    const auto cloudButton = [&](const QString& name)
    {
        auto* b = new QPushButton(name);
        cloudButtons->addWidget(b);
        return b;
    };
    auto* import = cloudButton("加载点云");
    auto* exportButton = cloudButton("导出点云");
    auto* reset = cloudButton("复位视图");
    auto* measure = cloudButton("拟合平面 / 测量");
    auto* voxel = cloudButton("体素降采样");
    auto* voxelSize = new QDoubleSpinBox;
    voxelSize->setRange(0.001, 1000);
    voxelSize->setValue(0.5);
    voxelSize->setSuffix(" mm");
    cloudButtons->addWidget(voxelSize);
    cloudLayout->addLayout(cloudButtons);
    cloudView_ = new CloudView;
    cloudLayout->addWidget(cloudView_, 1);
    cloudStatus_ = new QLabel("无点云");
    cloudStatus_->setWordWrap(true);
    cloudLayout->addWidget(cloudStatus_);
    centralTabs_->addTab(cloudPage, "点云");
    connect(import, &QPushButton::clicked, this,
            [this]
            {
                if (busy_)
                    return;
                const auto file = QFileDialog::getOpenFileName(this, "加载点云", baseDir_,
                                                               "点云 (*.xyz *.txt *.pcd *.ply)");
                if (file.isEmpty())
                    return;
                startJob(
                    [file](const TaskContext& task)
                    {
                        JobResult r;
                        r.kind = 4;
                        try
                        {
                            task.check();
                            r.batch.points = importCloud(path(file));
                            r.success = true;
                            r.message = "点云已加载";
                        }
                        catch (const std::exception& e)
                        {
                            r.message = e.what();
                        }
                        return r;
                    });
            });
    connect(exportButton, &QPushButton::clicked, this, &MainWindow::exportPointCloud);
    connect(reset, &QPushButton::clicked, cloudView_, &CloudView::reset);
    connect(measure, &QPushButton::clicked, this,
            [this]
            {
                if (cloud_.empty() || busy_)
                    return;
                const auto points = cloud_;
                startJob(
                    [points](const TaskContext& task)
                    {
                        JobResult r;
                        r.kind = 3;
                        try
                        {
                            task.check();
                            const auto fit = fitPlane(points, 0.5);
                            std::ostringstream s;
                            s << std::setprecision(6) << "平面 [a b c d] = " << fit.plane << "；内点 "
                              << fit.inliers << " / " << points.size() << "；RMS " << fit.rms
                              << " mm，最大内点残差 " << fit.maxError << " mm（0.5 mm 阈值）";
                            r.message = s.str();
                            r.success = true;
                        }
                        catch (const std::exception& e)
                        {
                            r.message = e.what();
                        }
                        return r;
                    });
            });
    connect(voxel, &QPushButton::clicked, this,
            [this, voxelSize]
            {
                if (cloud_.empty() || busy_)
                    return;
                const auto points = cloud_;
                const double size = voxelSize->value();
                startJob(
                    [points, size](const TaskContext& task)
                    {
                        JobResult r;
                        r.kind = 4;
                        try
                        {
                            task.check();
                            r.batch.points = voxelDownsample(points, size);
                            r.message = "体素降采样完成（原始导出文件保留）";
                            r.success = true;
                        }
                        catch (const std::exception& e)
                        {
                            r.message = e.what();
                        }
                        return r;
                    });
            });
    const auto reviewPage = [&](const QString& title, QListWidget*& list, ImageView*& view, bool diagnostics)
    {
        auto* page = new QWidget;
        auto* layout = new QVBoxLayout(page);
        auto* open = new QPushButton(diagnostics ? "加载激光线诊断目录" : "加载图像目录");
        layout->addWidget(open);
        auto* split = new QSplitter;
        list = new QListWidget;
        list->setMinimumWidth(150);
        view = new ImageView;
        split->addWidget(list);
        split->addWidget(view);
        split->setStretchFactor(1, 1);
        split->setSizes({220, 800});
        layout->addWidget(split, 1);
        centralTabs_->addTab(page, title);
        connect(open, &QPushButton::clicked, this,
                [this, diagnostics]
                {
                    const auto folder = QFileDialog::getExistingDirectory(this, "图像目录", baseDir_);
                    if (!folder.isEmpty())
                        loadImages(folder, diagnostics);
                });
        connect(list, &QListWidget::currentItemChanged, this,
                [this, view](QListWidgetItem* item, QListWidgetItem*)
                {
                    if (!item)
                        return;
                    try
                    {
                        view->setImage(readImage(path(item->data(Qt::UserRole).toString())));
                    }
                    catch (const std::exception& e)
                    {
                        log(q(e.what()), true);
                    }
                });
    };
    reviewPage("采集", captureList_, captureView_, false);
    reviewPage("激光线", laserList_, laserView_, true);
    setCentralWidget(centralTabs_);
    panel_ = new QWidget;
    panel_->setMinimumWidth(385);
    panel_->setMaximumWidth(620);
    auto* panelLayout = new QVBoxLayout(panel_);
    auto* hardware = new QGroupBox("单相机采集");
    auto* hardwareLayout = new QVBoxLayout(hardware);
    auto* form = new QFormLayout;
    devices_ = new QComboBox;
    devices_->setObjectName("cameraDevices");
    form->addRow("相机", devices_);
    real(form, "曝光 µs", "acquisition/exposure_us", 3000, 1, 10000000, 1);
    real(form, "增益 dB", "acquisition/gain", 0, 0, 60, 1);
    hardwareLayout->addLayout(form);
    auto* hwButtons = new QHBoxLayout;
    auto* refresh = new QPushButton("刷新设备");
    cameraButton_ = new QPushButton("连接 / 预览");
    cameraButton_->setObjectName("cameraPreviewButton");
    hwButtons->addWidget(refresh);
    hwButtons->addWidget(cameraButton_);
    hardwareLayout->addLayout(hwButtons);
    cameraStatus_ = new QLabel("未连接");
    hardwareLayout->addWidget(cameraStatus_);
    captureCategory_ = new QComboBox;
    captureCategory_->addItems({"相机标定图", "激光关闭（棋盘）", "激光开启（棋盘）", "重建图像"});
    hardwareLayout->addWidget(captureCategory_);
    auto* captureButtons = new QHBoxLayout;
    auto* newSession = new QPushButton("新建会话");
    captureButton_ = new QPushButton("采集一帧");
    captureButton_->setObjectName("captureFrameButton");
    captureButton_->setEnabled(false);
    captureButtons->addWidget(newSession);
    captureButtons->addWidget(captureButton_);
    hardwareLayout->addLayout(captureButtons);
    panelLayout->addWidget(hardware);
    connect(refresh, &QPushButton::clicked, this, &MainWindow::refreshDevices);
    connect(cameraButton_, &QPushButton::clicked, this, &MainWindow::toggleCamera);
    connect(captureButton_, &QPushButton::clicked, this, &MainWindow::captureFrame);
    connect(newSession, &QPushButton::clicked, this, &MainWindow::newCaptureSession);
    auto* workflows = new QTabWidget;
    auto* calibrationPage = new QWidget;
    auto* calibrationLayout = new QVBoxLayout(calibrationPage);
    auto* boardGroup = new QGroupBox("棋盘格与相机标定");
    auto* boardForm = new QFormLayout(boardGroup);
    pathRow(boardForm, "相机标定目录", "paths/camera_images");
    integer(boardForm, "内角点列数", "calibration/columns", 11, 3, 30);
    integer(boardForm, "内角点行数", "calibration/rows", 8, 3, 30);
    real(boardForm, "方格边长 mm", "calibration/square_mm", 15, 0.001, 1000);
    calibrationLayout->addWidget(boardGroup);
    auto* calibrate = button(calibrationLayout, "标定相机", "calibrateCameraButton");
    connect(calibrate, &QPushButton::clicked, this, &MainWindow::startCameraCalibration);
    auto* laserGroup = new QGroupBox("激光平面标定");
    auto* laserForm = new QFormLayout(laserGroup);
    pathRow(laserForm, "激光关闭目录", "paths/laser_off");
    pathRow(laserForm, "激光开启目录", "paths/laser_on");
    real(laserForm, "当前标定位置", "calibration/position", 0, -100000, 100000, 6);
    calibrationLayout->addWidget(laserGroup);
    auto* note = new QLabel(
        "每个位置至少 3 "
        "组不同棋盘姿态；开关图像同名配对且棋盘静止。使用重建页的提线参数。振镜扫描需逐位置标定。");
    note->setWordWrap(true);
    calibrationLayout->addWidget(note);
    auto* plane = button(calibrationLayout, "标定当前激光平面", "calibrateLaserButton");
    connect(plane, &QPushButton::clicked, this, &MainWindow::startLaserCalibration);
    auto* fileForm = new QFormLayout;
    pathRow(fileForm, "标定 YAML", "paths/calibration_file", true);
    calibrationLayout->addLayout(fileForm);
    auto* load = button(calibrationLayout, "加载标定文件", "loadCalibrationButton");
    auto* save = button(calibrationLayout, "导出完整标定", "exportCalibrationButton");
    connect(load, &QPushButton::clicked, this, &MainWindow::loadCalibrationFile);
    connect(save, &QPushButton::clicked, this, &MainWindow::exportCalibrationFile);
    calibrationStatus_ = new QLabel;
    calibrationStatus_->setWordWrap(true);
    calibrationLayout->addWidget(calibrationStatus_);
    planeTable_ = new QTableWidget(0, 3);
    planeTable_->setHorizontalHeaderLabels({"位置", "RMS mm", "有效点"});
    planeTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    planeTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    planeTable_->setMinimumHeight(130);
    calibrationLayout->addWidget(planeTable_);
    calibrationLayout->addStretch();
    workflows->addTab(wrapScroll(calibrationPage), "标定");
    auto* reconstructionPage = new QWidget;
    auto* reconstructionLayout = new QVBoxLayout(reconstructionPage);
    auto* pathsForm = new QFormLayout;
    pathRow(pathsForm, "重建图像目录", "paths/reconstruction");
    pathRow(pathsForm, "输出根目录", "paths/output");
    reconstructionLayout->addLayout(pathsForm);
    auto* lineGroup = new QGroupBox("激光中心线");
    auto* lineForm = new QFormLayout(lineGroup);
    choice(lineForm, "算法", "laser/method", {"灰度重心", "Steger"});
    choice(lineForm, "颜色通道", "laser/channel", {"灰度", "红", "绿", "蓝"});
    choice(lineForm, "条纹方向", "laser/direction", {"垂直（逐行）", "水平（逐列）"});
    integer(lineForm, "强度阈值", "laser/threshold", 100, 0, 255);
    real(lineForm, "Steger 线宽 px", "laser/width", 3, 0.5, 30, 2);
    integer(lineForm, "ROI X", "laser/roi_x", 0, 0, 100000);
    integer(lineForm, "ROI Y", "laser/roi_y", 0, 0, 100000);
    integer(lineForm, "ROI 宽（0=剩余）", "laser/roi_w", 0, 0, 100000);
    integer(lineForm, "ROI 高（0=剩余）", "laser/roi_h", 0, 0, 100000);
    reconstructionLayout->addWidget(lineGroup);
    auto* preview = button(reconstructionLayout, "预览当前图像中心线", "previewLaserButton");
    connect(preview, &QPushButton::clicked, this,
            [this]
            {
                try
                {
                    cv::Mat image = latestFrame_;
                    if (captureList_->currentItem())
                        image = readImage(path(captureList_->currentItem()->data(Qt::UserRole).toString()));
                    if (image.empty())
                        throw std::runtime_error("请连接相机或在采集页加载并选择图像");
                    const auto result = extractLaser(image, laserConfig());
                    laserView_->setImage(result.preview);
                    centralTabs_->setCurrentIndex(3);
                    log(QString("中心线 %1 点").arg(result.pixels.size()));
                }
                catch (const std::exception& e)
                {
                    log(q(e.what()), true);
                }
            });
    auto* scanGroup = new QGroupBox("重建与扫描几何");
    auto* scanForm = new QFormLayout(scanGroup);
    choice(scanForm, "扫描模型", "reconstruction/mode",
           {"固定激光面", "已知平移步距", "振镜 / 逐位置激光面"});
    real(scanForm, "有符号平移步距 mm", "reconstruction/step_mm", 0, -10000, 10000);
    choice(scanForm, "平移坐标轴", "reconstruction/axis", {"X", "Y", "Z"});
    real(scanForm, "激光起始位置", "reconstruction/position_start", 0, -100000, 100000, 6);
    real(scanForm, "逐帧位置增量", "reconstruction/position_step", 1, -100000, 100000, 6);
    real(scanForm, "最小深度 mm", "reconstruction/min_depth", 1, 0.001, 1000000);
    real(scanForm, "最大深度 mm", "reconstruction/max_depth", 10000, 0.001, 1000000);
    reconstructionLayout->addWidget(scanGroup);
    auto* diagnostics = new QCheckBox("保存并加载激光线诊断图");
    diagnostics->setChecked(saved("reconstruction/save_diagnostics", true).toBool());
    controls_["reconstruction/save_diagnostics"] = diagnostics;
    reconstructionLayout->addWidget(diagnostics);
    connect(diagnostics, &QCheckBox::toggled, this, [this] { settingsTimer_->start(); });
    auto* single = button(reconstructionLayout, "单帧重建…", "singleReconstructionButton");
    auto* batch = button(reconstructionLayout, "批量重建", "reconstructButton");
    connect(single, &QPushButton::clicked, this, [this] { startReconstruction(true); });
    connect(batch, &QPushButton::clicked, this, [this] { startReconstruction(false); });
    reconstructionLayout->addStretch();
    workflows->addTab(wrapScroll(reconstructionPage), "重建");
    panelLayout->addWidget(workflows, 1);
    auto* right = new QDockWidget("测量工作流", this);
    right->setObjectName("workflowDock");
    right->setWidget(panel_);
    addDockWidget(Qt::RightDockWidgetArea, right);
    auto* messageDock = new QDockWidget("消息", this);
    messageDock->setObjectName("messageDock");
    messages_ = new QPlainTextEdit;
    messages_->setReadOnly(true);
    messages_->setMaximumBlockCount(3000);
    messages_->setMinimumHeight(65);
    messageDock->setWidget(messages_);
    addDockWidget(Qt::BottomDockWidgetArea, messageDock);
    resizeDocks({messageDock}, {95}, Qt::Vertical);
    progress_ = new QProgressBar;
    progress_->setRange(0, 100);
    progress_->setMaximumWidth(180);
    cancelButton_ = new QPushButton("取消任务");
    cancelButton_->setObjectName("cancelTaskButton");
    cancelButton_->setEnabled(false);
    statusBar()->addPermanentWidget(progress_);
    statusBar()->addPermanentWidget(cancelButton_);
    connect(cancelButton_, &QPushButton::clicked, this,
            [this]
            {
                task_.cancel->store(true);
                cancelButton_->setEnabled(false);
                log("正在请求取消；当前 OpenCV 求解结束后退出。");
            });
}
int MainWindow::intValue(const QString& key) const
{
    auto* w = controls_.value(key);
    if (auto* spin = qobject_cast<QSpinBox*>(w))
        return spin->value();
    if (auto* combo = qobject_cast<QComboBox*>(w))
        return combo->currentIndex();
    return 0;
}
double MainWindow::realValue(const QString& key) const
{
    return qobject_cast<QDoubleSpinBox*>(controls_.value(key))->value();
}
QString MainWindow::pathValue(const QString& key) const
{
    return qobject_cast<QLineEdit*>(controls_.value(key))->text().trimmed();
}
Path MainWindow::fsPath(const QString& key) const
{
    const auto value = pathValue(key);
    if (value.isEmpty())
        return {};
    return path(QFileInfo(value).isAbsolute() ? value : QDir(baseDir_).absoluteFilePath(value));
}
BoardConfig MainWindow::boardConfig() const
{
    return {{intValue("calibration/columns"), intValue("calibration/rows")},
            realValue("calibration/square_mm")};
}
LaserConfig MainWindow::laserConfig() const
{
    LaserConfig c;
    c.method = intValue("laser/method");
    c.channel = intValue("laser/channel");
    c.horizontal = intValue("laser/direction") == 1;
    c.threshold = intValue("laser/threshold");
    c.width = realValue("laser/width");
    c.roi = {intValue("laser/roi_x"), intValue("laser/roi_y"), intValue("laser/roi_w"),
             intValue("laser/roi_h")};
    return c;
}
ReconstructionConfig MainWindow::reconstructionConfig() const
{
    ReconstructionConfig c;
    c.laser = laserConfig();
    c.mode = intValue("reconstruction/mode");
    c.axis = intValue("reconstruction/axis");
    c.stepMm = realValue("reconstruction/step_mm");
    c.positionStart = realValue("reconstruction/position_start");
    c.positionStep = realValue("reconstruction/position_step");
    c.minDepth = realValue("reconstruction/min_depth");
    c.maxDepth = realValue("reconstruction/max_depth");
    c.saveDiagnostics =
        qobject_cast<QCheckBox*>(controls_.value("reconstruction/save_diagnostics"))->isChecked();
    return c;
}
void MainWindow::refreshDevices()
{
    try
    {
        const auto selected = devices_->currentData().toString();
        devices_->clear();
        for (const auto& info : cameraDevices())
            devices_->addItem(info.name, info.id);
        const int index = devices_->findData(selected);
        if (index >= 0)
            devices_->setCurrentIndex(index);
        log("设备列表已刷新。");
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}
void MainWindow::toggleCamera()
{
    if (camera_)
    {
        camera_->stop();
        cameraButton_->setEnabled(false);
        cameraStatus_->setText("正在停止预览…");
        captureButton_->setEnabled(false);
        return;
    }
    camera_ = new CameraThread(devices_->currentData().toString(), realValue("acquisition/exposure_us"),
                               realValue("acquisition/gain"), this);
    latestFrame_.release();
    devices_->setEnabled(false);
    cameraButton_->setText("停止预览");
    connect(camera_, &CameraThread::opened, this,
            [this](const QString& name)
            {
                cameraStatus_->setText("已连接：" + name);
                log("预览已连接：" + name);
            });
    connect(camera_, &CameraThread::frameReady, this,
            [this](const cv::Mat& frame)
            {
                if (closing_)
                    return;
                latestFrame_ = frame;
                liveView_->setImage(frame);
                captureButton_->setEnabled(!busy_);
            });
    connect(camera_, &CameraThread::errorOccurred, this, [this](const QString& text) { log(text, true); });
    connect(camera_, &QThread::finished, this,
            [this]
            {
                camera_->deleteLater();
                camera_ = nullptr;
                latestFrame_.release();
                cameraStatus_->setText("未连接");
                cameraButton_->setText("连接 / 预览");
                cameraButton_->setEnabled(true);
                devices_->setEnabled(true);
                captureButton_->setEnabled(false);
            });
    camera_->start();
}
void MainWindow::newCaptureSession()
{
    try
    {
        captureSession_ = uniqueDirectory(fsPath("paths/output"), "capture");
        for (auto& n : captureCounts_)
            n = 0;
        log("新建采集会话：" + display(captureSession_));
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}
void MainWindow::captureFrame()
{
    try
    {
        if (latestFrame_.empty() || !camera_ || !camera_->isRunning())
            throw std::runtime_error("没有活动相机图像");
        if (captureSession_.empty())
            newCaptureSession();
        if (captureSession_.empty())
            return;
        const int category = captureCategory_->currentIndex();
        const QString names[] = {"camera", "off", "on", "reconstruction"};
        const QString keys[] = {"paths/camera_images", "paths/laser_off", "paths/laser_on",
                                "paths/reconstruction"};
        const Path folder = captureSession_ / names[category].toStdString();
        std::ostringstream name;
        name << std::setw(6) << std::setfill('0') << (captureCounts_[category] + 1) << ".png";
        writeImage(folder / name.str(), latestFrame_);
        ++captureCounts_[category];
        qobject_cast<QLineEdit*>(controls_.value(keys[category]))->setText(display(folder));
        loadImages(display(folder));
        log("已采集：" + display(folder / name.str()));
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}
void MainWindow::startCameraCalibration()
{
    if (calibration_.camera.valid() && !calibration_.planes.empty() &&
        QMessageBox::question(this, "重新标定相机",
                              "重新计算相机内参会清除当前激光面表，之后需重新标定激光面。继续？") !=
            QMessageBox::Yes)
        return;
    const auto input = fsPath("paths/camera_images"), output = fsPath("paths/output");
    const auto board = boardConfig();
    startJob([=](const TaskContext& task) { return cameraWorkflow(input, output, board, task); });
}
void MainWindow::startLaserCalibration()
{
    const auto off = fsPath("paths/laser_off"), on = fsPath("paths/laser_on"),
               output = fsPath("paths/output");
    const auto cal = calibration_;
    const auto board = boardConfig();
    const auto laser = laserConfig();
    const double position = realValue("calibration/position");
    startJob([=](const TaskContext& task)
             { return laserWorkflow(off, on, output, cal, board, laser, position, task); });
}
void MainWindow::startReconstruction(bool single)
{
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
    startJob(
        [=](const TaskContext& task)
        {
            if (!single)
                return reconstructionWorkflow(input, output, cal, c, task);
            return singleFrameWorkflow(path(singleFile), output, cal, c, task);
        });
}
void MainWindow::startJob(std::function<JobResult(const TaskContext&)> work)
{
    if (busy_ || closing_)
        return;
    busy_ = true;
    panel_->setEnabled(false);
    centralTabs_->setEnabled(false);
    cancelButton_->setEnabled(true);
    progress_->setValue(0);
    saveSettings();
    task_ = TaskContext{};
    task_.progress = [this](int value, const std::string& text)
    {
        QMetaObject::invokeMethod(
            this,
            [this, value, text]
            {
                progress_->setValue(value);
                statusBar()->showMessage(q(text));
            },
            Qt::QueuedConnection);
    };
    log("开始处理…");
    const auto task = task_;
    watcher_.setFuture(QtConcurrent::run([work, task] { return work(task); }));
}
void MainWindow::finishJob()
{
    const auto r = watcher_.result();
    busy_ = false;
    panel_->setEnabled(!closing_);
    centralTabs_->setEnabled(!closing_);
    cancelButton_->setEnabled(false);
    captureButton_->setEnabled(camera_ && !latestFrame_.empty() && !closing_);
    progress_->setValue(r.success ? 100 : 0);
    log(q(r.message), !r.success && !r.cancelled);
    if (!r.success || closing_)
        return;
    if (r.kind == 0 || r.kind == 1)
    {
        calibration_ = r.calibration;
        qobject_cast<QLineEdit*>(controls_.value("paths/calibration_file"))
            ->setText(display(r.directory / "calibration.yml"));
        updateCalibration();
    }
    if (r.kind == 2 || r.kind == 4)
    {
        cloud_ = r.batch.points;
        updateCloud();
        centralTabs_->setCurrentIndex(1);
    }
    if (r.kind == 2)
    {
        for (const auto& warning : r.batch.warnings)
            log(q(warning), true);
        if (!r.batch.diagnostics.empty())
            loadImages(display(r.directory / "laser_lines"), true);
    }
    if (!r.directory.empty())
        log("结果目录：" + display(r.directory));
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
                : "已加载完整标定及激光面表。");
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
        log("没有可导出的相机标定", true);
        return;
    }
    const auto file =
        QFileDialog::getSaveFileName(this, "导出标定", baseDir_ + "/calibration.yml", "YAML (*.yml)");
    if (file.isEmpty())
        return;
    try
    {
        atomicSave(file, calibrationYaml(calibration_));
        log("标定已导出：" + file);
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}
void MainWindow::exportPointCloud()
{
    if (cloud_.empty() || busy_)
        return;
    const auto file = QFileDialog::getSaveFileName(this, "导出点云", baseDir_ + "/cloud.xyz",
                                                   "XYZ (*.xyz);;PCD (*.pcd);;PLY (*.ply);;TXT (*.txt)");
    if (file.isEmpty())
        return;
    try
    {
        atomicSave(file, cloudText(path(file).extension().u8string(), cloud_));
        log("点云已导出：" + file);
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true);
    }
}
void MainWindow::updateCloud()
{
    cloudView_->setCloud(cloud_);
    if (cloud_.empty())
    {
        cloudStatus_->setText("无点云");
        return;
    }
    const auto [lo, hi] = cloudBounds(cloud_);
    cloudStatus_->setText(QString("%1 点 | X [%2, %3] · Y [%4, %5] · Z [%6, %7] mm | 包围盒 %8 × %9 × %10 "
                                  "mm\n包围盒为相机坐标轴方向尺寸，不能替代标准件精度验收。")
                              .arg(cloud_.size())
                              .arg(lo.x, 0, 'f', 3)
                              .arg(hi.x, 0, 'f', 3)
                              .arg(lo.y, 0, 'f', 3)
                              .arg(hi.y, 0, 'f', 3)
                              .arg(lo.z, 0, 'f', 3)
                              .arg(hi.z, 0, 'f', 3)
                              .arg(hi.x - lo.x, 0, 'f', 3)
                              .arg(hi.y - lo.y, 0, 'f', 3)
                              .arg(hi.z - lo.z, 0, 'f', 3));
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
void MainWindow::log(const QString& text, bool error)
{
    const QString line =
        QDateTime::currentDateTime().toString("HH:mm:ss") + (error ? " [错误] " : " [信息] ") + text;
    messages_->appendPlainText(line);
    statusBar()->showMessage(text);
    QFile file(logPath_);
    if (file.open(QIODevice::WriteOnly | QIODevice::Append))
        file.write((line + "\n").toUtf8());
}
void MainWindow::saveSettings()
{
    for (auto it = controls_.begin(); it != controls_.end(); ++it)
    {
        QVariant value;
        if (auto* edit = qobject_cast<QLineEdit*>(it.value()))
            value = edit->text();
        else if (auto* integer = qobject_cast<QSpinBox*>(it.value()))
        {
            integer->interpretText();
            value = integer->value();
        }
        else if (auto* real = qobject_cast<QDoubleSpinBox*>(it.value()))
        {
            real->interpretText();
            value = real->value();
        }
        else if (auto* combo = qobject_cast<QComboBox*>(it.value()))
            value = combo->currentIndex();
        else if (auto* check = qobject_cast<QCheckBox*>(it.value()))
            value = check->isChecked();
        settings_->setValue(it.key(), value);
    }
    settings_->sync();
    if (settings_->status() != QSettings::NoError)
        log("参数保存失败，请检查配置目录权限", true);
}
void MainWindow::closeEvent(QCloseEvent* event)
{
    closing_ = true;
    task_.cancel->store(true);
    if (camera_)
        camera_->stop();
    if (watcher_.isRunning() || (camera_ && camera_->isRunning()))
    {
        event->ignore();
        statusBar()->showMessage("正在取消任务并释放相机…");
        QTimer::singleShot(100, this, [this] { close(); });
        return;
    }
    saveSettings();
    QMainWindow::closeEvent(event);
}
bool MainWindow::smokeCheck() const
{
    return centralTabs_->count() == 4 && findChild<QPushButton*>("calibrateCameraButton") &&
           findChild<QPushButton*>("calibrateLaserButton") && findChild<QPushButton*>("reconstructButton") &&
           planeTable_->columnCount() == 3;
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
    log("已加载合成演示数据；演示内参和激光面不用于真实设备。");
    if (reconstruct)
        startReconstruction(false);
}
} // namespace mllm
