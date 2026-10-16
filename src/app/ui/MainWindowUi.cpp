#include "app/services/CaptureSessionService.h"
#include "app/services/TaskRunner.h"
#include "app/ui/LogPanel.h"
#include "app/ui/MainWindow.h"
#include "app/ui/MainWindowHelpers.h"
#include "app/ui/PointCloudPanel.h"
#include "app/ui/Views.h"
#include "core/IO.h"
#include "core/Laser.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace mllm
{
using namespace ui_detail;
void MainWindow::buildUi()
{
    centralTabs_ = new QTabWidget;
    centralTabs_->setObjectName("centralTabs");
    liveView_ = new ImageView;
    centralTabs_->addTab(liveView_, "单目");
    cloudPanel_ = new PointCloudPanel(runner_, this);
    centralTabs_->addTab(cloudPanel_, "点云");
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
    captureCategory_->setObjectName("captureCategory");
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
    capturePairStatus_ = new QLabel("激光标定：先关闭图，再开启图，期间保持棋盘静止。");
    capturePairStatus_->setObjectName("capturePairStatus");
    capturePairStatus_->setWordWrap(true);
    hardwareLayout->addWidget(capturePairStatus_);
    abandonPairButton_ = new QPushButton("放弃未完成组");
    abandonPairButton_->setObjectName("abandonLaserPairButton");
    hardwareLayout->addWidget(abandonPairButton_);
    manualPositionEnabled_ = new QCheckBox("本帧使用人工输入的扫描位置 / 平移量");
    manualPositionEnabled_->setObjectName("captureManualPositionEnabled");
    manualPosition_ = new QDoubleSpinBox;
    manualPosition_->setObjectName("captureManualPosition");
    manualPosition_->setRange(-100000, 100000);
    manualPosition_->setDecimals(6);
    hardwareLayout->addWidget(manualPositionEnabled_);
    hardwareLayout->addWidget(manualPosition_);
    connect(captureCategory_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            &MainWindow::updateCaptureState);
    connect(manualPositionEnabled_, &QCheckBox::toggled, this, &MainWindow::updateCaptureState);
    connect(abandonPairButton_, &QPushButton::clicked, this,
            [this]
            {
                try
                {
                    captureSession_->abandonPair();
                    captureCategory_->setCurrentIndex(1);
                    updateCaptureState();
                    log("未完成图像对已放弃，暂存图保留在会话目录。", false, "Capture");
                }
                catch (const std::exception& error)
                {
                    log(q(error.what()), true, "Capture");
                }
            });

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
                    cv::Mat image = latestFrame_.image;
                    if (captureList_->currentItem())
                        image = readImage(path(captureList_->currentItem()->data(Qt::UserRole).toString()));
                    if (image.empty())
                        throw std::runtime_error("请连接相机或在采集页加载并选择图像");
                    const auto result = extractLaser(image, laserConfig());
                    laserView_->setImage(result.preview);
                    centralTabs_->setCurrentIndex(3);
                    log(QString("中心线 %1 点").arg(result.pixels.size()), false, "LaserExtraction");
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
    messages_ = new LogPanel;
    messageDock->setWidget(messages_);
    addDockWidget(Qt::BottomDockWidgetArea, messageDock);
    resizeDocks({messageDock}, {messages_->sizeHint().height()}, Qt::Vertical);
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
                runner_->cancel();
                cancelButton_->setEnabled(false);
                log("正在请求取消；当前 OpenCV 求解结束后退出。", false, "Workflow");
            });
}
} // namespace mllm
