#include "app/ui/PointCloudPanel.h"
#include "app/services/TaskRunner.h"
#include "app/ui/Views.h"
#include "core/Logger.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace mllm
{
namespace
{
QString text(const std::string& value)
{
    return QString::fromUtf8(value.data(), static_cast<int>(value.size()));
}
void report(const std::exception& error)
{
    Logger::instance().error("PointCloud", error.what());
}
} // namespace
PointCloudPanel::PointCloudPanel(TaskRunner* runner, QWidget* parent) : QWidget(parent), runner_(runner)
{
    setObjectName("pointCloudPanel");
    auto* layout = new QVBoxLayout(this);
    auto* row = new QHBoxLayout;
    documents_ = new QComboBox;
    documents_->setObjectName("cloudDocuments");
    documents_->setMinimumWidth(160);
    row->addWidget(documents_, 1);
    const auto button = [&](const QString& label, const char* name, QHBoxLayout* target)
    {
        auto* value = new QPushButton(label);
        value->setObjectName(name);
        target->addWidget(value);
        return value;
    };
    auto* load = button("加载", "cloudLoad", row);
    auto* exportButton = button("导出", "cloudExport", row);
    auto* reset = button("复位视图", "cloudReset", row);
    auto* select = button("框选", "cloudSelect", row);
    select->setCheckable(true);
    auto* clear = button("清除选区", "cloudClearSelection", row);
    auto* extract = button("选区另存为条目", "cloudExtractSelection", row);
    layout->addLayout(row);
    auto* operations = new QHBoxLayout;
    selectedOnly_ = new QCheckBox("仅处理选区");
    selectedOnly_->setObjectName("cloudSelectedOnly");
    operations->addWidget(selectedOnly_);
    auto* fit = button("拟合平面", "cloudFitPlane", operations);
    inlierThreshold_ = new QDoubleSpinBox;
    inlierThreshold_->setRange(0.001, 1000);
    inlierThreshold_->setValue(0.5);
    inlierThreshold_->setSuffix(" mm 内点阈值");
    operations->addWidget(inlierThreshold_);
    auto* voxel = button("体素降采样", "cloudVoxel", operations);
    voxelSize_ = new QDoubleSpinBox;
    voxelSize_->setRange(0.001, 1000);
    voxelSize_->setValue(0.5);
    voxelSize_->setSuffix(" mm");
    operations->addWidget(voxelSize_);
    auto* cropButton = button("按轴裁剪…", "cloudCrop", operations);
    operations->addStretch();
    layout->addLayout(operations);
    view_ = new CloudView;
    view_->setObjectName("cloudView");
    layout->addWidget(view_, 1);
    status_ = new QLabel("无点云");
    status_->setObjectName("cloudStatus");
    status_->setWordWrap(true);
    layout->addWidget(status_);
    measurement_ = new QLabel;
    measurement_->setObjectName("cloudMeasurement");
    measurement_->setWordWrap(true);
    layout->addWidget(measurement_);
    connect(documents_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                if (index < 0 || runner_->isRunning())
                    return;
                try
                {
                    workspace_.activate(static_cast<std::size_t>(documents_->itemData(index).toULongLong()));
                    refresh();
                }
                catch (const std::exception& error)
                {
                    report(error);
                }
            });
    connect(load, &QPushButton::clicked, this,
            [this]
            {
                if (runner_->isRunning())
                    return;
                const auto file =
                    QFileDialog::getOpenFileName(this, "加载点云", {}, "点云 (*.xyz *.txt *.pcd *.ply)");
                if (!file.isEmpty())
                    runner_->start(JobKind::CloudLoad,
                                   [path = Path(file.toStdWString())](const TaskContext& task)
                                   { return loadCloudWorkflow(path, task); });
            });
    connect(exportButton, &QPushButton::clicked, this, &PointCloudPanel::exportCloud);
    connect(reset, &QPushButton::clicked, view_, &CloudView::reset);
    connect(select, &QPushButton::toggled, view_, &CloudView::setSelectionMode);
    connect(clear, &QPushButton::clicked, this,
            [this]
            {
                if (!runner_->isRunning())
                {
                    workspace_.select({});
                    view_->setSelection({});
                    refresh();
                }
            });
    connect(extract, &QPushButton::clicked, this, [this] { execute(CloudOperation::ExtractSelection); });
    connect(fit, &QPushButton::clicked, this, [this] { execute(CloudOperation::PlaneFit); });
    connect(voxel, &QPushButton::clicked, this, [this] { execute(CloudOperation::Voxel); });
    connect(cropButton, &QPushButton::clicked, this, &PointCloudPanel::crop);
    connect(view_, &CloudView::selectionRequested, this,
            [this](const CloudProjection& projection, const cv::Rect2d& rectangle)
            {
                if (runner_->isRunning())
                    return;
                try
                {
                    CloudRequest request;
                    request.input = workspace_.snapshot(false);
                    request.operation = CloudOperation::SelectProjected;
                    request.projection = projection;
                    request.rectangle = rectangle;
                    runner_->start(JobKind::CloudSelect, [request](const TaskContext& task)
                                   { return pointCloudWorkflow(request, task); });
                }
                catch (const std::exception& error)
                {
                    report(error);
                }
            });
    connect(runner_, &TaskRunner::finished, this, &PointCloudPanel::apply);
    connect(runner_, &TaskRunner::started, this, [this] { setEnabled(false); });
    connect(runner_, &TaskRunner::finished, this, [this] { setEnabled(true); });
}
void PointCloudPanel::addCloud(std::string name, std::shared_ptr<const Cloud> points, std::size_t parentId)
{
    workspace_.add(std::move(name), std::move(points), parentId);
    refresh();
}
const PointCloudWorkspace& PointCloudPanel::workspace() const
{
    return workspace_;
}
void PointCloudPanel::refresh()
{
    const auto* current = workspace_.current();
    if (!current)
        return;
    if (current->id != measuredDocumentId_)
        measurement_->clear();
    {
        const QSignalBlocker blocker(documents_);
        documents_->clear();
        for (const auto& item : workspace_.documents())
            documents_->addItem(text(item.name), QVariant::fromValue(static_cast<qulonglong>(item.id)));
        documents_->setCurrentIndex(
            documents_->findData(QVariant::fromValue(static_cast<qulonglong>(current->id))));
    }
    view_->setCloud(current->points, current->selection);
    const auto bounds = cloudBounds(*current->points);
    status_->setText(QString("%1 点；选中 %2 点 | X [%3, %4] · Y [%5, %6] · Z [%7, %8] "
                             "mm\n尺寸 %9 × %10 × %11 mm；处理结果保存为新条目，选区按全部原始点计算。")
                         .arg(current->points->size())
                         .arg(current->selection.size())
                         .arg(bounds.first.x, 0, 'f', 3)
                         .arg(bounds.second.x, 0, 'f', 3)
                         .arg(bounds.first.y, 0, 'f', 3)
                         .arg(bounds.second.y, 0, 'f', 3)
                         .arg(bounds.first.z, 0, 'f', 3)
                         .arg(bounds.second.z, 0, 'f', 3)
                         .arg(bounds.second.x - bounds.first.x, 0, 'f', 3)
                         .arg(bounds.second.y - bounds.first.y, 0, 'f', 3)
                         .arg(bounds.second.z - bounds.first.z, 0, 'f', 3));
}
void PointCloudPanel::execute(CloudOperation operation)
{
    if (runner_->isRunning())
        return;
    try
    {
        CloudRequest request;
        request.input =
            workspace_.snapshot(selectedOnly_->isChecked() || operation == CloudOperation::ExtractSelection);
        request.operation = operation;
        request.voxelMm = voxelSize_->value();
        request.inlierMm = inlierThreshold_->value();
        const auto kind = operation == CloudOperation::PlaneFit ? JobKind::CloudFit : JobKind::CloudDerived;
        runner_->start(kind,
                       [request](const TaskContext& task) { return pointCloudWorkflow(request, task); });
    }
    catch (const std::exception& error)
    {
        report(error);
    }
}
void PointCloudPanel::crop()
{
    if (runner_->isRunning())
        return;
    try
    {
        CloudRequest request;
        request.input = workspace_.snapshot(selectedOnly_->isChecked());
        request.operation = CloudOperation::Crop;
        QDialog dialog(this);
        dialog.setWindowTitle("按轴裁剪点云");
        dialog.setObjectName("cloudCropDialog");
        auto* form = new QFormLayout(&dialog);
        auto* axis = new QComboBox;
        axis->addItems({"X", "Y", "Z"});
        axis->setCurrentIndex(2);
        axis->setObjectName("cloudCropAxis");
        form->addRow("坐标轴", axis);
        const auto bounds = cloudBounds(*request.input.points);
        auto* minimum = new QDoubleSpinBox;
        auto* maximum = new QDoubleSpinBox;
        minimum->setObjectName("cloudCropMinimum");
        maximum->setObjectName("cloudCropMaximum");
        for (auto* input : {minimum, maximum})
        {
            input->setDecimals(6);
            input->setRange(-1e12, 1e12);
        }
        const auto ranges = [bounds, axis, minimum, maximum]
        {
            const auto index = axis->currentIndex();
            minimum->setValue(index == 0 ? bounds.first.x : index == 1 ? bounds.first.y : bounds.first.z);
            maximum->setValue(index == 0 ? bounds.second.x : index == 1 ? bounds.second.y : bounds.second.z);
        };
        ranges();
        connect(axis, qOverload<int>(&QComboBox::currentIndexChanged), &dialog, ranges);
        form->addRow("最小值 mm", minimum);
        form->addRow("最大值 mm", maximum);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        form->addRow(buttons);
        connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        if (dialog.exec() != QDialog::Accepted || runner_->isRunning())
            return;
        request.axis = axis->currentIndex();
        request.minimum = minimum->value();
        request.maximum = maximum->value();
        runner_->start(JobKind::CloudDerived,
                       [request](const TaskContext& task) { return pointCloudWorkflow(request, task); });
    }
    catch (const std::exception& error)
    {
        report(error);
    }
}
void PointCloudPanel::exportCloud()
{
    if (runner_->isRunning())
        return;
    try
    {
        CloudRequest request;
        request.input = workspace_.snapshot(selectedOnly_->isChecked());
        request.operation = CloudOperation::Export;
        QString filter;
        auto file = QFileDialog::getSaveFileName(
            this, "导出点云", "cloud.xyz", "XYZ (*.xyz);;TXT (*.txt);;PCD (*.pcd);;PLY (*.ply)", &filter);
        if (file.isEmpty() || runner_->isRunning())
            return;
        request.exportPath = Path(file.toStdWString());
        if (request.exportPath.extension().empty())
            request.exportPath += filter.contains("pcd")   ? ".pcd"
                                  : filter.contains("ply") ? ".ply"
                                  : filter.contains("txt") ? ".txt"
                                                           : ".xyz";
        runner_->start(JobKind::CloudExport,
                       [request](const TaskContext& task) { return pointCloudWorkflow(request, task); });
    }
    catch (const std::exception& error)
    {
        report(error);
    }
}
void PointCloudPanel::apply(const JobResult& result)
{
    if (!result.succeeded())
        return;
    try
    {
        if (result.kind == JobKind::CloudLoad || result.kind == JobKind::CloudDerived)
            addCloud(result.documentName, result.cloud, result.sourceDocumentId);
        else if (result.kind == JobKind::CloudFit)
        {
            measuredDocumentId_ = result.sourceDocumentId;
            measurement_->setText(text(result.message));
        }
        else if (result.kind == JobKind::CloudSelect && result.selection)
        {
            const auto* current = workspace_.current();
            if (current && current->id == result.sourceDocumentId)
            {
                workspace_.select(*result.selection);
                refresh();
            }
        }
    }
    catch (const std::exception& error)
    {
        report(error);
    }
}
} // namespace mllm
