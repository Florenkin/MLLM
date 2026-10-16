#pragma once
#include "app/services/PointCloudWorkflow.h"
#include <QWidget>
class QComboBox;
class QCheckBox;
class QLabel;
class QDoubleSpinBox;
namespace mllm
{
class TaskRunner;
class CloudView;
// 主窗口只交付重建结果；面板以执行器接口发起任务，不操作采集或配置。
class PointCloudPanel final : public QWidget
{
    Q_OBJECT
  public:
    explicit PointCloudPanel(TaskRunner* runner, QWidget* parent = nullptr);
    void addCloud(std::string name, std::shared_ptr<const Cloud> points, std::size_t parentId = 0);
    const PointCloudWorkspace& workspace() const;

  private:
    void refresh();
    void execute(CloudOperation operation);
    void crop();
    void exportCloud();
    void apply(const JobResult& result);
    TaskRunner* runner_;
    PointCloudWorkspace workspace_;
    CloudView* view_;
    QComboBox* documents_;
    QCheckBox* selectedOnly_;
    QDoubleSpinBox *voxelSize_, *inlierThreshold_;
    QLabel *status_, *measurement_;
    std::size_t measuredDocumentId_ = 0;
};
} // namespace mllm
