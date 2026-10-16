#pragma once
#include "core/PointCloud.h"
#include "core/Types.h"
#include <QImage>
#include <QPointF>
#include <QWidget>
namespace mllm
{
class ImageView final : public QWidget
{
  public:
    explicit ImageView(QWidget* parent = nullptr);
    void setImage(const cv::Mat& image);
    void fit();

  protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;

  private:
    QImage image_;
    double scale_ = 1;
    QPointF offset_;
    QPoint last_;
};
class CloudView final : public QWidget
{
    Q_OBJECT
  public:
    explicit CloudView(QWidget* parent = nullptr);
    void setCloud(std::shared_ptr<const Cloud> cloud, std::vector<std::size_t> selection = {});
    void setSelection(std::vector<std::size_t> selection);
    void setSelectionMode(bool enabled);
    CloudProjection projection() const;
    void reset();
  signals:
    void selectionRequested(mllm::CloudProjection projection, cv::Rect2d rectangle);

  protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;

  private:
    std::shared_ptr<const Cloud> cloud_;
    std::vector<std::size_t> displayedIndices_, selection_;
    cv::Point3d center_;
    double extent_ = 1, yaw_ = -0.4, pitch_ = 0.4, zoom_ = 1;
    QPoint last_;
    QPointF pan_;
    bool selectionMode_ = false, selecting_ = false;
    QPoint selectionStart_, selectionEnd_;
};
} // namespace mllm
