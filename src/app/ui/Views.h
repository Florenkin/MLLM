#pragma once
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
  public:
    explicit CloudView(QWidget* parent = nullptr);
    void setCloud(const Cloud& cloud);
    void reset();

  protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;

  private:
    Cloud points_;
    cv::Point3d center_;
    double extent_ = 1, yaw_ = -0.4, pitch_ = 0.4, zoom_ = 1;
    QPoint last_;
    QPointF pan_;
};
} // namespace mllm
