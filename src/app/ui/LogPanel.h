#pragma once
#include "core/Logger.h"
#include <QWidget>
class QTableWidget;

namespace mllm
{
class LogPanel final : public QWidget
{
    Q_OBJECT
  public:
    explicit LogPanel(QWidget* parent = nullptr);
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;
  public slots:
    void appendMessage(const mllm::LogMessage& message);
    void clear();

  private:
    QTableWidget* table_ = nullptr;
    bool scrollPending_ = false;
};
} // namespace mllm
