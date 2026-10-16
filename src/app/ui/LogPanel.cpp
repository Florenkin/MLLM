#include "app/ui/LogPanel.h"
#include <QColor>
#include <QDateTime>
#include <QHeaderView>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace mllm
{
LogPanel::LogPanel(QWidget* parent) : QWidget(parent)
{
    table_ = new QTableWidget(0, 4, this);
    table_->setObjectName("logTable");
    table_->setHorizontalHeaderLabels({"时间", "级别", "模块", "消息"});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->verticalHeader()->setVisible(false);
    table_->verticalHeader()->setDefaultSectionSize(fontMetrics().height() + 8);
    table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    table_->setColumnWidth(0, 120);
    table_->setColumnWidth(1, 75);
    table_->setColumnWidth(2, 155);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setWordWrap(false);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(table_);
    setMinimumHeight(sizeHint().height());
}
QSize LogPanel::sizeHint() const
{
    return QSize(800, table_->horizontalHeader()->sizeHint().height() +
                          2 * table_->verticalHeader()->defaultSectionSize() + 2 * table_->frameWidth());
}
QSize LogPanel::minimumSizeHint() const
{
    return QSize(0, sizeHint().height());
}
void LogPanel::appendMessage(const LogMessage& message)
{
    if (message.level == LogLevel::Debug)
        return;
    constexpr int maximumRows = 2000;
    if (table_->rowCount() >= maximumRows)
        table_->removeRow(0);
    const auto row = table_->rowCount();
    table_->insertRow(row);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(message.timestamp.time_since_epoch()).count();
    const auto text = QString::fromUtf8(message.text.data(), static_cast<int>(message.text.size()));
    const QString columns[] = {
        QDateTime::fromMSecsSinceEpoch(milliseconds).toString("HH:mm:ss.zzz"),
        QString::fromLatin1(toString(message.level)),
        QString::fromUtf8(message.module.data(), static_cast<int>(message.module.size())), text.simplified()};
    for (int column = 0; column < 4; ++column)
    {
        auto* item = new QTableWidgetItem(columns[column]);
        if (message.level == LogLevel::Error)
            item->setForeground(QColor("#b42318"));
        else if (message.level == LogLevel::Warning)
            item->setForeground(QColor("#9a6700"));
        if (column == 3)
            item->setToolTip(text);
        table_->setItem(row, column, item);
    }
    if (!scrollPending_)
    {
        scrollPending_ = true;
        QTimer::singleShot(0, this,
                           [this]
                           {
                               scrollPending_ = false;
                               table_->scrollToBottom();
                           });
    }
}
void LogPanel::clear()
{
    table_->setRowCount(0);
}
} // namespace mllm
