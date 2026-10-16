#include "app/services/AppConfigService.h"
#include "app/ui/MainWindow.h"
#include "app/ui/MainWindowHelpers.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QLineEdit>
#include <QSpinBox>
#include <QTimer>
#include <algorithm>

namespace mllm
{
using namespace ui_detail;
QVariant MainWindow::saved(const QString& key, const QVariant& fallback) const
{
    return config_->value(key, fallback);
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

BoardConfig MainWindow::boardConfig()
{
    syncConfig();
    return config_->board();
}

LaserConfig MainWindow::laserConfig()
{
    syncConfig();
    return config_->laser();
}

ReconstructionConfig MainWindow::reconstructionConfig()
{
    syncConfig();
    return config_->reconstruction();
}

bool MainWindow::syncConfig()
{
    QVariantMap values;
    for (auto it = controls_.cbegin(); it != controls_.cend(); ++it)
    {
        if (auto* edit = qobject_cast<QLineEdit*>(it.value()))
            values[it.key()] = edit->text();
        else if (auto* spin = qobject_cast<QSpinBox*>(it.value()))
        {
            spin->interpretText();
            values[it.key()] = spin->value();
        }
        else if (auto* realSpin = qobject_cast<QDoubleSpinBox*>(it.value()))
        {
            realSpin->interpretText();
            values[it.key()] = realSpin->value();
        }
        else if (auto* combo = qobject_cast<QComboBox*>(it.value()))
            values[it.key()] = combo->currentIndex();
        else if (auto* check = qobject_cast<QCheckBox*>(it.value()))
            values[it.key()] = check->isChecked();
    }
    const bool valid = config_->update(values);
    if (!valid)
        log(config_->lastError(), true, "Config");
    return valid;
}

void MainWindow::saveSettings()
{
    if (syncConfig() && !config_->save())
        log(config_->lastError(), true, "Config");
}
} // namespace mllm
