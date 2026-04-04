#include "simulator/widgets/SignalConfigWidget.hpp"
#include <QHBoxLayout>
#include <QHeaderView>
#include <QVBoxLayout>

namespace simulator::widgets
{
    SignalConfigWidget::SignalConfigWidget(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);

        componentsTable = new QTableWidget(0, 2, this);
        componentsTable->setHorizontalHeaderLabels({ "Frequency (Hz)", "Amplitude" });
        componentsTable->horizontalHeader()->setStretchLastSection(true);
        componentsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        layout->addWidget(componentsTable);

        auto* buttonLayout = new QHBoxLayout();
        addButton = new QPushButton("Add", this);
        removeButton = new QPushButton("Remove", this);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        layout->addLayout(buttonLayout);

        connect(addButton, &QPushButton::clicked, this, &SignalConfigWidget::AddComponent);
        connect(removeButton, &QPushButton::clicked, this, &SignalConfigWidget::RemoveComponent);
    }

    void SignalConfigWidget::SetDefaultComponents(const std::vector<utils::SignalComponent>& components)
    {
        componentsTable->setRowCount(0);

        for (const auto& comp : components)
        {
            int row = componentsTable->rowCount();
            componentsTable->insertRow(row);
            componentsTable->setItem(row, 0, new QTableWidgetItem(QString::number(static_cast<double>(comp.frequencyHz))));
            componentsTable->setItem(row, 1, new QTableWidgetItem(QString::number(static_cast<double>(comp.amplitude))));
        }
    }

    std::vector<utils::SignalComponent> SignalConfigWidget::GetComponents() const
    {
        std::vector<utils::SignalComponent> components;

        for (int row = 0; row < componentsTable->rowCount(); ++row)
        {
            auto* freqItem = componentsTable->item(row, 0);
            auto* ampItem = componentsTable->item(row, 1);

            if (freqItem && ampItem)
            {
                utils::SignalComponent component;
                component.frequencyHz = freqItem->text().toFloat();
                component.amplitude = ampItem->text().toFloat();
                components.push_back(component);
            }
        }

        return components;
    }

    void SignalConfigWidget::AddComponent()
    {
        int row = componentsTable->rowCount();
        componentsTable->insertRow(row);
        componentsTable->setItem(row, 0, new QTableWidgetItem("1000"));
        componentsTable->setItem(row, 1, new QTableWidgetItem("0.5"));
        emit ComponentsChanged();
    }

    void SignalConfigWidget::RemoveComponent()
    {
        auto selectedRows = componentsTable->selectionModel()->selectedRows();

        if (!selectedRows.isEmpty())
            componentsTable->removeRow(selectedRows.first().row());
        else if (componentsTable->rowCount() > 0)
            componentsTable->removeRow(componentsTable->rowCount() - 1);

        emit ComponentsChanged();
    }
}
