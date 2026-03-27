#pragma once

#include "simulator/utils/SignalGenerator.hpp"
#include <QPushButton>
#include <QTableWidget>
#include <QWidget>
#include <vector>

namespace simulator::widgets
{
    class SignalConfigWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit SignalConfigWidget(QWidget* parent = nullptr);

        void SetDefaultComponents(const std::vector<utils::SignalComponent>& components);
        [[nodiscard]] std::vector<utils::SignalComponent> GetComponents() const;

    signals:
        void ComponentsChanged();

    private:
        void AddComponent();
        void RemoveComponent();

        QTableWidget* componentsTable;
        QPushButton* addButton;
        QPushButton* removeButton;
    };
}
