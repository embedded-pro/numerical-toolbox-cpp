#pragma once

#include "simulator/filters/IirFilter/application/IirForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::filters::iir::view
{
    class IirMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit IirMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        IirForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* frequencyChart;
        widgets::TimeSeriesChartWidget* impulseChart;
    };
}
