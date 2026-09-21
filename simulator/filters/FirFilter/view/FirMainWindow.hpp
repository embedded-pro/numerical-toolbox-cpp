#pragma once

#include "simulator/filters/FirFilter/application/FirForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::filters::fir::view
{
    class FirMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit FirMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        FirForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* frequencyChart;
        widgets::TimeSeriesChartWidget* impulseChart;
    };
}
