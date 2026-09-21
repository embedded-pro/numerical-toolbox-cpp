#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::analysis::psd::view
{
    class PsdMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit PsdMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        PsdSimulator psdSimulator;
        PsdForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* psdChart;
    };
}
