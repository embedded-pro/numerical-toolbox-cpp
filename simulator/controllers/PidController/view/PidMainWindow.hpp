#pragma once

#include "simulator/controllers/PidController/application/PidForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>
#include <QTimer>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::controllers::view
{
    class PidRootLocusWidget;

    class PidMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit PidMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();
        void OnPoleGainChanged(float gain);
        void RecomputeAndUpdateCharts();

        void DisplayTimeResponse(widgets::TimeSeriesChartWidget* chart, const TimeResponse& result);
        void DisplayBodeResponse(widgets::FrequencyChartWidget* chart, const BodeResult& result);

        PidSimulator pidSimulator;
        pid::PidForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* stepChart;
        widgets::TimeSeriesChartWidget* rampChart;
        widgets::FrequencyChartWidget* bodeChart;
        PidRootLocusWidget* rootLocusChart;

        QTimer* dragTimer;
    };
}
