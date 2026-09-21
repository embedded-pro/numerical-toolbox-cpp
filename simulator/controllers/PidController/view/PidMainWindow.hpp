#pragma once

#include "simulator/controllers/PidController/application/PidForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include "ui/charts/Log10Axis.hpp"
#include <QMainWindow>
#include <QTimer>

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

        void DisplayTimeResponse(ui::charts::ChartCore& chart, ui::backend::qt::QtPaintedWidget* view, const TimeResponse& result);
        void DisplayBodeResponse(ui::charts::ChartCore& chart, ui::backend::qt::QtPaintedWidget* view, const BodeResult& result);

        PidSimulator pidSimulator;
        pid::PidForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::Log10Axis frequencyAxis;
        ui::charts::ChartCore stepChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore rampChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore bodeChart{ frequencyAxis, ui::charts::ChartConfig{ 1, 2 } };

        ui::backend::qt::QtPaintedWidget* stepView;
        ui::backend::qt::QtPaintedWidget* rampView;
        ui::backend::qt::QtPaintedWidget* bodeView;
        PidRootLocusWidget* rootLocusChart;

        QTimer* dragTimer;
    };
}
