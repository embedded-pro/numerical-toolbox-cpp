#pragma once

#include "simulator/controllers/Mpc/application/MpcForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::controllers::view
{
    class MpcMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit MpcMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();
        void DisplayResponse(widgets::TimeSeriesChartWidget* chart, const MpcTimeResponse& result, float referencePosition);

        MpcSimulator mpcSimulator;
        mpc::MpcForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* stepChart;
        widgets::TimeSeriesChartWidget* constrainedChart;
    };
}
