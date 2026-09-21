#pragma once

#include "simulator/controllers/Lqg/application/LqgForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::controllers::lqg::view
{
    class LqgMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit LqgMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();
        void DisplayResponse(const LqgTimeResponse& result);

        LqgSimulator lqgSimulator;
        LqgForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;
        widgets::TimeSeriesChartWidget* chart;
    };
}
