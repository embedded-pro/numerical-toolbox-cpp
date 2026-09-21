#pragma once

#include "simulator/controllers/Lqg/application/LqgForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include <QMainWindow>

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

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::ChartCore chart{ timeAxis, ui::charts::ChartConfig{} };

        ui::backend::qt::QtPaintedWidget* chartView;
    };
}
