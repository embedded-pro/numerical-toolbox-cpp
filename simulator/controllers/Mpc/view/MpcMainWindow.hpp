#pragma once

#include "simulator/controllers/Mpc/application/MpcForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include <QMainWindow>

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
        void DisplayResponse(ui::charts::ChartCore& chart, ui::backend::qt::QtPaintedWidget* view, const MpcTimeResponse& result, float referencePosition);

        MpcSimulator mpcSimulator;
        mpc::MpcForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::ChartCore stepChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore constrainedChart{ timeAxis, ui::charts::ChartConfig{} };

        ui::backend::qt::QtPaintedWidget* stepView;
        ui::backend::qt::QtPaintedWidget* constrainedView;
    };
}
