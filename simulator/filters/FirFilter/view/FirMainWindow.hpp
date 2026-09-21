#pragma once

#include "simulator/filters/FirFilter/application/FirForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include "ui/charts/Log10Axis.hpp"
#include <QMainWindow>

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

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::Log10Axis frequencyAxis;
        ui::charts::LinearAxis sampleAxis{ "Sample", 5, 0, "n = ", "" };
        ui::charts::ChartCore timeDomainChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore frequencyChart{ frequencyAxis, ui::charts::ChartConfig{ 1, 2 } };
        ui::charts::ChartCore impulseChart{ sampleAxis, ui::charts::ChartConfig{} };

        ui::backend::qt::QtPaintedWidget* timeDomainView;
        ui::backend::qt::QtPaintedWidget* frequencyView;
        ui::backend::qt::QtPaintedWidget* impulseView;
    };
}
