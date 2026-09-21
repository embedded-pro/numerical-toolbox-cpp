#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include "ui/charts/Log10Axis.hpp"
#include <QMainWindow>

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

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::Log10Axis frequencyAxis;
        ui::charts::ChartCore timeDomainChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore psdChart{ frequencyAxis, ui::charts::ChartConfig{ 1, 2 } };

        ui::backend::qt::QtPaintedWidget* timeDomainView;
        ui::backend::qt::QtPaintedWidget* psdView;
    };
}
