#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "simulator/analysis/FastFourierTransform/view/FftConfigurationPanel.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include "ui/charts/Log10Axis.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::analysis::view
{
    class FftMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit FftMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        FftSimulator fftSimulator;
        FftConfigurationPanel* configPanel;
        QTabWidget* tabWidget;

        // The charts are the same engine twice, differing only in their axis transform; the
        // widgets are the adapter that puts a ui::PaintedView inside the tab bar.
        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::Log10Axis frequencyAxis;
        ui::charts::ChartCore timeDomainChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore frequencyChart{ frequencyAxis, ui::charts::ChartConfig{ 1, 2 } };

        ui::backend::qt::QtPaintedWidget* timeDomainView;
        ui::backend::qt::QtPaintedWidget* frequencyView;
    };
}
