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

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::Log10Axis frequencyAxis;
        ui::charts::ChartCore timeDomainChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore frequencyChart{ frequencyAxis, ui::charts::ChartConfig{ 1, 2 } };

        ui::backend::qt::QtPaintedWidget* timeDomainView;
        ui::backend::qt::QtPaintedWidget* frequencyView;
    };
}
