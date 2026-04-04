#pragma once

#include <QMainWindow>
#include <QTabWidget>

namespace simulator::estimators::rls::view
{
    class RlsConfigurationPanel;
}

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::estimators::rls::view
{
    class RlsMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit RlsMainWindow(QWidget* parent = nullptr);

    private slots:
        void OnComputeRequested();

    private:
        RlsConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* outputChart;
        widgets::TimeSeriesChartWidget* coefficientChart;
        widgets::TimeSeriesChartWidget* metricsChart;
    };
}
