#include "simulator/controllers/LqrCartPole/view/LqrEvaluationWidget.hpp"
#include "ui/theme/Theme.hpp"
#include <QHBoxLayout>
#include <QSplitter>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace simulator::controllers::lqr::view
{
    LqrEvaluationWidget::LqrEvaluationWidget(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);

        auto* metricsLayout = new QHBoxLayout();

        auto makeMetricLabel = [&](const QString& title) -> QLabel*
        {
            auto* container = new QWidget();
            auto* vbox = new QVBoxLayout(container);
            vbox->setContentsMargins(8, 4, 8, 4);

            auto* titleLabel = new QLabel(title);
            titleLabel->setAlignment(Qt::AlignCenter);
            auto titleFont = titleLabel->font();
            titleFont.setPointSize(8);
            titleLabel->setFont(titleFont);

            auto* valueLabel = new QLabel("--");
            valueLabel->setAlignment(Qt::AlignCenter);
            auto valueFont = valueLabel->font();
            valueFont.setPointSize(12);
            valueFont.setBold(true);
            valueLabel->setFont(valueFont);

            vbox->addWidget(titleLabel);
            vbox->addWidget(valueLabel);
            metricsLayout->addWidget(container);
            return valueLabel;
        };

        costLabel = makeMetricLabel("LQR Cost (J)");
        settlingTimeLabel = makeMetricLabel("Settling Time (s)");
        peakThetaLabel = makeMetricLabel("Peak \u03B8 (\u00B0)");
        peakXLabel = makeMetricLabel("Peak x (m)");
        totalEffortLabel = makeMetricLabel("Total Effort (N\u00B7s)");

        layout->addLayout(metricsLayout);

        auto* chartSplitter = new QSplitter(Qt::Vertical, this);

        stateView = new ui::backend::qt::QtPaintedWidget(stateChart, chartSplitter);
        controlView = new ui::backend::qt::QtPaintedWidget(controlChart, chartSplitter);

        stateView->SetPanCursorEnabled(true);
        controlView->SetPanCursorEnabled(true);

        chartSplitter->addWidget(stateView);
        chartSplitter->addWidget(controlView);
        chartSplitter->setStretchFactor(0, 2);
        chartSplitter->setStretchFactor(1, 1);

        layout->addWidget(chartSplitter, 1);
    }

    void LqrEvaluationWidget::SetConfig(const LqrCartPoleConfig& cfg)
    {
        config = cfg;
    }

    void LqrEvaluationWidget::Clear()
    {
        time.clear();
        xHistory.clear();
        xDotHistory.clear();
        thetaHistory.clear();
        thetaDotHistory.clear();
        thetaDegHistory.clear();
        thetaDotDegHistory.clear();
        forceHistory.clear();
        costHistory.clear();

        cumulativeCost = 0.0f;
        peakTheta = 0.0f;
        peakX = 0.0f;
        totalEffort = 0.0f;
        settlingTime = 0.0f;
        sampleCount = 0;

        costLabel->setText("--");
        settlingTimeLabel->setText("--");
        peakThetaLabel->setText("--");
        peakXLabel->setText("--");
        totalEffortLabel->setText("--");

        stateChart.Clear();
        controlChart.Clear();
    }

    void LqrEvaluationWidget::OnStateUpdated(float x, float xDot, float theta, float thetaDot, float force)
    {
        static constexpr auto radToDeg = 180.0f / std::numbers::pi_v<float>;

        auto t = static_cast<float>(sampleCount) * config.simulation.dt;
        ++sampleCount;

        time.push_back(t);
        xHistory.push_back(x);
        xDotHistory.push_back(xDot);
        thetaHistory.push_back(theta);
        thetaDotHistory.push_back(thetaDot);
        thetaDegHistory.push_back(theta * radToDeg);
        thetaDotDegHistory.push_back(thetaDot * radToDeg);
        forceHistory.push_back(force);

        // LQR cost: J = sum( x'Qx + u'Ru ) * dt
        auto& w = config.weights;
        auto stepCost = w.qX * x * x + w.qXDot * xDot * xDot + w.qTheta * theta * theta + w.qThetaDot * thetaDot * thetaDot + w.rForce * force * force;
        cumulativeCost += stepCost * config.simulation.dt;
        costHistory.push_back(cumulativeCost);

        peakTheta = std::max(peakTheta, std::abs(theta));
        peakX = std::max(peakX, std::abs(x));
        totalEffort += std::abs(force) * config.simulation.dt;

        // Settling time: last time any state exceeds threshold
        auto stateNorm = std::sqrt(x * x + theta * theta);
        if (stateNorm > settlingThreshold)
            settlingTime = t;

        if (sampleCount % updateInterval == 0)
        {
            UpdateMetrics();
            UpdateCharts();
        }
    }

    void LqrEvaluationWidget::UpdateMetrics()
    {
        static constexpr auto radToDeg = 180.0f / std::numbers::pi_v<float>;

        costLabel->setText(QString::number(static_cast<double>(cumulativeCost), 'f', 3));
        settlingTimeLabel->setText(QString::number(static_cast<double>(settlingTime), 'f', 2));
        peakThetaLabel->setText(QString::number(static_cast<double>(peakTheta * radToDeg), 'f', 2));
        peakXLabel->setText(QString::number(static_cast<double>(peakX), 'f', 3));
        totalEffortLabel->setText(QString::number(static_cast<double>(totalEffort), 'f', 1));
    }

    void LqrEvaluationWidget::UpdateCharts()
    {
        const auto& theme = ui::theme::Current();

        stateChart.SetAxisValues(time);
        stateChart.SetPanels({
            {
                "Position & Angle",
                "",
                {
                    { "x (m)", theme.Series(0), xHistory },
                    { "\u03B8 (\u00B0)", theme.Series(1), thetaDegHistory },
                },
                2,
            },
            {
                "Velocities",
                "",
                {
                    { "x\u0307 (m/s)", theme.Series(2), xDotHistory },
                    { "\u03B8\u0307 (\u00B0/s)", theme.Series(3), thetaDotDegHistory },
                },
                1,
            },
        });

        controlChart.SetAxisValues(time);
        controlChart.SetPanels({
            {
                "Control Force",
                "",
                {
                    { "Force (N)", theme.Series(4), forceHistory },
                },
                2,
            },
            {
                "Cumulative Cost (J)",
                "",
                {
                    { "J", theme.Series(6), costHistory },
                },
                1,
            },
        });

        stateView->update();
        controlView->update();
    }
}
