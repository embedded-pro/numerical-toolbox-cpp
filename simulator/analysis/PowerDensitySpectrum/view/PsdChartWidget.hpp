#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdSimulator.hpp"
#include <QWidget>
#include <vector>

namespace simulator::analysis::psd::view
{
    class PsdChartWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit PsdChartWidget(QWidget* parent = nullptr);

        void SetData(const PsdResult& result);
        void Clear();

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        void DrawTimeDomain(QPainter& painter, const QRect& plotArea);
        void DrawPowerSpectralDensity(QPainter& painter, const QRect& plotArea);
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawGridLines(QPainter& painter, const QRect& plotArea);

        std::vector<float> time;
        std::vector<float> signal;
        std::vector<float> frequencies;
        std::vector<float> powerDensityDb;

        float maxTime = 0.0f;
        float maxSignalAmplitude = 0.0f;
        float minPowerDb = 0.0f;
        float maxPowerDb = 0.0f;
        float maxFrequency = 0.0f;
    };
}
