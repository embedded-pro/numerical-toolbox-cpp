#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include <QWidget>
#include <vector>

namespace simulator::analysis::view
{
    class FftChartWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit FftChartWidget(QWidget* parent = nullptr);

        void SetData(const FftResult& result);
        void Clear();

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawSpectrum(QPainter& painter, const QRect& plotArea);
        void DrawLabels(QPainter& painter, const QRect& plotArea);

        std::vector<float> frequencies;
        std::vector<float> magnitudes;
        float maxMagnitude = 0.0f;
        float maxFrequency = 0.0f;
    };
}
