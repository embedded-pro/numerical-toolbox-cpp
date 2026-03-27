#pragma once

#include "simulator/controllers/PidController/application/PidSimulator.hpp"
#include <QMouseEvent>
#include <QWidget>
#include <complex>
#include <vector>

namespace simulator::controllers::view
{
    class PidRootLocusWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit PidRootLocusWidget(QWidget* parent = nullptr);

        void SetData(const RootLocusResult& result);

    signals:
        void PoleGainChanged(float gain);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;

    private:
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawGridLines(QPainter& painter, const QRect& plotArea);
        void DrawUnitCircle(QPainter& painter, const QRect& plotArea);
        void DrawLoci(QPainter& painter, const QRect& plotArea);
        void DrawPoles(QPainter& painter, const QRect& plotArea);
        void DrawZeros(QPainter& painter, const QRect& plotArea);
        void DrawClosedLoopPoles(QPainter& painter, const QRect& plotArea);
        void DrawStabilityRegion(QPainter& painter, const QRect& plotArea);

        QPointF ComplexToPixel(std::complex<float> c, const QRect& plotArea) const;
        std::complex<float> PixelToComplex(QPointF pixel, const QRect& plotArea) const;
        float FindNearestGain(std::complex<float> point) const;
        QRect GetPlotArea() const;

        std::vector<std::vector<std::complex<float>>> loci;
        std::vector<float> gains;
        std::vector<std::complex<float>> openLoopPoles;
        std::vector<std::complex<float>> openLoopZeros;
        std::vector<std::complex<float>> closedLoopPoles;
        float currentGain = 0.0f;

        float viewRealMin = -10.0f;
        float viewRealMax = 5.0f;
        float viewImagMin = -7.0f;
        float viewImagMax = 7.0f;

        bool isDragging = false;
        int draggedPoleIndex = -1;
    };
}
