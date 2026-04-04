#pragma once

#include "simulator/widgets/ChartInteraction.hpp"
#include <QColor>
#include <QMouseEvent>
#include <QString>
#include <QWheelEvent>
#include <QWidget>
#include <span>
#include <vector>

namespace simulator::widgets
{
    struct Series
    {
        QString name;
        QColor color;
        std::vector<float> data;
    };

    struct ChartPanel
    {
        QString title;
        QString yAxisLabel;
        std::vector<Series> series;
        int heightWeight = 1;
    };

    class TimeSeriesChartWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit TimeSeriesChartWidget(QWidget* parent = nullptr);

        void SetTimeAxis(std::span<const float> time);
        void SetTimeAxis(std::vector<float> time);
        void SetPanels(std::vector<ChartPanel> panels);
        void Clear();

    protected:
        void paintEvent(QPaintEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseDoubleClickEvent(QMouseEvent* event) override;
        void leaveEvent(QEvent* event) override;

    private:
        struct PanelBounds
        {
            float minY = 0.0f;
            float maxY = 0.0f;
        };

        struct PanelLayout
        {
            QRect plotArea;
            PanelBounds bounds;
            std::size_t panelIndex = 0;
        };

        void DrawPanel(QPainter& painter, const QRect& plotArea, const ChartPanel& panel, const PanelBounds& bounds);
        void DrawSeries(QPainter& painter, const QRect& plotArea, const Series& series, const PanelBounds& bounds);
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawGridLines(QPainter& painter, const QRect& plotArea);
        void DrawYLabels(QPainter& painter, const QRect& plotArea, const PanelBounds& bounds);
        void DrawLegend(QPainter& painter, const QRect& plotArea, const ChartPanel& panel);
        void DrawCrosshair(QPainter& painter);
        [[nodiscard]] PanelBounds ComputeBounds(const ChartPanel& panel) const;
        [[nodiscard]] std::vector<PanelLayout> ComputeLayouts() const;
        [[nodiscard]] float TimeToX(float t, int plotLeft, int plotWidth) const;

        std::vector<float> timeData;
        std::vector<ChartPanel> chartPanels;
        float maxTime = 0.0f;
        ChartInteraction interaction;
        std::vector<PanelLayout> cachedLayouts;

        static constexpr int leftMargin = 65;
        static constexpr int rightMargin = 20;
        static constexpr int topMargin = 15;
        static constexpr int bottomMargin = 35;
        static constexpr int panelSpacing = 45;
        static constexpr int gridLines = 5;
    };
}
