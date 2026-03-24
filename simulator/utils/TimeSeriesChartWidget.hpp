#pragma once

#include <QColor>
#include <QString>
#include <QWidget>
#include <span>
#include <vector>

namespace simulator::utils
{
    struct Series
    {
        QString name;
        QColor color;
        std::span<const float> data;
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
        void SetPanels(std::vector<ChartPanel> panels);
        void Clear();

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        struct PanelBounds
        {
            float minY = 0.0f;
            float maxY = 0.0f;
        };

        void DrawPanel(QPainter& painter, const QRect& plotArea, const ChartPanel& panel, const PanelBounds& bounds);
        void DrawSeries(QPainter& painter, const QRect& plotArea, const Series& series, const PanelBounds& bounds);
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawGridLines(QPainter& painter, const QRect& plotArea);
        void DrawYLabels(QPainter& painter, const QRect& plotArea, const PanelBounds& bounds);
        void DrawLegend(QPainter& painter, const QRect& plotArea, const ChartPanel& panel);
        [[nodiscard]] PanelBounds ComputeBounds(const ChartPanel& panel) const;

        std::span<const float> timeData;
        std::vector<ChartPanel> chartPanels;
        float maxTime = 0.0f;

        static constexpr int leftMargin = 65;
        static constexpr int rightMargin = 20;
        static constexpr int topMargin = 15;
        static constexpr int bottomMargin = 35;
        static constexpr int panelSpacing = 45;
        static constexpr int gridLines = 5;
    };
}
