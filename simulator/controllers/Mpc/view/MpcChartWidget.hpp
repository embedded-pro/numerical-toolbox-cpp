#pragma once

#include "simulator/controllers/Mpc/application/MpcSimulator.hpp"
#include <QWidget>
#include <vector>

class QPainter;

namespace simulator::controllers::view
{
    class MpcTimeResponseWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit MpcTimeResponseWidget(const QString& title, QWidget* parent = nullptr);

        void SetData(const MpcTimeResponse& result, float referencePosition);

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawStateChart(QPainter& painter, const QRect& plotArea);
        void DrawControlChart(QPainter& painter, const QRect& plotArea);
        void DrawCostChart(QPainter& painter, const QRect& plotArea);

        QString chartTitle;
        std::vector<float> time;
        std::vector<std::vector<float>> states;
        std::vector<float> control;
        std::vector<float> cost;
        float referencePosition = 0.0f;

        float maxTime = 0.0f;
        float maxState = 0.0f;
        float minState = 0.0f;
        float maxControl = 0.0f;
        float minControl = 0.0f;
        float maxCost = 0.0f;
    };

    class MpcStepResponseWidget : public MpcTimeResponseWidget
    {
    public:
        explicit MpcStepResponseWidget(QWidget* parent = nullptr)
            : MpcTimeResponseWidget("Step Response", parent)
        {
        }
    };

    class MpcConstrainedResponseWidget : public MpcTimeResponseWidget
    {
    public:
        explicit MpcConstrainedResponseWidget(QWidget* parent = nullptr)
            : MpcTimeResponseWidget("Constrained Response", parent)
        {
        }
    };
}
