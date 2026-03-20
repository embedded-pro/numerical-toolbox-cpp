#pragma once

#include "simulator/controllers/LqrCartPole/application/LqrCartPoleSimulator.hpp"
#include <QMouseEvent>
#include <QTimer>
#include <QWidget>

namespace simulator::controllers::lqr::view
{
    class CartPoleWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit CartPoleWidget(QWidget* parent = nullptr);

        void SetSimulator(LqrCartPoleSimulator* sim);
        void Start();
        void Stop();
        void Reset();

    signals:
        void StateUpdated(float x, float xDot, float theta, float thetaDot, float force);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;

    private:
        void OnTimerTick();
        [[nodiscard]] float PixelsToMeters(float pixels) const;
        [[nodiscard]] float MetersToPixels(float meters) const;
        [[nodiscard]] float TrackCenterY() const;
        [[nodiscard]] float TrackCenterX() const;

        void DrawTrack(QPainter& painter);
        void DrawCart(QPainter& painter, float cartScreenX, float cartScreenY);
        void DrawPole(QPainter& painter, float cartScreenX, float cartScreenY);
        void DrawForceArrow(QPainter& painter, float cartScreenX, float cartScreenY);
        void DrawStateInfo(QPainter& painter);

        LqrCartPoleSimulator* simulator = nullptr;
        QTimer* animationTimer;

        bool isDragging = false;
        float mouseForce = 0.0f;
        int lastMouseX = 0;

        static constexpr float pixelsPerMeter = 150.0f;
        static constexpr float cartWidthPx = 80.0f;
        static constexpr float cartHeightPx = 40.0f;
        static constexpr int timerIntervalMs = 16;
    };
}
