#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include <QMouseEvent>
#include <QPainter>
#include <QWidget>
#include <array>
#include <cmath>

namespace simulator::dynamics::view
{
    class RobotArm3DWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit RobotArm3DWidget(QWidget* parent = nullptr);

        void SetState(const RobotArmState& state, int dof);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;

    private:
        struct Camera
        {
            float azimuth = 0.8f;
            float elevation = 0.45f;
            float distance = 3.5f;
            float lookAtX = 0.0f;
            float lookAtY = 0.0f;
            float lookAtZ = 0.3f;
        };

        QPointF Project(float wx, float wy, float wz) const;
        float ProjectDepth(float wx, float wy, float wz) const;
        void DrawGrid(QPainter& painter);
        void DrawAxes(QPainter& painter);
        void DrawRobot(QPainter& painter);
        void DrawShadow(QPainter& painter);
        void DrawTrail(QPainter& painter);
        void DrawInfoOverlay(QPainter& painter);
        void DrawLine3D(QPainter& painter, float x1, float y1, float z1, float x2, float y2, float z2);

        Camera camera;
        RobotArmState currentState;
        int currentDof = 2;
        QPoint lastMousePos;

        static constexpr std::array<QRgb, 3> linkColors = { 0xFF2196F3, 0xFF4CAF50, 0xFFFF9800 };
    };
}
