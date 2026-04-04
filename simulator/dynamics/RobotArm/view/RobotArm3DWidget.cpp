#include "simulator/dynamics/RobotArm/view/RobotArm3DWidget.hpp"
#include <QLinearGradient>
#include <QPainterPath>
#include <algorithm>

namespace simulator::dynamics::view
{
    RobotArm3DWidget::RobotArm3DWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 400);
        setMouseTracking(false);
    }

    void RobotArm3DWidget::SetState(const RobotArmState& state, int dof)
    {
        currentState = state;
        currentDof = dof;
        update();
    }

    QPointF RobotArm3DWidget::Project(float wx, float wy, float wz) const
    {
        float cx = camera.distance * std::cos(camera.elevation) * std::cos(camera.azimuth) + camera.lookAtX;
        float cy = camera.distance * std::cos(camera.elevation) * std::sin(camera.azimuth) + camera.lookAtY;
        float cz = camera.distance * std::sin(camera.elevation) + camera.lookAtZ;

        float fx = camera.lookAtX - cx;
        float fy = camera.lookAtY - cy;
        float fz = camera.lookAtZ - cz;
        float fLen = std::sqrt(fx * fx + fy * fy + fz * fz);
        if (fLen < 1e-6f)
            fLen = 1e-6f;
        fx /= fLen;
        fy /= fLen;
        fz /= fLen;

        float rx = fy * 1.0f - fz * 0.0f;
        float ry = fz * 0.0f - fx * 1.0f;
        float rz = fx * 0.0f - fy * 0.0f;
        float rLen = std::sqrt(rx * rx + ry * ry + rz * rz);
        if (rLen < 1e-6f)
        {
            rx = 1.0f;
            ry = 0.0f;
            rz = 0.0f;
        }
        else
        {
            rx /= rLen;
            ry /= rLen;
            rz /= rLen;
        }

        float ux = ry * fz - rz * fy;
        float uy = rz * fx - rx * fz;
        float uz = rx * fy - ry * fx;

        float dx = wx - cx;
        float dy = wy - cy;
        float dz = wz - cz;
        float vx = rx * dx + ry * dy + rz * dz;
        float vy = ux * dx + uy * dy + uz * dz;
        float vz = fx * dx + fy * dy + fz * dz;

        float fov = 60.0f * 3.14159265f / 180.0f;
        float tanHalfFov = std::tan(fov * 0.5f);
        float aspect = static_cast<float>(width()) / std::max(1, height());
        if (vz < 0.01f)
            vz = 0.01f;

        float px = vx / (vz * tanHalfFov * aspect);
        float py = vy / (vz * tanHalfFov);

        float sx = (px + 1.0f) * 0.5f * width();
        float sy = (1.0f - py) * 0.5f * height();

        return QPointF(sx, sy);
    }

    float RobotArm3DWidget::ProjectDepth(float wx, float wy, float wz) const
    {
        float cx = camera.distance * std::cos(camera.elevation) * std::cos(camera.azimuth) + camera.lookAtX;
        float cy = camera.distance * std::cos(camera.elevation) * std::sin(camera.azimuth) + camera.lookAtY;
        float cz = camera.distance * std::sin(camera.elevation) + camera.lookAtZ;

        float fx = camera.lookAtX - cx;
        float fy = camera.lookAtY - cy;
        float fz = camera.lookAtZ - cz;
        float fLen = std::sqrt(fx * fx + fy * fy + fz * fz);
        if (fLen < 1e-6f)
            fLen = 1e-6f;
        fx /= fLen;
        fy /= fLen;
        fz /= fLen;

        float dx = wx - cx;
        float dy = wy - cy;
        float dz = wz - cz;
        return fx * dx + fy * dy + fz * dz;
    }

    void RobotArm3DWidget::paintEvent(QPaintEvent*)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QLinearGradient bg(0, 0, 0, height());
        bg.setColorAt(0.0, QColor(30, 30, 45));
        bg.setColorAt(1.0, QColor(15, 15, 25));
        painter.fillRect(rect(), bg);

        DrawGrid(painter);
        DrawAxes(painter);
        DrawShadow(painter);
        DrawTrail(painter);
        DrawRobot(painter);
        DrawInfoOverlay(painter);
    }

    void RobotArm3DWidget::DrawGrid(QPainter& painter)
    {
        QPen pen(QColor(60, 60, 80), 1);
        painter.setPen(pen);

        float gridSize = 2.0f;
        float step = 0.25f;

        for (float x = -gridSize; x <= gridSize; x += step)
        {
            auto p1 = Project(x, -gridSize, 0);
            auto p2 = Project(x, gridSize, 0);
            painter.drawLine(p1, p2);
        }

        for (float y = -gridSize; y <= gridSize; y += step)
        {
            auto p1 = Project(-gridSize, y, 0);
            auto p2 = Project(gridSize, y, 0);
            painter.drawLine(p1, p2);
        }
    }

    void RobotArm3DWidget::DrawAxes(QPainter& painter)
    {
        float len = 0.4f;
        auto origin = Project(0, 0, 0);

        painter.setPen(QPen(QColor(220, 60, 60), 2));
        painter.drawLine(origin, Project(len, 0, 0));
        painter.setPen(QPen(QColor(60, 220, 60), 2));
        painter.drawLine(origin, Project(0, len, 0));
        painter.setPen(QPen(QColor(60, 100, 255), 2));
        painter.drawLine(origin, Project(0, 0, len));

        QFont font("Monospace", 9);
        painter.setFont(font);
        painter.setPen(QColor(220, 60, 60));
        painter.drawText(Project(len + 0.05f, 0, 0).toPoint(), "X");
        painter.setPen(QColor(60, 220, 60));
        painter.drawText(Project(0, len + 0.05f, 0).toPoint(), "Y");
        painter.setPen(QColor(60, 100, 255));
        painter.drawText(Project(0, 0, len + 0.05f).toPoint(), "Z");
    }

    void RobotArm3DWidget::DrawShadow(QPainter& painter)
    {
        if (currentState.jointPositions.size() < 2)
            return;

        QPen shadowPen(QColor(80, 80, 80, 80), 3);
        painter.setPen(shadowPen);

        for (std::size_t i = 0; i + 1 < currentState.jointPositions.size(); ++i)
        {
            auto& p1 = currentState.jointPositions[i];
            auto& p2 = currentState.jointPositions[i + 1];
            auto sp1 = Project(p1[0], p1[1], 0);
            auto sp2 = Project(p2[0], p2[1], 0);
            painter.drawLine(sp1, sp2);
        }
    }

    void RobotArm3DWidget::DrawTrail(QPainter& painter)
    {
        if (currentState.endEffectorTrail.size() < 2)
            return;

        auto trailSize = currentState.endEffectorTrail.size();
        for (std::size_t i = 1; i < trailSize; ++i)
        {
            float alpha = static_cast<float>(i) / static_cast<float>(trailSize);
            int a = static_cast<int>(alpha * 180);
            QPen trailPen(QColor(255, 100, 100, a), 1);
            painter.setPen(trailPen);

            auto& p1 = currentState.endEffectorTrail[i - 1];
            auto& p2 = currentState.endEffectorTrail[i];
            painter.drawLine(Project(p1[0], p1[1], p1[2]), Project(p2[0], p2[1], p2[2]));
        }
    }

    void RobotArm3DWidget::DrawRobot(QPainter& painter)
    {
        if (currentState.jointPositions.size() < 2)
            return;

        // Draw links
        for (std::size_t i = 0; i + 1 < currentState.jointPositions.size(); ++i)
        {
            auto& p1 = currentState.jointPositions[i];
            auto& p2 = currentState.jointPositions[i + 1];
            QColor color = QColor::fromRgba(linkColors[i % linkColors.size()]);

            QPen linkPen(color, 6, Qt::SolidLine, Qt::RoundCap);
            painter.setPen(linkPen);
            painter.drawLine(
                Project(p1[0], p1[1], p1[2]),
                Project(p2[0], p2[1], p2[2]));

            // Brighter outline
            QPen outlinePen(color.lighter(140), 8, Qt::SolidLine, Qt::RoundCap);
            outlinePen.setColor(QColor(color.red(), color.green(), color.blue(), 80));
            painter.setPen(outlinePen);
            painter.drawLine(
                Project(p1[0], p1[1], p1[2]),
                Project(p2[0], p2[1], p2[2]));
        }

        // Draw joints
        for (std::size_t i = 0; i < currentState.jointPositions.size(); ++i)
        {
            auto& p = currentState.jointPositions[i];
            auto sp = Project(p[0], p[1], p[2]);

            bool isBase = (i == 0);
            bool isEndEffector = (i == currentState.jointPositions.size() - 1);

            if (isBase)
            {
                painter.setPen(QPen(QColor(200, 200, 200), 2));
                painter.setBrush(QColor(80, 80, 80));
                painter.drawEllipse(sp, 10, 10);

                // Base platform
                auto bl = Project(-0.1f, -0.1f, 0);
                auto br = Project(0.1f, -0.1f, 0);
                auto tr = Project(0.1f, 0.1f, 0);
                auto tl = Project(-0.1f, 0.1f, 0);
                QPainterPath basePath;
                basePath.moveTo(bl);
                basePath.lineTo(br);
                basePath.lineTo(tr);
                basePath.lineTo(tl);
                basePath.closeSubpath();
                painter.setPen(QPen(QColor(100, 100, 120), 1));
                painter.setBrush(QColor(60, 60, 80, 150));
                painter.drawPath(basePath);
            }
            else if (isEndEffector)
            {
                painter.setPen(QPen(QColor(255, 255, 255), 2));
                painter.setBrush(QColor(244, 67, 54));
                painter.drawEllipse(sp, 8, 8);

                // Cross-hair at end-effector
                painter.setPen(QPen(QColor(255, 255, 255, 150), 1));
                painter.drawLine(QPointF(sp.x() - 14, sp.y()), QPointF(sp.x() - 10, sp.y()));
                painter.drawLine(QPointF(sp.x() + 10, sp.y()), QPointF(sp.x() + 14, sp.y()));
                painter.drawLine(QPointF(sp.x(), sp.y() - 14), QPointF(sp.x(), sp.y() - 10));
                painter.drawLine(QPointF(sp.x(), sp.y() + 10), QPointF(sp.x(), sp.y() + 14));
            }
            else
            {
                painter.setPen(QPen(QColor(220, 220, 220), 2));
                painter.setBrush(QColor(50, 50, 60));
                painter.drawEllipse(sp, 7, 7);
            }
        }
    }

    void RobotArm3DWidget::DrawInfoOverlay(QPainter& painter)
    {
        QFont font("Monospace", 10);
        painter.setFont(font);
        painter.setPen(QColor(200, 200, 220));

        int y = 20;
        int x = 10;
        int lineHeight = 18;

        painter.drawText(x, y, QString("t = %1 s").arg(static_cast<double>(currentState.time), 0, 'f', 2));
        y += lineHeight;

        for (int i = 0; i < currentDof && i < static_cast<int>(currentState.q.size()); ++i)
        {
            float deg = currentState.q[i] * 180.0f / 3.14159265f;
            painter.drawText(x, y,
                QString("q%1 = %2 deg  |  dq = %3 rad/s")
                    .arg(i + 1)
                    .arg(static_cast<double>(deg), 7, 'f', 1)
                    .arg(static_cast<double>(currentState.qDot[i]), 7, 'f', 2));
            y += lineHeight;
        }

        if (!currentState.inverseDynamicsTorques.empty())
        {
            y += 4;
            painter.setPen(QColor(180, 180, 120));
            painter.drawText(x, y, "RNEA Inv. Dynamics:");
            y += lineHeight;
            for (int i = 0; i < currentDof && i < static_cast<int>(currentState.inverseDynamicsTorques.size()); ++i)
            {
                painter.drawText(x, y,
                    QString("  τ%1 = %2 N·m")
                        .arg(i + 1)
                        .arg(static_cast<double>(currentState.inverseDynamicsTorques[i]), 8, 'f', 3));
                y += lineHeight;
            }
        }

        painter.setPen(QColor(200, 200, 220));

        if (!currentState.jointPositions.empty())
        {
            auto& ee = currentState.jointPositions.back();
            painter.drawText(x, y,
                QString("EE: (%1, %2, %3)")
                    .arg(static_cast<double>(ee[0]), 0, 'f', 3)
                    .arg(static_cast<double>(ee[1]), 0, 'f', 3)
                    .arg(static_cast<double>(ee[2]), 0, 'f', 3));
        }

        // Camera controls hint
        painter.setPen(QColor(120, 120, 140));
        painter.setFont(QFont("Monospace", 8));
        painter.drawText(10, height() - 10, "LMB: Rotate  |  Scroll: Zoom");
    }

    void RobotArm3DWidget::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
            lastMousePos = event->pos();
    }

    void RobotArm3DWidget::mouseMoveEvent(QMouseEvent* event)
    {
        if (event->buttons() & Qt::LeftButton)
        {
            int dx = event->pos().x() - lastMousePos.x();
            int dy = event->pos().y() - lastMousePos.y();

            camera.azimuth -= dx * 0.008f;
            camera.elevation += dy * 0.008f;

            float maxElev = 1.5f;
            float minElev = -0.2f;
            camera.elevation = std::clamp(camera.elevation, minElev, maxElev);

            lastMousePos = event->pos();
            update();
        }
    }

    void RobotArm3DWidget::wheelEvent(QWheelEvent* event)
    {
        float delta = event->angleDelta().y() / 120.0f;
        camera.distance -= delta * 0.3f;
        camera.distance = std::clamp(camera.distance, 1.0f, 15.0f);
        update();
    }
}
