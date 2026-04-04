#include "simulator/controllers/LqrCartPole/view/CartPoleWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace simulator::controllers::lqr::view
{
    CartPoleWidget::CartPoleWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(600, 400);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
        setMouseTracking(true);

        animationTimer = new QTimer(this);
        animationTimer->setInterval(timerIntervalMs);
        connect(animationTimer, &QTimer::timeout, this, &CartPoleWidget::OnTimerTick);
    }

    void CartPoleWidget::SetSimulator(LqrCartPoleSimulator* sim)
    {
        simulator = sim;
    }

    void CartPoleWidget::Start()
    {
        animationTimer->start();
    }

    void CartPoleWidget::Stop()
    {
        animationTimer->stop();
    }

    void CartPoleWidget::Reset()
    {
        mouseForce = 0.0f;
        isDragging = false;
        if (simulator)
            simulator->Reset();
        update();
    }

    void CartPoleWidget::OnTimerTick()
    {
        if (!simulator)
            return;

        auto force = simulator->Step(mouseForce);

        auto& s = simulator->GetState();
        emit StateUpdated(s.x, s.xDot, s.theta, s.thetaDot, force);

        update();
    }

    float CartPoleWidget::PixelsToMeters(float pixels) const
    {
        return pixels / pixelsPerMeter;
    }

    float CartPoleWidget::MetersToPixels(float meters) const
    {
        return meters * pixelsPerMeter;
    }

    float CartPoleWidget::TrackCenterY() const
    {
        return static_cast<float>(height()) * 0.65f;
    }

    float CartPoleWidget::TrackCenterX() const
    {
        return static_cast<float>(width()) * 0.5f;
    }

    void CartPoleWidget::paintEvent(QPaintEvent* /*event*/)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        DrawTrack(painter);

        if (!simulator)
            return;

        auto& s = simulator->GetState();
        auto cartScreenX = TrackCenterX() + MetersToPixels(s.x);
        auto cartScreenY = TrackCenterY();

        DrawCart(painter, cartScreenX, cartScreenY);
        DrawPole(painter, cartScreenX, cartScreenY);
        DrawForceArrow(painter, cartScreenX, cartScreenY);
        DrawStateInfo(painter);
    }

    void CartPoleWidget::DrawTrack(QPainter& painter)
    {
        auto y = TrackCenterY();
        auto centerX = TrackCenterX();

        auto trackLimitPx = simulator
                                ? MetersToPixels(simulator->GetPlantParameters().trackLimit)
                                : MetersToPixels(2.4f);

        auto left = centerX - trackLimitPx;
        auto right = centerX + trackLimitPx;

        // Track rail
        QPen trackPen(QColor(80, 80, 80), 3);
        painter.setPen(trackPen);
        painter.drawLine(QPointF(left, y + cartHeightPx / 2.0f),
            QPointF(right, y + cartHeightPx / 2.0f));

        // Track limit markers
        QPen limitPen(QColor(200, 60, 60), 2, Qt::DashLine);
        painter.setPen(limitPen);
        painter.drawLine(QPointF(left, y - 60), QPointF(left, y + cartHeightPx / 2.0f + 10));
        painter.drawLine(QPointF(right, y - 60), QPointF(right, y + cartHeightPx / 2.0f + 10));

        // Center marker
        QPen centerPen(QColor(100, 100, 100), 1, Qt::DotLine);
        painter.setPen(centerPen);
        painter.drawLine(QPointF(centerX, y - 40), QPointF(centerX, y + cartHeightPx / 2.0f + 10));

        // Ground hatch marks
        QPen hatchPen(QColor(120, 120, 120), 1);
        painter.setPen(hatchPen);
        auto groundY = y + cartHeightPx / 2.0f;
        for (float hx = left; hx <= right; hx += 15.0f)
            painter.drawLine(QPointF(hx, groundY), QPointF(hx - 8, groundY + 8));

        // Instruction text
        painter.setPen(QColor(120, 120, 120));
        QFont font = painter.font();
        font.setPointSize(9);
        painter.setFont(font);
        painter.drawText(QRectF(0, static_cast<float>(height()) - 30, static_cast<float>(width()), 25),
            Qt::AlignCenter, "Click and drag to apply force to the cart");
    }

    void CartPoleWidget::DrawCart(QPainter& painter, float cartScreenX, float cartScreenY)
    {
        QRectF cartRect(
            cartScreenX - cartWidthPx / 2.0f,
            cartScreenY - cartHeightPx / 2.0f,
            cartWidthPx,
            cartHeightPx);

        painter.setPen(QPen(QColor(40, 40, 40), 2));
        painter.setBrush(QColor(70, 130, 210));
        painter.drawRoundedRect(cartRect, 5, 5);

        // Wheels
        auto wheelRadius = 8.0f;
        auto wheelY = cartScreenY + cartHeightPx / 2.0f;
        painter.setBrush(QColor(50, 50, 50));
        painter.drawEllipse(QPointF(cartScreenX - cartWidthPx / 4.0f, wheelY),
            wheelRadius, wheelRadius);
        painter.drawEllipse(QPointF(cartScreenX + cartWidthPx / 4.0f, wheelY),
            wheelRadius, wheelRadius);
    }

    void CartPoleWidget::DrawPole(QPainter& painter, float cartScreenX, float cartScreenY)
    {
        if (!simulator)
            return;

        auto& s = simulator->GetState();
        auto poleLength = simulator->GetPlantParameters().poleLength;
        auto poleLengthPx = MetersToPixels(poleLength * 2.0f);

        auto pivotX = cartScreenX;
        auto pivotY = cartScreenY - cartHeightPx / 2.0f;

        // theta=0 is upright (negative Y in screen space); positive theta rotates clockwise
        auto tipX = pivotX + poleLengthPx * std::sin(s.theta);
        auto tipY = pivotY - poleLengthPx * std::cos(s.theta);

        // Pole shadow for depth
        QPen shadowPen(QColor(0, 0, 0, 40), 10, Qt::SolidLine, Qt::RoundCap);
        painter.setPen(shadowPen);
        painter.drawLine(QPointF(pivotX + 2, pivotY + 2), QPointF(tipX + 2, tipY + 2));

        // Pole
        auto absTheta = std::abs(s.theta);
        auto green = static_cast<int>(std::max(0.0f, 200.0f - absTheta * 400.0f));
        auto red = static_cast<int>(std::min(220.0f, 60.0f + absTheta * 400.0f));
        QPen polePen(QColor(red, green, 40), 7, Qt::SolidLine, Qt::RoundCap);
        painter.setPen(polePen);
        painter.drawLine(QPointF(pivotX, pivotY), QPointF(tipX, tipY));

        // Pivot joint
        painter.setPen(QPen(QColor(40, 40, 40), 2));
        painter.setBrush(QColor(200, 200, 200));
        painter.drawEllipse(QPointF(pivotX, pivotY), 6, 6);

        // Pole tip mass
        painter.setBrush(QColor(red, green, 40));
        painter.drawEllipse(QPointF(tipX, tipY), 8, 8);
    }

    void CartPoleWidget::DrawForceArrow(QPainter& painter, float cartScreenX, float cartScreenY)
    {
        if (std::abs(mouseForce) < 0.5f)
            return;

        auto arrowLength = std::clamp(mouseForce * 2.0f, -100.0f, 100.0f);

        QPen arrowPen(QColor(220, 80, 40), 3);
        painter.setPen(arrowPen);
        painter.setBrush(QColor(220, 80, 40));

        auto startX = cartScreenX;
        auto endX = cartScreenX + arrowLength;
        auto arrowY = cartScreenY;

        painter.drawLine(QPointF(startX, arrowY), QPointF(endX, arrowY));

        // Arrow head
        auto headSize = 8.0f;
        auto dir = (arrowLength > 0) ? 1.0f : -1.0f;
        QPolygonF arrowHead;
        arrowHead << QPointF(endX, arrowY)
                  << QPointF(endX - dir * headSize, arrowY - headSize / 2.0f)
                  << QPointF(endX - dir * headSize, arrowY + headSize / 2.0f);
        painter.drawPolygon(arrowHead);

        // Force label
        painter.setPen(QColor(220, 80, 40));
        QFont font = painter.font();
        font.setPointSize(9);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(QPointF(cartScreenX - 30, arrowY + 30),
            QString("F = %1 N").arg(static_cast<double>(mouseForce), 0, 'f', 1));
    }

    void CartPoleWidget::DrawStateInfo(QPainter& painter)
    {
        if (!simulator)
            return;

        static constexpr auto radToDeg = 180.0f / std::numbers::pi_v<float>;

        auto& s = simulator->GetState();
        auto force = simulator->ComputeControlForce();

        painter.setPen(QColor(60, 60, 60));
        auto font = painter.font();
        font.setPointSize(10);
        painter.setFont(font);

        int y = 20;
        int lineHeight = 18;
        int x = 10;

        painter.drawText(x, y, QString("x: %1 m").arg(static_cast<double>(s.x), 7, 'f', 3));
        y += lineHeight;
        painter.drawText(x, y, QString("v: %1 m/s").arg(static_cast<double>(s.xDot), 7, 'f', 3));
        y += lineHeight;
        auto thetaDeg = s.theta * radToDeg;
        painter.drawText(x, y, QString("θ: %1°").arg(static_cast<double>(thetaDeg), 7, 'f', 2));
        y += lineHeight;
        auto thetaDotDeg = s.thetaDot * radToDeg;
        painter.drawText(x, y, QString("ω: %1°/s").arg(static_cast<double>(thetaDotDeg), 7, 'f', 2));
        y += lineHeight;
        painter.drawText(x, y, QString("u: %1 N").arg(static_cast<double>(force), 7, 'f', 2));
    }

    void CartPoleWidget::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            isDragging = true;
            lastMouseX = event->pos().x();
            mouseForce = 0.0f;
        }
    }

    void CartPoleWidget::mouseMoveEvent(QMouseEvent* event)
    {
        if (isDragging)
        {
            int dx = event->pos().x() - lastMouseX;
            mouseForce = static_cast<float>(dx) * 0.5f;
        }
    }

    void CartPoleWidget::mouseReleaseEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            isDragging = false;
            mouseForce = 0.0f;
        }
    }
}
