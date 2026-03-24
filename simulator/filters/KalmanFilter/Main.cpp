#include "simulator/filters/KalmanFilter/view/KalmanMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::filters::view::KalmanMainWindow window;
    window.show();

    return app.exec();
}
