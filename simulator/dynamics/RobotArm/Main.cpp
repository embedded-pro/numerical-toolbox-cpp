#include "simulator/dynamics/RobotArm/view/RobotArmMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::dynamics::view::RobotArmMainWindow window;
    window.show();

    return app.exec();
}
