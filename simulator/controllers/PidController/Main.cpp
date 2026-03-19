#include "simulator/controllers/PidController/view/PidMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::controllers::view::PidMainWindow window;
    window.show();

    return app.exec();
}
