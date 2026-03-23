#include "simulator/controllers/Mpc/view/MpcMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::controllers::view::MpcMainWindow window;
    window.show();

    return app.exec();
}
