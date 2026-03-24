#include "simulator/controllers/LqrCartPole/view/LqrMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::controllers::lqr::view::LqrMainWindow window;
    window.show();

    return app.exec();
}
