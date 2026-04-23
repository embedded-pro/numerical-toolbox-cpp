#include "simulator/controllers/Lqg/view/LqgMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::controllers::lqg::view::LqgMainWindow window;
    window.show();

    return app.exec();
}
