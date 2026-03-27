#include "simulator/filters/FirFilter/view/FirMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::filters::fir::view::FirMainWindow window;
    window.show();

    return app.exec();
}
