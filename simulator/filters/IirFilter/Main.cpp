#include "simulator/filters/IirFilter/view/IirMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::filters::iir::view::IirMainWindow window;
    window.show();

    return app.exec();
}
