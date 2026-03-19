#include "simulator/analysis/PowerDensitySpectrum/view/PsdMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::analysis::psd::view::PsdMainWindow window;
    window.show();

    return app.exec();
}
