#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::analysis::view::FftMainWindow window;
    window.show();

    return app.exec();
}
