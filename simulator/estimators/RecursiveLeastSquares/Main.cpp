#include "simulator/estimators/RecursiveLeastSquares/view/RlsMainWindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::estimators::rls::view::RlsMainWindow window;
    window.show();

    return app.exec();
}
