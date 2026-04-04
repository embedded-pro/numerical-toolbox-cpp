#include "simulator/controllers/BayesianMpcCalibration/view/BayesianMpcCalibrationView.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    simulator::controllers::view::BayesianMpcCalibrationView window;
    window.show();

    return app.exec();
}
