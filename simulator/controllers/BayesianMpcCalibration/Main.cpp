#include "simulator/controllers/BayesianMpcCalibration/view/BayesianMpcCalibrationView.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::controllers::view::BayesianMpcCalibrationView>(argc, argv, ui::theme::Light());
}
