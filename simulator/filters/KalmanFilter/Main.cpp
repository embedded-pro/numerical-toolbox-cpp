#include "simulator/filters/KalmanFilter/view/KalmanMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::filters::view::KalmanMainWindow>(argc, argv, ui::theme::Light());
}
