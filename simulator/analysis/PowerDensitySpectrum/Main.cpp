#include "simulator/analysis/PowerDensitySpectrum/view/PsdMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::analysis::psd::view::PsdMainWindow>(argc, argv, ui::theme::Light());
}
