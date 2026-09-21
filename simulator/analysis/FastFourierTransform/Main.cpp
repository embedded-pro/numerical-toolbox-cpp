#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::analysis::view::FftMainWindow>(argc, argv, ui::theme::Light());
}
