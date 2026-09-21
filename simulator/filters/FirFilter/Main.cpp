#include "simulator/filters/FirFilter/view/FirMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::filters::fir::view::FirMainWindow>(argc, argv, ui::theme::Light());
}
