#include "simulator/filters/IirFilter/view/IirMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::filters::iir::view::IirMainWindow>(argc, argv, ui::theme::Light());
}
