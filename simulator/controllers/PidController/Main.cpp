#include "simulator/controllers/PidController/view/PidMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::controllers::view::PidMainWindow>(argc, argv, ui::theme::Light());
}
