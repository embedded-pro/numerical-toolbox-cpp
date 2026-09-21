#include "simulator/controllers/Mpc/view/MpcMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::controllers::view::MpcMainWindow>(argc, argv, ui::theme::Light());
}
