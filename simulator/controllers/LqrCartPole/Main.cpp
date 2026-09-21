#include "simulator/controllers/LqrCartPole/view/LqrMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::controllers::lqr::view::LqrMainWindow>(argc, argv, ui::theme::Light());
}
