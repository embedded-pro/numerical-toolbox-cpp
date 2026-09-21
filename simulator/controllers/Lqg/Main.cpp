#include "simulator/controllers/Lqg/view/LqgMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::controllers::lqg::view::LqgMainWindow>(argc, argv, ui::theme::Light());
}
