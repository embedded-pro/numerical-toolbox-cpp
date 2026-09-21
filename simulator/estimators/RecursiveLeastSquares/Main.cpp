#include "simulator/estimators/RecursiveLeastSquares/view/RlsMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::estimators::rls::view::RlsMainWindow>(argc, argv, ui::theme::Light());
}
