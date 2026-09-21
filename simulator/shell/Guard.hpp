#pragma once

#include "ui/shell/AppShell.hpp"
#include <exception>
#include <string_view>

namespace simulator::shell
{
    template<class Invocable>
    void Guard(ui::shell::ShellView& shell, std::string_view title, Invocable&& invocable)
    {
        try
        {
            invocable();
        }
        catch (const std::exception& error)
        {
            shell.ShowAlert(title, error.what());
            shell.SetStatus("Computation failed");
        }
    }
}
