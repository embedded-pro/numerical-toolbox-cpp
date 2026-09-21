#pragma once

#include "ui/shell/AppShell.hpp"
#include <exception>
#include <string_view>

namespace simulator::shell
{
    // The guard wraps the body of a callback, never its registration. ui-cpp never throws and never
    // catches, but a callback reaching it from here can, and an exception escaping a Qt slot
    // reaches QCoreApplication::notify and terminates the process. Moving the try outward to
    // "simplify" this turns the first rejected configuration into a crash.
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
