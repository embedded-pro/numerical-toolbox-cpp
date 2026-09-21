#pragma once

#include "ui/backend/qt/QtTheme.hpp"
#include <QApplication>

namespace simulator::shell
{
    // The twelve-line Main.cpp that was copied byte for byte into ten applications.
    template<class Window>
    int Run(int argc, char* argv[], const ui::theme::Theme& theme)
    {
        QApplication application{ argc, argv };

        ui::backend::qt::ApplyTheme(theme);

        Window window;
        window.show();

        return application.exec();
    }
}
