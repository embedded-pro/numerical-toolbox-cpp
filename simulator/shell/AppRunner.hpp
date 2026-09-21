#pragma once

#include "ui/backend/qt/QtTheme.hpp"
#include <QApplication>

namespace simulator::shell
{
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
