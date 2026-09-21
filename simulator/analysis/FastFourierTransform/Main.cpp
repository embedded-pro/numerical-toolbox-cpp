#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    ui::backend::qt::ApplyTheme(ui::theme::Light());

    simulator::analysis::view::FftMainWindow window;
    window.show();

    return app.exec();
}
