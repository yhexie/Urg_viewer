#include <QApplication>
#include <QTranslator>
#include "Urg_viewer_window.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QString locale = QLocale::system().name();
    QTranslator translator;
    translator.load("Urg_viewer_" + locale);
    app.installTranslator(&translator);

    Urg_viewer_window window;
    for (int i = 1; i < argc; ++i) {
        if (window.load_play_file(argv[i])) {
            break;
        }
    }
    window.show();
    return app.exec();
}