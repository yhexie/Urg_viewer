#include <QLocale>
#include <QTextCodec>
#include "convert_path_codec.h"

using namespace std;


const std::string hrk::std_string_path(const QString& path)
{
    QTextCodec* codec = QTextCodec::codecForLocale();
    if (codec && codec->canEncode(path)) {
        return QString(codec->fromUnicode(path)).toStdString();
    }
    return path.toStdString();
}


const QString hrk::qt_qstring_path(const std::string& path)
{
    QTextCodec* codec = QTextCodec::codecForLocale();
    if (codec) {
        return codec->toUnicode(path.c_str());
    }
    return path.c_str();
}
