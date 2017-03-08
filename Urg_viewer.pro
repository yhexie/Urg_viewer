######################################################################
# Automatically generated by qmake (2.01a) ? 5? 2 08:24:57 2012
######################################################################

#CONFIG += debug
#CONFIG += release
QT += opengl
TEMPLATE = app
TARGET =
VERSION = -2.0.7
DEPENDPATH += .
INCLUDEPATH += .

QMAKE_CXXFLAGS += -DNO_LIBLUABIND
#QMAKE_LIBS += -static-libgcc -static-libstdc++
win32:LIBS += -lsetupapi -lglew32 -lwsock32

# Input
HEADERS += Urg_viewer_window.h \
        Step_value_widget.h \
        Plotter_2d_widget.h \
        Scan_setting_widget.h \
        Player_widget.h \
        Recorder_widget.h \
        Connect_thread.h \
        Receive_thread.h \
        Connection_widget.h \
        Serial_connection_widget.h \
        Ethernet_connection_widget.h \
        Preview_widget.h \

FORMS += Urg_viewer_window_form.ui \
        Step_value_widget_form.ui \
        Scan_setting_widget_form.ui \
        Player_widget_form.ui \
        Recorder_widget_form.ui \
        Serial_connection_widget_form.ui \
        Ethernet_connection_widget_form.ui \

SOURCES += main.cpp Urg_viewer_window.cpp \
        Step_value_widget.cpp \
        Plotter_2d_widget.cpp \
        Scan_setting_widget.cpp \
        Player_widget.cpp \
        Recorder_widget.cpp \
        Connect_thread.cpp \
        Receive_thread.cpp \
        counter_utils.cpp \
        Csv_recorder.cpp \
        Connection_widget.cpp \
        Serial_connection_widget.cpp \
        Ethernet_connection_widget.cpp \
        handle_ethernet_setting.cpp \
        Urg_driver.cpp \
        Urg_log_reader.cpp \
        Serial.cpp \
        Tcpip.cpp \
        connection_utils.cpp \
        Receive_recorder.cpp \
        Color.cpp \
        convert_path_codec.cpp \
        product_utils.cpp \
        Preview_widget.cpp \
        plugin.cpp \

DISTFILES += detect_os.h Lidar.h State.h Color.h Receive_recorder.h Stream.h Connection.h connection_utils.h convert_path_codec.h Scan_setting.h counter_utils.h Csv_recorder.h handle_ethernet_setting.h Urg_driver.h Ring_buffer.hpp Tcpip.h Serial.h Urg_log_reader.h product_utils.h plugin.h \
           Serial_windows.cpp Serial_linux.cpp Tcpip_windows.cpp Tcpip_linux.cpp \
           rescan_icon.png folder_icon.png play_icon.png pause_icon.png stop_icon.png record_icon.png zoom_in_icon.png zoom_out_icon.png Urg_viewer_icon.ico Urg_viewer_icon.png \
           README.txt COPYING.txt Urg_viewer.rc \

RC_FILE = Urg_viewer.rc
RESOURCES += Urg_viewer.qrc
TRANSLATIONS = Urg_viewer_ja.ts

QMAKE_POST_LINK = $$system(lrelease -silent Urg_viewer_ja.ts)
