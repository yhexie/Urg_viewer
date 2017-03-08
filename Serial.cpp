#include "detect_os.h"


#if defined(WINDOWS_OS)
#include "Serial_windows.cpp"
#else
#include "Serial_linux.cpp"
#endif