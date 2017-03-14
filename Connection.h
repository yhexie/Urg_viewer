#ifndef HRK_CONNECTION_H
#define HRK_CONNECTION_H
#include "Stream.h"


namespace hrk
{
    class Connection : public Stream
    {
    public:
        enum {
            Timeout_infinity = -1, //!< read()
        };


        virtual ~Connection(void)
        {
        }

        virtual const char* what(void) const = 0;


        virtual bool change_baudrate(long baudrate) = 0;

        virtual void ungetc(int ch) = 0;
    };
}

#endif
