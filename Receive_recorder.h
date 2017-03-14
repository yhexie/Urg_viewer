#ifndef HRK_RECEIVE_RECORDER_H
#define HRK_RECEIVE_RECORDER_H


#include <memory>
#include "Connection.h"


namespace hrk
{
    class Receive_recorder : public Connection
    {
    public:
        Receive_recorder(void);
        ~Receive_recorder(void);

        /*!
          \brief
          \param[in] connection
          \param[in] file_path
          \retval true
          \retval false
        */
        bool open(Connection* connection, const char* file_path);

        const char* what(void) const;
        bool change_baudrate(long baudrate);
        bool is_open(void) const;
        void close(void);
        int write(const char* data, size_t data_size);
        int read(char* data, size_t max_data_size, int timeout);
        void ungetc(int ch);

    private:
        Receive_recorder(const Receive_recorder& rhs);
        Receive_recorder& operator = (const Receive_recorder& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
