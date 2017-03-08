#ifndef HRK_COLOR_H
#define HRK_COLOR_H

/*!
  \file
  \brief 色

  \author Satofumi Kamimura

  $Id$
*/

namespace hrk
{
    class Color
    {
    public:
        /*!
          \param[in] red[0.0, 1.0]
          \param[in] green[0.0, 1.0]
          \param[in] blue[0.0, 1.0]
          \param[in] alpha[0.0, 1.0]
        */
        explicit Color(double red, double green, double blue,
                       double alpha = 1.0);

        explicit Color(void);

        void set_red(double red);

        double red(void) const;

        void set_green(double red);

        double green(void) const;

        void set_blue(double red);

        double blue(void) const;

        void set_alpha(double red);

        double alpha(void) const;

    private:
        double red_;
        double green_;
        double blue_;
        double alpha_;
    };
}

#endif