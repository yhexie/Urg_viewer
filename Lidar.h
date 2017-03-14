#ifndef HRK_LIDAR_H
#define HRK_LIDAR_H

#include <vector>
#include <cstddef>


namespace hrk
{
    class Connection;

    class Lidar
    {
    public:
        typedef enum {
            Distance,
            Distance_intensity,
            Multiecho,
			Multiecho_intensity, 
        } measurement_t;

        virtual ~Lidar(void)
        {
        }
        virtual const char* what(void) const = 0;

        virtual void close(void) = 0;

        virtual bool is_open(void) const = 0;

        virtual void set_connection(Connection* connection) = 0;

        virtual Connection* connection(void) = 0;

        virtual bool start_measurement(measurement_t type,
                                       int scan_times, int skip_scan) = 0;
        virtual bool get_distance(std::vector<long>& data,
                                  long *time_stamp) = 0;
        virtual bool get_distance_intensity(std::vector<long>& data,
                                            std::vector<unsigned short>&
                                            intensity,
                                            long *time_stamp) = 0;
        virtual bool get_multiecho(std::vector<long>& data_multi,
                                   long* time_stamp) = 0;
        virtual bool get_multiecho_intensity(std::vector<long>& data_multiecho,
                                             std::vector<unsigned short>&
                                             intensity_multiecho,
                                             long* time_stamp) = 0;
        virtual bool set_scanning_parameter(int first_step, int last_step,
                                            int skip_step) = 0;
        virtual void stop_measurement(void) = 0;

        virtual double index2rad(int index) const = 0;
        virtual double index2deg(int index) const = 0;
        virtual int rad2index(double radian) const = 0;
        virtual int deg2index(double degree) const = 0;

        virtual double step2rad(int step) const = 0;
        virtual double step2deg(int step) const = 0;
        virtual int rad2step(double radian) const = 0;
        virtual int deg2step(double degree) const = 0;

        virtual int min_step(void) const = 0;
        virtual int max_step(void) const = 0;
        virtual int front_step(void) const = 0;
        virtual int total_steps(void) const = 0;
        virtual long min_distance(void) const = 0;
        virtual long max_distance(void) const = 0;
        virtual long scan_usec(void) const = 0;
        virtual int max_data_size(void) const = 0;
        virtual int max_echo_size(void) const = 0;
    };
}
#endif /* !HRK_LIDAR_H */