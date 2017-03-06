#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <altimeter_driver/altimeter.h>
#include <basic_serial.h>

using namespace std;

typedef struct alt alt_t;
struct alt {
    int valid;
    double P;   // [hPa]
    double alt; // [m]
    double T;   // [C]
};

int
_send_init_command (int fd)
{
    unsigned char cmd[] = {'<', 'M', 'C', '\r', '\n'};
    int n_written = write (fd, cmd, sizeof(cmd)-1);

    if (n_written > 0)
        printf ("Write (%d): %s\n", n_written, cmd);
    else
        printf ("Error writing n=%d\n", n_written);

    return n_written;
}

alt_t
parse_alt_data (char *str)
{
    // ex $0,1015.2070,-16.28,40.29
    //    SENSOR_ID, pressure, altimeter, temparature

    char *pch = strtok (str," ,;(){}");

    //int dat1;
    double pressure = 0.0;    // [hPa]
    double alt = 0.0;         // [m]
    double temperature = 0.0; // [C]

    size_t token_counter = 0;

    while (pch) {
        if (token_counter == 1)
            pressure = atof (pch);

        if (token_counter == 2) {
            alt = atof (pch);
        }

        if (token_counter == 3) {
            temperature = atof (pch);
        }

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    //printf ("Parced for %g, alt=%g, temperature=%g\n", pressure, alt, temperature);

    // return alt_t
    alt_t alt_data;

    // validate the data
    if (300.0 < pressure && pressure < 1100.0           // pressure range 300 ~ 1100 [hPa]
        && -500.0 < alt && alt < 9000.0                 // altimeter range -500 ~ 900 [m]
        && -40.0 < temperature && temperature < 85.0) { // temperature range -45 ~ 85 [C]

        //printf ("validation done for %g, alt=%g, temperature=%g\n", pressure, alt, temperature);

        alt_data.P = pressure;
        alt_data.alt = alt;
        alt_data.T = temperature;
        alt_data.valid = 1;

        //printf ("valid\n");
    }
    else {
        //printf ("not valid data P = %g, alt=%g, temperature=%g\n", pressure, alt, temperature);

        alt_data.P = -1.0;
        alt_data.alt = -1.0;
        alt_data.T = -1.0;
        alt_data.valid = 0;

        //printf ("Not valid\n");
    }

    return alt_data;    
}

altimeter_driver::altimeter
convert_to_msg (char *str)
{
    altimeter_driver::altimeter altimeter_data;

    alt_t alt_data = parse_alt_data (str);

    if (alt_data.valid == 1) {
        altimeter_data.data = alt_data.alt;
    }
    return altimeter_data;
}


int 
main (int argc, char *argv[])
{

    ros::init(argc, argv, "altimeter_driver_node");
    ros::NodeHandle nh;
    ros::Publisher altimeter_pub = nh.advertise<altimeter_driver::altimeter>("altimeter_data", 10);

    char ttydev[] = "/dev/ttyUSB-alt";
    int brate = 115200;
    char channel[] = "altimeter_data";

    int fd = serial_open (ttydev, brate, 0);

    serial_set_canonical (fd);

    char buf [256];
    int len;
    memset (&buf, '\0', 256);
    
    struct pollfd poll_events;

    int poll_state;

    long left_count;
    long right_count;
    
    poll_events.fd = fd;
    poll_events.events = POLLIN|POLLERR;
    poll_events.revents = 0;

    if (_send_init_command (fd) < 0) return -1;

    altimeter_driver::altimeter alt_data;

    while(ros::ok()) {
        ros::spinOnce();
        poll_state = poll((struct pollfd*)&poll_events, 1, 10);

        if(0 < poll_state) {
            if (poll_events.revents & POLLIN) {

                ROS_INFO_STREAM("Reading from serial port");
                memset (&buf, '\0', 256);
                
                len = read (fd, buf, sizeof buf);

                alt_data = convert_to_msg(buf);
                alt_data.header.stamp  = ros::Time::now();
                alt_data.header.frame_id = "altimeter";
                altimeter_pub.publish(alt_data);
                
            }

            if (poll_events.revents & POLLERR) {
                printf( "receiver error!" );
                break;
            }
        }
    }

    serial_close(fd);

    return 0;
}


