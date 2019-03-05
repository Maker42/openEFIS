
#include <CmdMessenger.h>

// Attach a new CmdMessenger object to the default Serial port
extern CmdMessenger cmdMessenger;

// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
 ack
,nack
,sensor_reading                // channel, value, timestamp
,send_log                      // loglevel, string
,trace_log                     // line number

// Messages from this program to the board
,setup_digital_sensor          // channel, pin, polling period (ms)
,setup_analog_sensor           // channel, pin, polling period (ms)
,setup_i2c_sensor              // channel, function, polling period (ms)
,setup_spi_sensor              // channel, function, polling period (ms)
,setup_serial_sensor           // channel, function, txpin, rxpin, polling period (ms)
,setup_analog_output           // pin, initial_value
,setup_digital_output          // pin, initial_value
,set_analog_output             // pin, value
,set_digital_output            // pin, value
,save_configuration            // nchans
};

//
// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!


#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"

#define MAX_PWM_VALUE           255

typedef struct
{
    uint32_t    period;
    float       filter_coefficient[2];
    union
    {
        float       secondary_band;
        char        stype[4];
    };
    float       rejection_band;
    unsigned    secondary_filter_duration;
    char        function;
    uint8_t     pin;

    float       state[3];
    uint32_t    next_time;
    uint32_t    sample_count;
    unsigned    secondary_filter_count;
    unsigned    reject_count;
    unsigned    secondary_use_count;
} sChannel;

#define CHANNEL_CONFIGURATION_SIZE (4+4+4+4+2+1+1)

typedef sChannel    *pChannel;

#define MAX_CHANNELS  10

enum
{
    accel_function,
    rotation_function,
    magnetic_function,
    pressure_function,
    temp_function,
    gps_function,
    pitot_function,
    num_functions
};

#define NELEMENTS(x)        (sizeof(x) / sizeof(x[0]))
#define MIN(x,y)            ((x) < (y) ? (x) : (y))
#define ABS(x)              ((x) < 0 ? -(x) : (x))

void cmdLog(unsigned level, const char *s);
