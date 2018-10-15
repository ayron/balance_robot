/* Interface node for ADXL345 accelerometer sensor.
 */

#include "ros/ros.h"
#include "balance_msgs/Vector3.h"

extern "C"
{
  #include <unistd.h>         // write, read
  #include <fcntl.h>          // open, O_RDWR
  #include <sys/ioctl.h>      // ioctl
  #include <linux/i2c-dev.h>  // I2C_SLAVE

  #define soc_cv_av
  #include "hwlib.h"
}

#define ADXL345_ADDRESS       0x53

// Bit values in BW_RATE
// Expresed as output data rate
#define XL345_RATE_3200       0x0f
#define XL345_RATE_1600       0x0e
#define XL345_RATE_800        0x0d
#define XL345_RATE_400        0x0c
#define XL345_RATE_200        0x0b
#define XL345_RATE_100        0x0a
#define XL345_RATE_50         0x09
#define XL345_RATE_25         0x08
#define XL345_RATE_12_5       0x07
#define XL345_RATE_6_25       0x06
#define XL345_RATE_3_125      0x05
#define XL345_RATE_1_563      0x04
#define XL345_RATE__782       0x03
#define XL345_RATE__39        0x02
#define XL345_RATE__195       0x01
#define XL345_RATE__098       0x00

// Bit values in DATA_FORMAT

// Register values read in DATAX0 through DATAZ1 are dependant on the
//   value specified in data format.  Customer code will need to interpret
//   the data as desired.
#define XL345_RANGE_2G             0x00
#define XL345_RANGE_4G             0x01
#define XL345_RANGE_8G             0x02
#define XL345_RANGE_16G            0x03
#define XL345_DATA_JUST_RIGHT      0x00
#define XL345_DATA_JUST_LEFT       0x04
#define XL345_10BIT                0x00
#define XL345_FULL_RESOLUTION      0x08
#define XL345_INT_LOW              0x20
#define XL345_INT_HIGH             0x00
#define XL345_SPI3WIRE             0x40
#define XL345_SPI4WIRE             0x00
#define XL345_SELFTEST             0x80

// Bit values in INT_ENABLE, INT_MAP, and INT_SOURCE are identical
// use these bit values to read or write any of these registers.
#define XL345_OVERRUN              0x01
#define XL345_WATERMARK            0x02
#define XL345_FREEFALL             0x04
#define XL345_INACTIVITY           0x08
#define XL345_ACTIVITY             0x10
#define XL345_DOUBLETAP            0x20
#define XL345_SINGLETAP            0x40
#define XL345_DATAREADY            0x80

// Bit values in POWER_CTL
#define XL345_WAKEUP_8HZ           0x00
#define XL345_WAKEUP_4HZ           0x01
#define XL345_WAKEUP_2HZ           0x02
#define XL345_WAKEUP_1HZ           0x03
#define XL345_SLEEP                0x04
#define XL345_MEASURE              0x08
#define XL345_STANDBY              0x00
#define XL345_AUTO_SLEEP           0x10
#define XL345_ACT_INACT_SERIAL     0x20
#define XL345_ACT_INACT_CONCURRENT 0x00

// Register List
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_FIFO_CTL    0x38
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_INT_ENABLE  0x2E  // default value: 0x00
#define ADXL345_REG_INT_MAP     0x2F  // default value: 0x00
#define ADXL345_REG_INT_SOURCE  0x30  // default value: 0x02
#define ADXL345_REG_DATA_FORMAT 0x31  // defuault value: 0x00
#define ADXL345_REG_DATAX0      0x32  // read only
#define ADXL345_REG_DATAX1      0x33  // read only
#define ADXL345_REG_DATAY0      0x34  // read only
#define ADXL345_REG_DATAY1      0x35  // read only
#define ADXL345_REG_DATAZ0      0x36  // read only
#define ADXL345_REG_DATAZ1      0x37  // read only


typedef struct {
  int                   file;
  balance_msgs::Vector3 last;
  bool                  new_measurement;
} ADXL345_T;


bool ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value)
{
  bool success = false;
  uint8_t values[2];

  values[0] = address;
  values[1] = value;
  success = write(file, &values, sizeof(values)) == sizeof(values);
        
  return success;
}


bool ADXL345_REG_READ(int file, uint8_t address, uint8_t *value_out)
{
  bool success = false;
  uint8_t value;
        
  // write to define register
  if (write(file, &address, sizeof(address)) == sizeof(address))
  {
    // read back value
    if (read(file, &value, sizeof(value)) == sizeof(value))
    {
      *value_out = value;
      success = true;
    }
  }

  return success;
}


bool ADXL345_REG_MULTI_READ(int file, uint8_t readaddr, uint8_t readdata[], uint8_t len)
{
  bool success = false;

  // write to define register
  if (write(file, &readaddr, sizeof(readaddr)) == sizeof(readaddr))
  {
    // read back value
    if (read(file, readdata, len) == len)
    {
      success = true;
    }
  }
        
  return success;
}


int adxl345_init(ADXL345_T* info)
{
  ROS_INFO("Opening connection to ADXL345");

  // Open i2c bus driver
  if ((info->file = open("/dev/i2c-0", O_RDWR)) < 0)
  {
    ROS_FATAL("i2c dev file could not be opened");
    return -1;
  }

  // Specify the sensor's adddress
  if (ioctl(info->file, I2C_SLAVE, ADXL345_ADDRESS) < 0)
  {
    ROS_FATAL("Failed to aquire bus access and/or talk to slave");
    return -1;
  }
  // You can check errno to see what went wrong

  // Configuration
  // -------------
 
  // Set range and resolution
  if (!ADXL345_REG_WRITE(info->file, ADXL345_REG_DATA_FORMAT, XL345_RANGE_4G | XL345_FULL_RESOLUTION))
  {
    ROS_FATAL("Failed to set data range and resolution");
    return -1;
  }

  // Set Data Rate
  if (!ADXL345_REG_WRITE(info->file, ADXL345_REG_BW_RATE, XL345_RATE_200))
  {
    ROS_FATAL("Failed to set data rate");
    return -1;
  }

  if (!ADXL345_REG_WRITE(info->file, ADXL345_REG_INT_ENABLE, XL345_DATAREADY))
  {
    ROS_FATAL("Failed to enable measurements");
    return -1;
  }

  if (!ADXL345_REG_WRITE(info->file, ADXL345_REG_POWER_CTL, XL345_STANDBY))
  {
    ROS_FATAL("Failed to stop measurements");
    return -1;
  }

  if (!ADXL345_REG_WRITE(info->file, ADXL345_REG_POWER_CTL, XL345_MEASURE))
  {
    ROS_FATAL("Failed to start measurements");
    return -1;
  }

  // Initialize last Vector
  info->last.x = 0;
  info->last.y = 0;
  info->last.z = 0;

  info->new_measurement = false;

  return 0;
}

void adxl345_close(const ADXL345_T* info)
{
  ROS_INFO("Closing connection to ADXL345");

  if (info->file)
    close(info->file);

  //fclose(info->fp);
}

void adxl345_get_data(ADXL345_T* info)
{
  uint8_t data8[6];
  uint8_t interrupt_data;
  bool success;
  int mg_per_lsb = 4;

  success = ADXL345_REG_READ(info->file, ADXL345_REG_INT_SOURCE, &interrupt_data);

  // Only read data if there is new data
  if (success && (interrupt_data & XL345_DATAREADY))
  {
    success = ADXL345_REG_MULTI_READ(info->file, ADXL345_REG_DATAX0, (uint8_t *) &data8, sizeof(data8));

    if (success)
    {
      info->last.x = (int16_t)((data8[1] << 8) | data8[0])*mg_per_lsb;
      info->last.y = (int16_t)((data8[3] << 8) | data8[2])*mg_per_lsb;
      info->last.z = (int16_t)((data8[5] << 8) | data8[4])*mg_per_lsb;

      info->new_measurement = true;
    }
    else
    {
      info->new_measurement = false;
    }
  }
  else
  {
    info->new_measurement = false;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "adxl345");

  ros::NodeHandle n;

  ros::Publisher accel_topic = n.advertise<balance_msgs::Vector3>("adxl345", 1000);

  ros::Rate loop_rate(10);

  ADXL345_T adxl;
  adxl345_init(&adxl);

  while (ros::ok())
  {
    adxl345_get_data(&adxl);

    if (adxl.new_measurement) 
      accel_topic.publish(adxl.last);

    ros::spinOnce();
    loop_rate.sleep();
  }

  adxl345_close(&adxl);

  return 0;
}

