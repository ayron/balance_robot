/* Openloop motor test sequence.
 */

#include "ros/ros.h"
#include "balance_msgs/PairF.h"


void test_sequence(float elapsed_time, float* voltage, int* done)
{
  // Sweep
  if (elapsed_time >= 1000 && elapsed_time < 5000)
    *voltage = (elapsed_time - 1000)*(11.0/4000);
  else if (elapsed_time >= 5000 && elapsed_time < 13000)
    *voltage = -(elapsed_time - 9000)*(11.0/4000);
  else if (elapsed_time >= 13000 && elapsed_time < 17000)
    *voltage = (elapsed_time - 17000)*(11.0/4000);

  // Steps
  else if (elapsed_time >= 18000 && elapsed_time < 19000)
    *voltage = 3;
  else if (elapsed_time >= 20000 && elapsed_time < 21000)
    *voltage = -3;

  else if (elapsed_time >= 22000 && elapsed_time < 23000)
    *voltage = 7;
  else if (elapsed_time >= 24000 && elapsed_time < 25000)
    *voltage = -7;

  else if (elapsed_time >= 26000 && elapsed_time < 27000)
    *voltage = 11;
  else if (elapsed_time >= 28000 && elapsed_time < 29000)
    *voltage = -11;

  else if (elapsed_time >= 30000)
    *done = 1;
}


class PClass
{
  public:

    ros::Publisher* topic;

    void callback(const balance_msgs::PairI16::ConstPtr& volts)
    {
      balance_msgs::PairI16 pwms;

      pwms.left  = volts->left  * VOLT_TO_PWM;
      pwms.right = volts->right * VOLT_TO_PWM;

      this->topic->publish(pwms);
    }


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_test");

  ros::NodeHandle n;
 
  ros::Publisher topic = n.advertise<balance_msgs::PairF>("volts", 1000);

  PClass process;
  process.topic = &topic;

  ros::Subscriber control_topic = n.subscribe("control", 1000, &PClass::callback, &process);

  ros::spin();

  return 0;
}

