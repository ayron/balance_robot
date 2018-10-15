/* Interface node for the FPGA via the LWAXI interface.
 */

#include "ros/ros.h"
#include "balance_msgs/PairU16.h"
#include "balance_msgs/PairF.h"


const int   LEFT_ADC_OFFSET  = 216;
const int   RIGHT_ADC_OFFSET = 216;  // To be updated
const float MV_TO_CURRENT    = 1.0f / 140;
const float ADC_TO_MV        = 5000.0f / 4096;
const float ADC_TO_CURRENT   = ADC_TO_MV * MV_TO_CURRENT;


class PClass
{
  public:

    ros::Publisher* topic;

    void callback(const balance_msgs::PairU16::ConstPtr& adcs)
    {
      balance_msgs::PairF currents;

      currents.left  = (adcs->left  -  LEFT_ADC_OFFSET) * ADC_TO_CURRENT;
      currents.right = (adcs->right - RIGHT_ADC_OFFSET) * ADC_TO_CURRENT;

      this->topic->publish(currents);
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "currents");

  ros::NodeHandle n;
 
  ros::Publisher  currents_topic  = n.advertise<balance_msgs::PairF>("currents", 1000);

  PClass adcs_process;
  adcs_process.topic = &currents_topic;

  ros::Subscriber adcs_topic      = n.subscribe("adcs", 1000, &PClass::callback, &adcs_process);

  ros::spin();

  return 0;
}

