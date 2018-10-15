/* Interface node for the FPGA via the LWAXI interface.
 */

#include "ros/ros.h"
#include "balance_msgs/PairI16.h"
#include "balance_msgs/PairF.h"


const float BAT_MAX_VOLT = 11.0;
const float PWM_MAX_COUNT = 2500.0;
const float VOLT_TO_PWM =  PWM_MAX_COUNT / BAT_MAX_VOLT;


uint16_t in_range(uint16_t in, float low, float high)
{
  if (in > high)
    return high;

  else if (in < low)
    return low;

  else
    return in;
}


class Node
{
  public:
    Node();

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    void subscribe_callback(const balance_msgs::PairF::ConstPtr&);
};

Node::Node()
{
  sub_    = n_.subscribe("voltages", 1000, &Node::subscribe_callback, this);
  pub_    = n_.advertise<balance_msgs::PairI16>("pwms", 1000);
}

void Node::subscribe_callback(const balance_msgs::PairF::ConstPtr& voltages)
{
  balance_msgs::PairI16 pwms;

  pwms.left  = voltages->left  * VOLT_TO_PWM;
  pwms.right = voltages->right * VOLT_TO_PWM;

  pwms.left  = in_range(pwms.left,  -PWM_MAX_COUNT, PWM_MAX_COUNT);
  pwms.right = in_range(pwms.right, -PWM_MAX_COUNT, PWM_MAX_COUNT);

  pub_.publish(pwms);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pwms");

  Node node();

  ros::spin();

  return 0;
}

