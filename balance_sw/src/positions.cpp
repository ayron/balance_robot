/* Interface node for the FPGA via the LWAXI interface.
 */

#include "ros/ros.h"
#include "balance_msgs/PairI32.h"
#include "balance_msgs/PairF.h"


const float PI             = 3.14159265;
const float GEAR_RATIO     = (22.0f*22*22*23)/(12.0f*10*10*10);
const float COUNTS_PER_REV = 48.0;
const float RADS_PER_REV   = 2*PI;
const float COUNTS_TO_RAD  = RADS_PER_REV / COUNTS_PER_REV / GEAR_RATIO;


class Node
{
  public:
    Node();

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    void subscribe_callback(const balance_msgs::PairI32::ConstPtr&);
};

Node::Node()
{
  sub_    = n_.subscribe("counts", 1000, &Node::subscribe_callback, this);
  pub_    = n_.advertise<balance_msgs::PairF>("positions", 1000);
}

void Node::subscribe_callback(const balance_msgs::PairI32::ConstPtr& counts)
{
  balance_msgs::PairF positions;

  positions.left  = counts->left  * COUNTS_TO_RAD;
  positions.right = counts->right * COUNTS_TO_RAD;

  pub_.publish(positions);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "positions");

  Node node();

  ros::spin();

  return 0;
}

