/* Interface node for the FPGA via the LWAXI interface.
 */

#include "ros/ros.h"
#include "balance_msgs/PairF.h"


class PIController
{
  public:

  float intg_part; 
  float k_p, k_i;
  float desired_position;
  float actual_position;
  float voltage;

  PIController()
  {
    intg_part = 0;
    k_p = 10;
    k_i = 0;

    desired_position = 0;
    actual_position = 0;
    voltage = 0;
  }

  void update()
  {
    float speed_error = desired_position - actual_position;

    float prop_part = speed_error * k_p;
    intg_part += speed_error * k_i * 0.01;

    voltage = speed_error * k_p;
  }

};


class Node
{
  public:
    Node();
    ~Node();

  private:
    ros::NodeHandle n_;
    ros::Subscriber ref_sub_, pos_sub_;
    ros::Publisher  voltage_pub_;
    ros::Timer      timer_;

    PIController controller_;

    void ref_sub_callback(const balance_msgs::PairF::ConstPtr&);
    void pos_sub_callback(const balance_msgs::PairF::ConstPtr&);

    void publish_callback(const ros::TimerEvent&);
};

Node::Node()
{
  ref_sub_     = n_.subscribe("motor_refs", 1000, &Node::ref_sub_callback, this);
  pos_sub_     = n_.subscribe("positions", 1000, &Node::pos_sub_callback, this);

  voltage_pub_ = n_.advertise<balance_msgs::PairF>("voltages", 1000);
  timer_       = n_.createTimer(ros::Duration(0.01), &Node::publish_callback, this);
}

Node::~Node()
{
}

void Node::ref_sub_callback(const balance_msgs::PairF::ConstPtr& motor_refs)
{
  controller_.desired_position = motor_refs->left;
}

void Node::pos_sub_callback(const balance_msgs::PairF::ConstPtr& positions)
{
  controller_.actual_position = positions->left;
}

void Node::publish_callback(const ros::TimerEvent& event)
{
  balance_msgs::PairF voltages;

  controller_.update();

  voltages.left = controller_.voltage;

  voltage_pub_.publish(voltages);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwaxi");

  Node node();

  ros::spin();

  return 0;
}

