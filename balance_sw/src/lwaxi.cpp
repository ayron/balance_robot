/* Interface node for the FPGA via the LWAXI interface.
 */

#include "ros/ros.h"
#include "balance_msgs/PairU16.h"
#include "balance_msgs/PairI32.h"
#include "balance_msgs/PairI16.h"

extern "C"
{
  #include <unistd.h>         // write, read, close
  #include <fcntl.h>          // open, O_RDWR, O_SYNC
  #include <sys/mman.h>       // mmap, PROT_READ, PROT_WRITE, MAP_SHARED

  #include <stdlib.h>
  #include <string.h>

  #define soc_cv_av
  #include "hwlib.h"

  #include "soc_cv_av/socal/socal.h"
  #include "soc_cv_av/socal/hps.h"
  #include "soc_cv_av/socal/alt_gpio.h"

  #include "hps_0.h"
}


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )


typedef struct {
  int fd;
  void *virtual_base;
} LWAXI_T;


void Set(const LWAXI_T* info, unsigned long pio_base, const void* input, size_t num)
{
  void *ref_addr; 
  ref_addr = info->virtual_base + ( (unsigned long)(ALT_LWFPGASLVS_OFST + pio_base) & (unsigned long)(HW_REGS_MASK) );

  memcpy(ref_addr, input, num);
}


void Get(const LWAXI_T* info, unsigned long pio_base, void* output, size_t num)
{
  void *ref_addr; 
  ref_addr = info->virtual_base + ( (unsigned long)(ALT_LWFPGASLVS_OFST + pio_base) & (unsigned long)(HW_REGS_MASK) );

  memcpy(output, ref_addr, num);
}


int lwaxi_init(LWAXI_T* info)
{
  // Open the /dev/mem, which provides access to the
  // OSs physical memory. (ie. open the ram) 
  info->fd = open( "/dev/mem", O_RDWR | O_SYNC);

  if (info->fd == -1)
  {
		ROS_FATAL("ERROR: could not open /dev/mem...");
    return -1;
  }

  // mmap allows us to map the contents of a file ('/dev/mem' in this case)
  // to the programs memory (accessed via a pointer).
  // Thus this allows us to access the RAM directly by reading and writing
  // to virtual_base;
  // We specify the specific areas of the physical memory that are shared
  // with the FPGA, as indicated by HW_REGS_BASE and HW_REGS_SPAN
	info->virtual_base = mmap(NULL, HW_REGS_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, info->fd, HW_REGS_BASE);

  if (info->virtual_base == MAP_FAILED)
  {
    close(info->fd);
		ROS_FATAL("ERROR: mmap() failed...");
    return -1;
	}

  return 0;
}


int lwaxi_close(const LWAXI_T* info)
{
  // Set all outputs to zero
  uint16_t zero = 0;
  Set(info, LEFT_PWM_PIO_BASE,  &zero, LEFT_PWM_PIO_DATA_WIDTH/8);
  Set(info, RIGHT_PWM_PIO_BASE, &zero, RIGHT_PWM_PIO_DATA_WIDTH/8);

  // clean up our memory mapping and exit
  if (munmap(info->virtual_base, HW_REGS_SPAN) != 0)
  {
    close(info->fd);
    ROS_FATAL("ERROR: munmap() failed...");
    return -1;
  }

  if (info->fd)
    close(info->fd);

  return 0;
}


class Node
{
  public:
    Node();
    ~Node();

  private:
    ros::NodeHandle n_;
    ros::Subscriber pwms_sub_;
    ros::Publisher  adcs_pub_;
    ros::Publisher  counts_pub_;
    ros::Timer      timer_;

    LWAXI_T axi_;

    void subscribe_callback(const balance_msgs::PairI16::ConstPtr&);
    void publish_callback(const ros::TimerEvent&);
};


Node::Node()
{
  lwaxi_init(&axi_);

  pwms_sub_    = n_.subscribe("pwms", 1000, &Node::subscribe_callback, this);
  adcs_pub_    = n_.advertise<balance_msgs::PairU16>("adcs", 1000);
  counts_pub_  = n_.advertise<balance_msgs::PairI32>("counts", 1000);
  timer_       = n_.createTimer(ros::Duration(0.01), &Node::publish_callback, this);
}


Node::~Node()
{
  lwaxi_close(&axi_);
}


void Node::subscribe_callback(const balance_msgs::PairI16::ConstPtr& pwms)
{
  //ROS_INFO("I heard: [%d %d]", pwms->left, pwms->right);

  // Write FPGA outputs
  Set(&axi_, LEFT_PWM_PIO_BASE,      &pwms->left,      LEFT_PWM_PIO_DATA_WIDTH/8);
  Set(&axi_, RIGHT_PWM_PIO_BASE,     &pwms->right,     RIGHT_PWM_PIO_DATA_WIDTH/8);
}


void Node::publish_callback(const ros::TimerEvent& event)
{
  balance_msgs::PairI32 counts;
  balance_msgs::PairU16   adcs;

  // Get data from FPGA via LWAXI
  Get(&axi_, LEFT_CURRENT_PIO_BASE,  &adcs.left,      LEFT_CURRENT_PIO_DATA_WIDTH/8);
  Get(&axi_, RIGHT_CURRENT_PIO_BASE, &adcs.right,     RIGHT_CURRENT_PIO_DATA_WIDTH/8);
  Get(&axi_, LEFT_COUNT_PIO_BASE,    &counts.left,    LEFT_COUNT_PIO_DATA_WIDTH/8);
  Get(&axi_, RIGHT_COUNT_PIO_BASE,   &counts.right,   RIGHT_COUNT_PIO_DATA_WIDTH/8);

  adcs_pub_.publish(adcs);
  counts_pub_.publish(counts);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwaxi");

  Node node();

  ros::spin();

  return 0;
}

