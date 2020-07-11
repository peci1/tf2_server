#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf2_msgs/TFMessage.h>
#include <pluginlib/class_list_macros.h>

namespace tf2_server
{

/**
 * This is a helper class for test tf2_server_early_init_params.test . Its task is to subscribe to a TF topic as a
 * nodelet (and relay it further). This should normally make no practical difference to running as a node, but it shows
 * that nodelet subscribers call connect_cb much faster/earlier than interprocess subscribers. And this speed was the
 * culprit of a nasty race condition for which we test now.
 */

class Tf2ServerEarlyInitialParamsNodelet : public nodelet::Nodelet
{
protected:
  void onInit() override
  {
    this->sub = this->getNodeHandle().subscribe("/tf", 10, &Tf2ServerEarlyInitialParamsNodelet::onMessage, this);
  }

  void onMessage(const tf2_msgs::TFMessageConstPtr& msg)
  {
    if (!this->pub)
    {
      this->pub = this->getNodeHandle().advertise<tf2_msgs::TFMessage>("/test", 10);
      ros::WallDuration(0.01).sleep();
    }
    this->pub.publish(msg);
  }

private:
  ros::Subscriber sub;
  ros::Publisher pub;
};

}

PLUGINLIB_EXPORT_CLASS(tf2_server::Tf2ServerEarlyInitialParamsNodelet, nodelet::Nodelet)