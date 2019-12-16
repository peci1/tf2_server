#include <tf2_server/tf2_subtree_listener.h>
#include <tf2_server/RequestTransformStream.h>

#include <memory>

namespace tf2_server
{

TransformSubtreeListener::TransformSubtreeListener(
    const tf2_server::RequestTransformStreamRequest& subtree, tf2::BufferCore &buffer,
    bool spinThread, ros::Duration maxServerWait) :
    TransformSubtreeListener(subtree, buffer, ros::NodeHandle(), spinThread, std::move(maxServerWait)) {}

TransformSubtreeListener::TransformSubtreeListener(
    const tf2_server::RequestTransformStreamRequest& subtree, tf2::BufferCore &buffer,
    const ros::NodeHandle &nh, bool spinThread, ros::Duration maxServerWait) :
    buffer(buffer), nh(nh)
{
  ros::NodeHandle serverNh(this->nh, "tf2_server");
  ros::ServiceClient client = serverNh.serviceClient<tf2_server::RequestTransformStream>(
      "request_transform_stream");

  ROS_INFO_NAMED("tf2_subtree_listener", "Waiting for service %s", this->nh.resolveName(client.getService()).c_str());
  const auto serverExists = client.waitForExistence(maxServerWait);
  if (!serverExists)
    throw tf2::ConnectivityException("Service " + this->nh.resolveName(client.getService()) + " is not available.");
  ROS_INFO_NAMED("tf2_subtree_listener", "Service %s is available now", this->nh.resolveName(client.getService()).c_str());

  tf2_server::RequestTransformStreamResponse topics;

  ROS_INFO_NAMED("tf2_subtree_listener", "Requesting topic names for transform subtree");
  const auto succeeded = client.call(subtree, topics);
  if (!succeeded)
    throw tf2::ConnectivityException("Could not determine transform subtree topics.");

  const ros::M_string remap = {
      {"/tf", topics.topic_name},
      {"/tf_static", topics.static_topic_name}
  };

  ros::NodeHandle remappedNh(this->nh, std::string(), remap);

  this->listener = std::make_unique<tf2_ros::TransformListener>(this->buffer, remappedNh, spinThread);

  ROS_INFO_NAMED("tf2_subtree_listener", "Created transform subtree listener with /tf:=%s and /tf_static:=%s",
      this->nh.resolveName(topics.topic_name).c_str(), this->nh.resolveName(topics.static_topic_name).c_str());
}
}
