#include <tf2_server/tf2_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

namespace tf2_server
{

void warnLeadingSlash(const std::string& s)
{
  ROS_WARN_STREAM("Found initial slash in " << s);
}

std::string stripLeadingSlash(const std::string &s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn) {
      warnLeadingSlash(s);
    }
    return s.substr(1);
  }

  return s;
}

TF2Server::TF2Server(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh(nh), pnh(pnh)
{
  double buffer_size;
  this->nh.param("buffer_size", buffer_size, 120.0);

  bool publish_frame_service;
  this->nh.param("publish_frame_service", publish_frame_service, false);

  // Legacy behavior re: #209
  bool use_node_namespace;
  this->nh.param("use_node_namespace", use_node_namespace, false);
  std::string node_name;
  if (use_node_namespace)
  {
    node_name = ros::this_node::getName();
  }
  else
  {
    node_name = "tf2_buffer_server";
  }

  this->buffer =
      std::make_unique<tf2_ros::Buffer>(ros::Duration(buffer_size), publish_frame_service);
  this->listener = std::make_unique<tf2_ros::TransformListener>(*buffer, this->nh);
  this->server = std::make_unique<tf2_ros::BufferServer>(*buffer, node_name, false);

  this->requestTransformStreamServer =
      pnh.advertiseService("request_transform_stream", &TF2Server::onRequestTransformStream, this);
}

void TF2Server::start()
{
  this->server->start();
  this->started = true;
}

bool operator==(const tf2_msgs::TFMessage& lhs, const tf2_msgs::TFMessage& rhs)
{
  if (lhs.transforms.size() != rhs.transforms.size())
    return false;

  for (size_t i = 0; i < lhs.transforms.size(); ++i)
  {
    const auto& lt = lhs.transforms[i];
    const auto& rt = rhs.transforms[i];

    if (lt.header.stamp != rt.header.stamp) return false;
    if (lt.header.frame_id != rt.header.frame_id) return false;
    if (lt.child_frame_id != rt.child_frame_id) return false;
    if (lt.transform.translation.x != rt.transform.translation.x) return false;
    if (lt.transform.translation.y != rt.transform.translation.y) return false;
    if (lt.transform.translation.z != rt.transform.translation.z) return false;
    if (lt.transform.rotation.x != rt.transform.rotation.x) return false;
    if (lt.transform.rotation.y != rt.transform.rotation.y) return false;
    if (lt.transform.rotation.z != rt.transform.rotation.z) return false;
    if (lt.transform.rotation.w != rt.transform.rotation.w) return false;
  }

  return true;
}

bool operator!=(const tf2_msgs::TFMessage& lhs, const tf2_msgs::TFMessage& rhs)
{
  return !(lhs == rhs);
}

bool TF2Server::onRequestTransformStream(RequestTransformStreamRequest &req,
                                         RequestTransformStreamResponse &resp)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  const auto topicName = this->getTopicName(req);
  if (topicName.empty())
    return false;

  auto framesList = this->getFramesList(req);
  if (framesList->empty())
    throw std::runtime_error("Could not find any child frames of frame " + req.parent_frame);

  this->frames[req] = std::move(framesList);

  this->timers[req] =  this->nh.createTimer(req.publication_period,
      std::bind(&TF2Server::streamTransform, this, std::placeholders::_1, req), false, false);

  this->publishers[req] =
      this->nh.advertise<tf2_msgs::TFMessage>(topicName, req.publisher_queue_size,
          std::bind(&TF2Server::onSubscriberConnected, this, req),
          std::bind(&TF2Server::onSubscriberDisconnected, this, req));

  const auto staticTopicName = this->getStaticTopicName(req);
  this->staticPublishers[req] =
      this->nh.advertise<tf2_msgs::TFMessage>(staticTopicName, req.publisher_queue_size,
          std::bind(&TF2Server::onSubscriberConnected, this, req),
          std::bind(&TF2Server::onSubscriberDisconnected, this, req),
          ros::VoidConstPtr(), true);

  resp.topic_name = topicName;
  resp.static_topic_name = staticTopicName;

  return true;
}

void TF2Server::streamTransform(const ros::TimerEvent &,
                                const RequestTransformStreamRequest &request)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  const ros::Duration timeout(request.publication_period.toSec() * 0.9 / this->frames[request]->size());
  tf2_msgs::TFMessage msg;
  tf2_msgs::TFMessage staticMsg;

  for (const auto& frame : *this->frames[request])
  {
    try
    {
      const auto transform = this->buffer->lookupTransform(frame.first, frame.second, ros::Time(0), timeout);
      if (transform.header.stamp != ros::Time(0, 0)) // static transforms are returned with zero timestamp
        msg.transforms.push_back(transform);
      else
        staticMsg.transforms.push_back(transform);
    }
    catch (tf2::TransformException &e)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Error publishing transform stream: " << e.what());
    }
  }

  this->publishers[request].publish(msg);
  if (staticMsg != this->lastStaticTransforms[request])
  {
    this->staticPublishers[request].publish(staticMsg);
    this->lastStaticTransforms[request] = staticMsg;
  }
}

std::string TF2Server::getTopicName(const RequestTransformStreamRequest &request)
{
  if (this->publishers.find(request) != this->publishers.end())
    return this->pnh.resolveName(this->publishers.at(request).getTopic());

  const auto baseName = ros::names::append("streams", stripLeadingSlash(request.parent_frame, true));
  std::string error;

  for (size_t i = 0; i < 10000; ++i)
  {
    const auto topicName = ros::names::append(baseName, "stream_" + std::to_string(i));
    if (ros::names::validate(topicName, error) && this->topicNames.find(topicName) == this->topicNames.end())
    {
      this->topicNames.insert(topicName);
      return this->pnh.resolveName(topicName);
    }
  }

  ROS_ERROR_STREAM("Error generating topic name: " << error);
  return "";
}

std::string TF2Server::getStaticTopicName(const RequestTransformStreamRequest &request)
{
  if (this->staticPublishers.find(request) != this->staticPublishers.end())
    return this->pnh.resolveName(this->staticPublishers.at(request).getTopic());

  return ros::names::append(this->getTopicName(request), "static");
}

std::unique_ptr<TF2Server::FramesList> TF2Server::getFramesList(const RequestTransformStreamRequest &req) const
{
  if (!this->buffer->_frameExists(req.parent_frame))
    throw tf2::LookupException("Frame " + req.parent_frame + " doesn't exist.");

  auto result = std::make_unique<TF2Server::FramesList>();
  if (!req.intermediate_frames)
  { // if intermediate frames are not requested, we just publish pairs of parent and all children
    result->reserve(req.child_frames.size());
    for (const auto& child : req.child_frames)
    {
      if (!this->buffer->_frameExists(child))
      {
        ROS_WARN("Frame %s doesn't exist, it won't be streamed.", child.c_str());
        continue;
      }
      result->emplace_back(req.parent_frame, child);
    }
  }
  else
  { // otherwise, we need to get a whole subtree
    std::vector<std::string> chainFrames;
    std::vector<std::string> childFrames;

    if (!req.child_frames.empty())
    { // only subtree to specific children is requested
      for (const auto& child : req.child_frames)
      {
        if (!this->buffer->_frameExists(child))
        {
          ROS_WARN("Frame %s doesn't exist, it won't be streamed.", child.c_str());
          continue;
        }
        childFrames.push_back(child);
      }
    }
    else
    { // whole subtree is requested; we don't know what children it has, so we have to find them
      std::vector<std::string> allFrames;
      this->buffer->_getFrameStrings(allFrames);

      std::string parentParent; // parent frame of req.parent_frame
      if (!this->buffer->_getParent(req.parent_frame, ros::Time(0), parentParent))
        parentParent = "";  // if req.parent_frame is the topmost frame

      for (const auto& frame : allFrames)
      {
        if (frame == req.parent_frame)
          continue;

        chainFrames.clear();
        try
        {
          this->buffer->_chainAsVector(frame, ros::Time(0), req.parent_frame, ros::Time(0), req.parent_frame, chainFrames);
        }
        catch (tf2::TransformException& e)
        {
          ROS_ERROR("Error while searching TF tree: %s", e.what());
          continue;
        }

        // if chainFrames[1] is parentParent, it means that the path to the given frame does not
        // descend into req.parent_frame's subtree, but it goes above; we throw away such paths
        if (chainFrames[1] != parentParent)
          childFrames.push_back(frame);
      }
    }

    // collect all unique pairs of parent-child in the requested subtree

    std::set<TF2Server::FrameSpec> framePairs;
    for (const auto& child : childFrames)
    {
      chainFrames.clear();
      this->buffer->_chainAsVector(child, ros::Time(0), req.parent_frame, ros::Time(0), req.parent_frame, chainFrames);

      if (chainFrames.size() <= 1)
        continue;

      for (size_t i = 1; i < chainFrames.size(); ++i)
        framePairs.emplace(chainFrames[i-1], chainFrames[i]);
    }

    result->insert(result->begin(), framePairs.begin(), framePairs.end());
  }
  return result;
}

void TF2Server::onSubscriberConnected(const RequestTransformStreamRequest &request)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->subscriberNumbers[request] = this->subscriberNumbers[request] + 1;
  if (this->subscriberNumbers[request] == 1)
    ROS_INFO("Started streaming %s", this->publishers[request].getTopic().c_str());

  this->timers[request].start();
}

void TF2Server::onSubscriberDisconnected(const RequestTransformStreamRequest &request)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->subscriberNumbers[request] = this->subscriberNumbers[request] - 1;
  if (this->subscriberNumbers[request] == 0)
  {
    ROS_INFO("Stopped streaming %s", this->publishers[request].getTopic().c_str());
    this->timers[request].stop();
  }
}

bool RequestTransformStreamRequestComparator::operator()(
    const tf2_server::RequestTransformStreamRequest &r1,
    const tf2_server::RequestTransformStreamRequest &r2) const
{
  if (r1.intermediate_frames != r2.intermediate_frames)
    return r1.intermediate_frames < r2.intermediate_frames;
  else if (r1.publication_period != r2.publication_period)
    return r1.publication_period < r2.publication_period;
  else if (r1.publisher_queue_size != r2.publisher_queue_size)
    return r1.publisher_queue_size < r2.publisher_queue_size;
  else if (r1.parent_frame != r2.parent_frame)
    return r1.parent_frame < r2.parent_frame;
  else if (r1.child_frames.size() != r2.child_frames.size())
    return r1.child_frames.size() < r2.child_frames.size();
  else
  {
    for (size_t i = 0; i < r1.child_frames.size(); ++i)
    {
      if (r1.child_frames[i] != r2.child_frames[i])
        return r1.child_frames[i] < r2.child_frames[i];
    }
    return false;
  }
}

}