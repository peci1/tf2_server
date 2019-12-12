#ifndef TF2_SERVER_TF2_SERVER_H
#define TF2_SERVER_TF2_SERVER_H

#include <map>
#include <memory>
#include <mutex>
#include <unordered_set>

#include <tf2_ros/buffer_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_server/RequestTransformStream.h>
#include <ros/ros.h>

namespace tf2_server
{

struct RequestTransformStreamRequestComparator
{
  bool operator()(const tf2_server::RequestTransformStreamRequest& r1,
                  const tf2_server::RequestTransformStreamRequest& r2) const;
};

class TF2Server
{

  protected: ros::NodeHandle& nh;
  protected: ros::NodeHandle& pnh;

  protected: std::unique_ptr<tf2_ros::Buffer> buffer;
  protected: std::unique_ptr<tf2_ros::BufferServer> server;
  protected: std::unique_ptr<tf2_ros::TransformListener> listener;
  protected: std::mutex mutex;

  protected: ros::ServiceServer requestTransformStreamServer;

  protected: std::unordered_set<std::string> topicNames;
  protected: std::map<RequestTransformStreamRequest, ros::Timer, RequestTransformStreamRequestComparator> timers;
  protected: std::map<RequestTransformStreamRequest, ros::Publisher, RequestTransformStreamRequestComparator> publishers;
  protected: std::map<RequestTransformStreamRequest, ros::Publisher, RequestTransformStreamRequestComparator> staticPublishers;
  protected: std::map<RequestTransformStreamRequest, tf2_msgs::TFMessage, RequestTransformStreamRequestComparator> lastStaticTransforms;

  typedef std::pair<std::string, std::string> FrameSpec;
  typedef std::vector<FrameSpec> FramesList;
  protected: std::map<RequestTransformStreamRequest, std::unique_ptr<FramesList>, RequestTransformStreamRequestComparator> frames;
  protected: std::map<RequestTransformStreamRequest, size_t, RequestTransformStreamRequestComparator> subscriberNumbers;

  protected: bool started = false;

  public: explicit TF2Server(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  public: virtual void start();

  protected: virtual bool onRequestTransformStream(RequestTransformStreamRequest& req, RequestTransformStreamResponse& resp);

  protected: virtual void streamTransform(const ros::TimerEvent& event, const RequestTransformStreamRequest& request);

  protected: virtual std::unique_ptr<FramesList> getFramesList(const RequestTransformStreamRequest& req) const;

  protected: virtual std::string getTopicName(const RequestTransformStreamRequest& request);
  protected: virtual std::string getStaticTopicName(const RequestTransformStreamRequest& request);

  protected: virtual void onSubscriberConnected(const RequestTransformStreamRequest& request);
  protected: virtual void onSubscriberDisconnected(const RequestTransformStreamRequest& request);

};

}

#endif //TF2_SERVER_TF2_SERVER_H
