# tf2_server

![CI](https://github.com/peci1/tf2_server/workflows/CI/badge.svg)

An upgraded [tf2_ros/buffer_server](https://github.com/ros/geometry2/blob/melodic-devel/tf2_ros/src/buffer_server.cpp).
Main features are:

 - It supports the same action API as `tf2_ros/buffer_server` node for on-demand transform querying (which is however
   not suitable for fast queries).
 - It allows clients to request only subparts (or "streams") of the TF tree, which results in lowering the data
   transfer rates and computational burden of TF clients.
 - It provides C++ and Python helper libraries that make the subtree subscription easier. However, use of these
   libraries is optional and client code can make full use of subscription to a TF subtree just by correct configuration
   in `.launch` files.
 - It can be used as a TF concentrator which reduces the usual M:N connection nature of the TF topic to a M:1 and 1:N.

The subtree subscription mode is useful as a middle way between standard `TransformListener` and the action interface of
the `buffer_server` - it transmits only the requested transform(s), but isn't burdened with the overhead the action
interface brings in.

The server publishes the so called "TF streams", which are internally utilized by the helper libraries,
but can also be used by custom user code. The streams can either be requested dynamically by calling service
`~/request_transform_stream` or by setting an initial set of streams using the `streams` parameter described in the
following section.

## Nodes and nodelets

This package provides node `tf2_server_node` and nodelet `tf2_server/nodelet` with
equal functionality. Try running the nodelet version with your TF-hungry nodes.

## Actions

 - [`tf2_msgs/LookupTransform`](http://docs.ros.org/api/tf2_msgs/html/action/LookupTransform.html): Depending on value
   of parameter `~use_node_namespace`, this action is either published in namespace of the node, or in namespace
   `tf2_buffer_server`.

## Parameters

The first 3 parameters are taken from the original TF2 buffer server. To stay compatible, the parameters are not
node-private, but should be set "one level" higher, e.g. besides the node and not inside it.

 - `double` `buffer_size`: Duration of the buffer memory (in seconds). Default is 120.0.
 - `bool` `publish_frame_service`: If `True`, the server will enable service `~/tf2_frames` which shows debug information
   about the contents of the buffer. Default is `False`.
 - `bool` `use_node_namespace`: If `True`, the lookup transform action of this server will be published under the
   namespace of this tf2_server. If `False`, the action server will be published in namespace `tf2_buffer_server`.
   Default is `False`.


 - `double` `~transforms_update_period`: The period at which the server looks for
   newly added TF frames.
 - `double` `~initial_streams_wait_time`: Specifies how long to wait before 
   registering streams from parameter `streams` (in seconds). This helps in case
   not all frames are right away available and you do not want to dynamically update
   the lists of available frames.
 - `dict` `~streams`: If nonempty, specifies some TF streams that should be registered
   right after starting the server. Each stream has a name (its key in this dict).
   The stream will be published at `~/stream_key` and 
   `~/stream_key/static`. The structure is the following and is described
   in more detail in the following section:

```YAML
streams:
  body:
    parent_frame: 'base_link'
    child_frames: []
    intermediate_frames: True
    publication_period: 0.1
    publisher_queue_size: 11
    allow_transforms_update: True
```

## Services

 - `tf2_server/RequestTransformStream` `~/request_transform_stream`: Requests a transform
   stream satisfying the given parameters.
 - [`tf2_msgs/FrameGraph`](http://docs.ros.org/api/tf2_msgs/html/srv/FrameGraph.html) `~/tf2_frames`: Returns a list
   of all available frames seen by the buffer server.
   
## tf2_server_node publications and subscriptions

 - `/tf`, `/tf_static`: The original TF topics.
 - `~/stream_key`, `~/stream_key/static` for each `stream_key`

## RequestTransformStream

The subtree subscription API is built around [ReuqestTranformStream](srv/RequestTransformStream.srv)
service type. The subtree listeners are configured by RequestTransformStreamRequest
objects. The meaning of the individual fields is as follows:

 - `string` `parent_frame`: The top-most frame in TF tree you're interested in.
 - `string[]` `child_frames`: In case a nonempty list is given, this specifies the
  child transforms you are interested in. They don't need to be direct children
  of `parent_frame`, but they should be in the same TF tree. If an empty list is
  given, stream the whole subtree of `parent_frame`. This mode requires 
  `intermediate_frames` set to `True`.
 - `bool` `intermediate_frames`: If `False`, stream only direct
 `parent_frame`->`child_frame` transforms (no matter if they were originally
 direct neighbors). If `True`, stream all the transforms between `parent_frame`
 and all `child_frames`.
 - `duration` `publication_period`: How often to publish the transforms.
 - `bool` `allow_transforms_update`: If true, periodically checks for updates in
   subtree topology. Can even handle cases when `parent_frame` isn't reachable
   at the time of the request. Default is false.
 - `int32` `publisher_queue_size`: Queue size of the transform publisher.
 - `string` `requested_topic_name`: If nonempty, the stream will be published on
   the given topic. Leads to an error if another stream is only registered on
   this topic with incompatible settings.
 - `string` `requested_static_topic_name`: If nonempty, specifies the name of
   the stream with static transforms. Defaults to `$(requested_topic_name)/static`.
 
## Example usage

### tf2_server.launch

```XML
<launch>
    <param name="buffer_size" value="30.0"/>
    <param name="publish_frame_service" value="true"/>
    <param name="use_node_namespace" value="true"/>
    <node name="tf_server" pkg="tf2_server" type="tf2_server_node">
        <param name="transforms_update_period" value="10.0" />
        <rosparam>
            streams:
                odom:
                    parent_frame: 'odom'
                    child_frames: ['base_link']
                    intermediate_frames: True
                    publication_period: 0.02
                    publisher_queue_size: 10
                    allow_transforms_update: True
        </rosparam>
    </node>
</launch>
```
    
### C++ client

This C++ client code shows how to subscribe the whole TF subtree under the
`base_link` frame. The transforms in the buffer get updated once every 0.1 s.

```C++
#include <tf2_server/tf2_subtree_listener.h>

RequestTransformStreamRequest req;
req.parent_frame = "base_link";
req.child_frames = { };
req.intermediate_frames = true;
req.publisher_queue_size = 10;
req.publication_period = ros::Duration(0.1);

tf2_ros::Buffer buffer;
TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

...

buffer.canTransform("base_link", "left_track", ros::Time(0));
```
    
### Python client

This Python client example shows how to subscribe exactly two transforms:
`base_link`->`left_track` and `base_link`->`front_left_flipper_endpoint`.
No other transforms are transmitted to the client.

```Python
req = RequestTransformStreamRequest()
req.parent_frame = "base_link"
req.child_frames = ["left_track", "front_left_flipper_endpoint"]
req.intermediate_frames = False
req.publisher_queue_size = 10
req.publication_period = rospy.Duration(0.1)

buffer = Buffer()
listener = TransformSubtreeListener(req, buffer, max_server_wait=rospy.Duration(10))

...

buffer.can_transform("base_link", "left_track", rospy.Time(0))
```

### Connect a node to an already existing transform stream

```XML
<launch>
    <!-- This assumes you've already requested a TF subtree "odom", e.g. by putting it in the `streams` parameter. -->
    <node name="my_node" pkg="any_pkg" type="node">
        <remap from="/tf" to="tf2_server/streams/odom" />
        <remap from="/tf_static" to="tf2_server/streams/odom/static" />
    </node>
</launch>
```

## Principle of working of the subtree listener/publisher

The `TransformSubtreeListener` classes hide some implementation details from the
user. Here we describe what exactly happens when a subtree is requested.

First, service `~/request_transform_stream` is called with the given
subtree configuration. The server responds with a message that specifies
`topic_name` and `static_topic_name`, which are autogenerated topic names from
namespace `~/streams/*`. The client then remaps `/tf` and `/tf_static`
to these topics and subscribes to them.

On the server side, the autogenerated topics are cached and when a new client
asks for an already published subtree configuration, the already existing topic
names are returned. Each unique subtree configuration starts a timer in the
server node, which publishes the requested transforms on the requested rate.
When last client with a given configuration disconnects from its topics, the
timer is stopped so that it doesn't eat resources. But the topic remains and
anyone can connect later, which will start the timer again.  
