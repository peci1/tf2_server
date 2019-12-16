#include <tf2_server/tf2_subtree_listener.h>

#include <gtest/gtest.h>

using namespace tf2_server;

TEST(tf2_subtree_listener, Basic){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  req.intermediate_frames = false;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE( buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE( buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE( buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, PublicationPeriod){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  req.intermediate_frames = false;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(10);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, IntermediateFrames){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, Subtree){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_tf2_subtree_listener");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}