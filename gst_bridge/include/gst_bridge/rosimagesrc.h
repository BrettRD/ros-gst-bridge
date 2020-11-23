/* GStreamer
 * Copyright (C) 2020 FIXME <fixme@example.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifndef _GST_ROSIMAGESRC_H_
#define _GST_ROSIMAGESRC_H_

#include <gst/video/video-format.h>
#include <gst/base/gstbasesrc.h>
#include <gst_bridge/gst_bridge.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


G_BEGIN_DECLS

#define GST_TYPE_ROSIMAGESRC   (rosimagesrc_get_type())
#define GST_ROSIMAGESRC(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSIMAGESRC,Rosimagesrc))
#define GST_ROSIMAGESRC_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSIMAGESRC,RosimagesrcClass))
#define GST_IS_ROSIMAGESRC(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSIMAGESRC))
#define GST_IS_ROSIMAGESRC_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSIMAGESRC))

typedef struct _Rosimagesrc Rosimagesrc;
typedef struct _RosimagesrcClass RosimagesrcClass;

struct _Rosimagesrc
{
  GstBaseSrc parent;
  gchar* node_name;
  gchar* node_namespace;
  gchar* sub_topic;
  gchar* frame_id;
  gchar* encoding;
  gchar* init_caps;

  bool msg_init;
  std::promise<sensor_msgs::msg::Image::ConstSharedPtr> new_msg;

  rclcpp::Context::SharedPtr ros_context;
  rclcpp::executor::Executor::SharedPtr ros_executor;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;
  GstClockTimeDiff ros_clock_offset;

  int height;
  int width;
  GstVideoFormat format;
  size_t step;   //bytes per pixel
  gint endianness;
};

struct _RosimagesrcClass
{
  GstBaseSrcClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rosimagesrc_get_type (void);

G_END_DECLS

#endif
