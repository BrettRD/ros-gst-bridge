/* gst_bridge
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _GST_ROSCOMPIMAGESINK_H_
#define _GST_ROSCOMPIMAGESINK_H_

#include <gst/base/gstbasesink.h>
#include <gst/video/video-format.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesink.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

G_BEGIN_DECLS

#define GST_TYPE_ROSCOMPIMAGESINK (roscompimagesink_get_type())
#define GST_ROSCOMPIMAGESINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSCOMPIMAGESINK, Roscompimagesink))
#define GST_ROSCOMPIMAGESINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSCOMPIMAGESINK, RoscompimagesinkClass))
#define GST_IS_ROSCOMPIMAGESINK(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROSCOMPIMAGESINK))
#define GST_IS_ROSCOMPIMAGESINK_CLASS(obj) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROSCOMPIMAGESINK))

typedef struct _Roscompimagesink Roscompimagesink;
typedef struct _RoscompimagesinkClass RoscompimagesinkClass;

struct _Roscompimagesink
{
  RosBaseSink parent;

  gchar * pub_topic;
  gchar * frame_id;
  gchar * encoding;   //image topic encoding string
  gchar * init_caps;  //optional caps override (used for limited apis)

  gchar * format;
  
  // New fields for timestamp handling
  gboolean pts_initialized;  // Whether we've initialized the PTS reference
  GstClockTime base_pts;     // Base PTS value for reference
  GstClockTime base_sys_time; // System time corresponding to base_pts

  std::shared_ptr<sensor_msgs::msg::CompressedImage> msg;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;
};

struct _RoscompimagesinkClass
{
  RosBaseSinkClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType roscompimagesink_get_type(void);

G_END_DECLS

#endif
