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

/**
 * SECTION:interface-rosbase
 *
 * The rosbase interface, aggregate ROS2 node complexity into a loadable interface.
 *
 */

#include <gst_bridge/rosbaseiface.h>


/* Gstreamer interface

G_DEFINE_INTERFACE (RosBase, rosbase, G_TYPE_OBJECT)


static void rosbase_default_init(RosBaseInterface * iface)
{
  g_object_interface_install_property(
    iface, g_param_spec_string(
             "ros-name", "node-name", "Name of the ROS node", "gst_bridge_node",
             (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_interface_install_property(
    iface, g_param_spec_string(
             "ros-namespace", "node-namespace", "Namespace for the ROS node", "",
             (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_interface_install_property(
    iface, g_param_spec_uint64(
             "ros-start-time", "ros-start-time", "ROS time (nanoseconds) of the first message", 0,
             (guint64)(-1), GST_CLOCK_TIME_NONE,
             (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}


static gboolean rosbase_open(RosBase *self)

*/

/* possible implementation

static void rosbaseimp_init(RosBaseInterface * iface)
{
  iface->open = rosbase_open;
  iface->close = rosbase_close;
}

void rosbaseimp_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSink * sink = GST_ROS_BASE_SINK(object);

  GST_DEBUG_OBJECT(sink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if (sink->node) {
        RCLCPP_ERROR(sink->logger, "can't change node name once opened");
      } else {
        g_free(sink->node_name);
        sink->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_NAMESPACE:
      if (sink->node) {
        RCLCPP_ERROR(sink->logger, "can't change node namespace once opened");
      } else {
        g_free(sink->node_namespace);
        sink->node_namespace = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_START_TIME:
      if (sink->node) {
        RCLCPP_ERROR(sink->logger, "can't change start_time once opened");
      } else {
        sink->stream_start_prop = g_value_get_uint64(value);
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

void rosbaseimp_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  RosBaseSink * sink = GST_ROS_BASE_SINK(object);

  GST_DEBUG_OBJECT(sink, "get_property");
  switch (property_id) {
    case PROP_ROS_NAME:
      g_value_set_string(value, sink->node_name);
      break;

    case PROP_ROS_NAMESPACE:
      g_value_set_string(value, sink->node_namespace);
      break;

    case PROP_ROS_START_TIME:
      g_value_set_uint64(value, sink->stream_start.nanoseconds());
      // XXX this allows inspection via props,
      //      but may cause confusion because it does not show the actual prop
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

*/



/* open a connection to the ROS graph */
// XXX allow the pipeline host to inject a node handle
gboolean rosbaseimp_open(RosBaseImp * self, gchar* node_name, gchar* node_namespace)
{
  gboolean result = TRUE;

  self->ros_context = std::make_shared<rclcpp::Context>();
  self->ros_context->init(0, NULL);  // XXX should expose the init arg list
  auto opts = rclcpp::NodeOptions();
  opts.context(self->ros_context);  //set a context to generate the node in
  self->node = std::make_shared<rclcpp::Node>(
    std::string(node_name), std::string(node_namespace), opts);

  auto ex_args = rclcpp::ExecutorOptions();
  ex_args.context = self->ros_context;
  self->ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(ex_args);
  self->ros_executor->add_node(self->node);
  //iface->ros_executor->spin_some();
  self->spin_thread = std::thread(
    [=](rclcpp::Executor::SharedPtr e){e->spin();},  // lambda spin
    self->ros_executor
  );

  return result;
}

/* close the connection to the ROS graph  */
gboolean rosbaseimp_close(RosBaseImp * iface)
{
  gboolean result = TRUE;

  GST_DEBUG_OBJECT(iface, "close");

  iface->ros_executor->cancel();
  iface->spin_thread.join();

  iface->ros_context->shutdown("gst closing rosbase");

  iface->node.reset();
  iface->ros_executor.reset();
  iface->ros_executor.reset();
  return result;
}
