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
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */
/**
 * SECTION:element-gstrosaudiosink
 *
 * The rosaudiosink element audio data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v audiotestsrc ! rosaudiosink node_name="gst_audio" topic="/audiotopic"
 * ]|
 * Streams test tones as ROS audio messages on topic.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/audio/gstaudiosink.h>
#include "gstrosaudiosink.h"


GST_DEBUG_CATEGORY_STATIC (rosaudiosink_debug_category);
#define GST_CAT_DEFAULT rosaudiosink_debug_category

/* prototypes */


static void rosaudiosink_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void rosaudiosink_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void rosaudiosink_dispose (GObject * object);
static void rosaudiosink_finalize (GObject * object);

static gboolean rosaudiosink_open (GstAudioSink * sink);
static gboolean rosaudiosink_prepare (GstAudioSink * sink,
    GstAudioRingBufferSpec * spec);
static gboolean rosaudiosink_unprepare (GstAudioSink * sink);
static gboolean rosaudiosink_close (GstAudioSink * sink);
static gint rosaudiosink_write (GstAudioSink * sink, gpointer data,
    guint length);
static guint rosaudiosink_delay (GstAudioSink * sink);
static void rosaudiosink_reset (GstAudioSink * sink);

enum
{
  PROP_0,
  PROP_ROS_NAME,
  PROP_ROS_TOPIC,
  PROP_ROS_ENCODING
};


/* pad templates */

/* FIXME add/remove the formats that you want to support */
static GstStaticPadTemplate rosaudiosink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw,format=S16LE,rate=[1,max],"
        "channels=[1,max],layout=interleaved")
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosaudiosink, rosaudiosink, GST_TYPE_AUDIO_SINK,
    GST_DEBUG_CATEGORY_INIT (rosaudiosink_debug_category, "rosaudiosink", 0,
        "debug category for rosaudiosink element"))

static void
rosaudiosink_class_init (RosaudiosinkClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstAudioSinkClass *audio_sink_class = GST_AUDIO_SINK_CLASS (klass);

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (GST_ELEMENT_CLASS (klass),
      &rosaudiosink_sink_template);

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "FIXME Long name", "Generic", "FIXME Description",
      "FIXME <fixme@example.com>");

  gobject_class->set_property = rosaudiosink_set_property;
  gobject_class->get_property = rosaudiosink_get_property;
  gobject_class->dispose = rosaudiosink_dispose;
  gobject_class->finalize = rosaudiosink_finalize;
  audio_sink_class->open = GST_DEBUG_FUNCPTR (rosaudiosink_open);
  audio_sink_class->prepare = GST_DEBUG_FUNCPTR (rosaudiosink_prepare);
  audio_sink_class->unprepare = GST_DEBUG_FUNCPTR (rosaudiosink_unprepare);
  audio_sink_class->close = GST_DEBUG_FUNCPTR (rosaudiosink_close);
  audio_sink_class->write = GST_DEBUG_FUNCPTR (rosaudiosink_write);
  audio_sink_class->delay = GST_DEBUG_FUNCPTR (rosaudiosink_delay);
  audio_sink_class->reset = GST_DEBUG_FUNCPTR (rosaudiosink_reset);

}

static void
rosaudiosink_init (Rosaudiosink * rosaudiosink)
{
  // Don't register the node or the publisher just yet,
  // wait for rosaudiosink_open()
  rosaudiosink->node_name = g_strdup("gst_audio_sink_node");
  rosaudiosink->pub_topic = g_strdup("gst_audio_pub");
  rosaudiosink->encoding = g_strdup("16SC1");
  
}

void
rosaudiosink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(rosaudiosink->node)
      {
        RCLCPP_ERROR(rosaudiosink->logger, "can't change node name once openned");
      }
      else
      {
        g_free(rosaudiosink->node_name);
        rosaudiosink->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_TOPIC:
      if(rosaudiosink->node)
      {
        RCLCPP_ERROR(rosaudiosink->logger, "can't change topic name once openned");
      }
      else
      {
        g_free(rosaudiosink->pub_topic);
        rosaudiosink->pub_topic = g_value_dup_string(value);
      }
      break;
    case PROP_ROS_ENCODING:
      rosaudiosink->encoding = g_value_dup_string(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
rosaudiosink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "get_property");
  switch (property_id) {
    case PROP_ROS_NAME:
      g_value_set_string(value, rosaudiosink->node_name);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, rosaudiosink->pub_topic);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, rosaudiosink->encoding);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
rosaudiosink_dispose (GObject * object)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (rosaudiosink_parent_class)->dispose (object);
}

void
rosaudiosink_finalize (GObject * object)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (rosaudiosink_parent_class)->finalize (object);
}

/* open the device with given specs */
static gboolean
rosaudiosink_open (GstAudioSink * sink)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "open");

  rosaudiosink->node = std::make_shared<rclcpp::Node>(rosaudiosink->node_name);
  rosaudiosink->pub = rosaudiosink->node->create_publisher<sensor_msgs::msg::Image>(rosaudiosink->pub_topic, 1);
  rosaudiosink->logger = rosaudiosink->node->get_logger();
  return TRUE;
}

/* prepare resources and state to operate with the given specs */
static gboolean
rosaudiosink_prepare (GstAudioSink * sink, GstAudioRingBufferSpec * spec)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "prepare");

  if(rosaudiosink->node)
      RCLCPP_INFO(rosaudiosink->logger, "preparing audio with caps '%s', format '%s'",
          gst_caps_to_string(spec->caps), gst_audio_format_to_string(spec->info.finfo->format));

  //collect a bunch of parameters to shoehorn into a message format
  rosaudiosink->channels = spec->info.channels;  //int number of channels
  rosaudiosink->stride = spec->info.bpf;
  rosaudiosink->endianness = spec->info.finfo->endianness;

  return TRUE;
}

/* undo anything that was done in prepare() */
static gboolean
rosaudiosink_unprepare (GstAudioSink * sink)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "unprepare");

  return TRUE;
}

/* close the device */
static gboolean
rosaudiosink_close (GstAudioSink * sink)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "close");

  return TRUE;
}

/* write samples to the device */
static gint
rosaudiosink_write (GstAudioSink * sink, gpointer data, guint length)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "write");

  //create a message (this loan should be extended upstream)
  auto msg = rosaudiosink->pub->borrow_loaned_message();
  //fill the blanks
  // XXX this is an awful hack and audio needs a better message format
  msg.get().is_bigendian = (rosaudiosink->endianness == G_BIG_ENDIAN);
  msg.get().step = rosaudiosink->stride;
  msg.get().width = rosaudiosink->channels; //image stride matches channel stride
  msg.get().height = length/rosaudiosink->stride; //same total size
  
  //put the data in (ROS expects arrays to be std::vectors)
  msg.get().data = std::vector<uint8_t>((uint8_t*)data, &((uint8_t*)data)[length]);
  //publish
  rosaudiosink->pub->publish(std::move(msg));

  return 0;
}

/* get number of samples queued in the device */
static guint
rosaudiosink_delay (GstAudioSink * sink)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "delay");

  return 0;
}

/* reset the audio device, unblock from a write */
static void
rosaudiosink_reset (GstAudioSink * sink)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "reset");

}

