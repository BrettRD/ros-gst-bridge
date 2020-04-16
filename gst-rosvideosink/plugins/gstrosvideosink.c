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
 * SECTION:element-gstrosvideosink
 *
 * The rosvideosink element does FIXME stuff.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v fakesrc ! rosvideosink ! FIXME ! fakesink
 * ]|
 * FIXME Describe what the pipeline does.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideosink.h>
#include "gstrosvideosink.h"

GST_DEBUG_CATEGORY_STATIC (gst_rosvideosink_debug_category);
#define GST_CAT_DEFAULT gst_rosvideosink_debug_category

/* prototypes */


static void gst_rosvideosink_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_rosvideosink_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void gst_rosvideosink_dispose (GObject * object);
static void gst_rosvideosink_finalize (GObject * object);

static GstFlowReturn gst_rosvideosink_show_frame (GstVideoSink * video_sink,
    GstBuffer * buf);

enum
{
  PROP_0
};

/* pad templates */

/* FIXME: add/remove formats you can handle */
#define VIDEO_SINK_CAPS \
    GST_VIDEO_CAPS_MAKE("{ I420, Y444, Y42B, UYVY, RGBA }")


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstRosvideosink, gst_rosvideosink, GST_TYPE_VIDEO_SINK,
    GST_DEBUG_CATEGORY_INIT (gst_rosvideosink_debug_category, "rosvideosink", 0,
        "debug category for rosvideosink element"));

static void
gst_rosvideosink_class_init (GstRosvideosinkClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstVideoSinkClass *video_sink_class = GST_VIDEO_SINK_CLASS (klass);

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_pad_template_new ("sink", GST_PAD_SINK, GST_PAD_ALWAYS,
          gst_caps_from_string (VIDEO_SINK_CAPS)));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "FIXME Long name", "Generic", "FIXME Description",
      "FIXME <fixme@example.com>");

  gobject_class->set_property = gst_rosvideosink_set_property;
  gobject_class->get_property = gst_rosvideosink_get_property;
  gobject_class->dispose = gst_rosvideosink_dispose;
  gobject_class->finalize = gst_rosvideosink_finalize;
  video_sink_class->show_frame =
      GST_DEBUG_FUNCPTR (gst_rosvideosink_show_frame);

}

static void
gst_rosvideosink_init (GstRosvideosink * rosvideosink)
{
}

void
gst_rosvideosink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstRosvideosink *rosvideosink = GST_ROSVIDEOSINK (object);

  GST_DEBUG_OBJECT (rosvideosink, "set_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rosvideosink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstRosvideosink *rosvideosink = GST_ROSVIDEOSINK (object);

  GST_DEBUG_OBJECT (rosvideosink, "get_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rosvideosink_dispose (GObject * object)
{
  GstRosvideosink *rosvideosink = GST_ROSVIDEOSINK (object);

  GST_DEBUG_OBJECT (rosvideosink, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (gst_rosvideosink_parent_class)->dispose (object);
}

void
gst_rosvideosink_finalize (GObject * object)
{
  GstRosvideosink *rosvideosink = GST_ROSVIDEOSINK (object);

  GST_DEBUG_OBJECT (rosvideosink, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (gst_rosvideosink_parent_class)->finalize (object);
}

static GstFlowReturn
gst_rosvideosink_show_frame (GstVideoSink * sink, GstBuffer * buf)
{
  GstRosvideosink *rosvideosink = GST_ROSVIDEOSINK (sink);

  GST_DEBUG_OBJECT (rosvideosink, "show_frame");

  return GST_FLOW_OK;
}

