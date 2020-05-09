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
 * The rosaudiosink element does FIXME stuff.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v fakesrc ! rosaudiosink ! FIXME ! fakesink
 * ]|
 * FIXME Describe what the pipeline does.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/audio/gstaudiosink.h>
#include "gstrosaudiosink.h"

GST_DEBUG_CATEGORY_STATIC (gst_rosaudiosink_debug_category);
#define GST_CAT_DEFAULT gst_rosaudiosink_debug_category

/* prototypes */


static void gst_rosaudiosink_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_rosaudiosink_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void gst_rosaudiosink_dispose (GObject * object);
static void gst_rosaudiosink_finalize (GObject * object);

static gboolean gst_rosaudiosink_open (GstAudioSink * sink);
static gboolean gst_rosaudiosink_prepare (GstAudioSink * sink,
    GstAudioRingBufferSpec * spec);
static gboolean gst_rosaudiosink_unprepare (GstAudioSink * sink);
static gboolean gst_rosaudiosink_close (GstAudioSink * sink);
static gint gst_rosaudiosink_write (GstAudioSink * sink, gpointer data,
    guint length);
static guint gst_rosaudiosink_delay (GstAudioSink * sink);
static void gst_rosaudiosink_reset (GstAudioSink * sink);

enum
{
  PROP_0
};

/* pad templates */

/* FIXME add/remove the formats that you want to support */
static GstStaticPadTemplate gst_rosaudiosink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw,format=S16LE,rate=[1,max],"
        "channels=[1,max],layout=interleaved")
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstRosaudiosink, gst_rosaudiosink, GST_TYPE_AUDIO_SINK,
    GST_DEBUG_CATEGORY_INIT (gst_rosaudiosink_debug_category, "rosaudiosink", 0,
        "debug category for rosaudiosink element"));

static void
gst_rosaudiosink_class_init (GstRosaudiosinkClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstAudioSinkClass *audio_sink_class = GST_AUDIO_SINK_CLASS (klass);

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (GST_ELEMENT_CLASS (klass),
      &gst_rosaudiosink_sink_template);

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "FIXME Long name", "Generic", "FIXME Description",
      "FIXME <fixme@example.com>");

  gobject_class->set_property = gst_rosaudiosink_set_property;
  gobject_class->get_property = gst_rosaudiosink_get_property;
  gobject_class->dispose = gst_rosaudiosink_dispose;
  gobject_class->finalize = gst_rosaudiosink_finalize;
  audio_sink_class->open = GST_DEBUG_FUNCPTR (gst_rosaudiosink_open);
  audio_sink_class->prepare = GST_DEBUG_FUNCPTR (gst_rosaudiosink_prepare);
  audio_sink_class->unprepare = GST_DEBUG_FUNCPTR (gst_rosaudiosink_unprepare);
  audio_sink_class->close = GST_DEBUG_FUNCPTR (gst_rosaudiosink_close);
  audio_sink_class->write = GST_DEBUG_FUNCPTR (gst_rosaudiosink_write);
  audio_sink_class->delay = GST_DEBUG_FUNCPTR (gst_rosaudiosink_delay);
  audio_sink_class->reset = GST_DEBUG_FUNCPTR (gst_rosaudiosink_reset);

}

static void
gst_rosaudiosink_init (GstRosaudiosink * rosaudiosink)
{
}

void
gst_rosaudiosink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "set_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rosaudiosink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "get_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rosaudiosink_dispose (GObject * object)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (gst_rosaudiosink_parent_class)->dispose (object);
}

void
gst_rosaudiosink_finalize (GObject * object)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (gst_rosaudiosink_parent_class)->finalize (object);
}

/* open the device with given specs */
static gboolean
gst_rosaudiosink_open (GstAudioSink * sink)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "open");

  return TRUE;
}

/* prepare resources and state to operate with the given specs */
static gboolean
gst_rosaudiosink_prepare (GstAudioSink * sink, GstAudioRingBufferSpec * spec)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "prepare");

  return TRUE;
}

/* undo anything that was done in prepare() */
static gboolean
gst_rosaudiosink_unprepare (GstAudioSink * sink)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "unprepare");

  return TRUE;
}

/* close the device */
static gboolean
gst_rosaudiosink_close (GstAudioSink * sink)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "close");

  return TRUE;
}

/* write samples to the device */
static gint
gst_rosaudiosink_write (GstAudioSink * sink, gpointer data, guint length)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "write");

  return 0;
}

/* get number of samples queued in the device */
static guint
gst_rosaudiosink_delay (GstAudioSink * sink)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "delay");

  return 0;
}

/* reset the audio device, unblock from a write */
static void
gst_rosaudiosink_reset (GstAudioSink * sink)
{
  GstRosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "reset");

}

