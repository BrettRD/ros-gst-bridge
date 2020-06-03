#!/usr/bin/python

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
import std_msgs

from pipeline import Pipeline
from simplebin import Simplebin

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

def run_update(updater):
  updater.update()
  return True

if __name__ == '__main__':
  GObject.threads_init()

  pipe_node = Pipeline()
  webrtc_bin = Simplebin("audiotestsrc is-live=true do-timestamp=true  ! queue ! fakesink", "fakeseg")
  pipe_node.add_section(webrtc_bin)
  pipe_node.start_pipeline()


  GLib.timeout_add(100, run_update, pipe_node.updater)
  loop = GLib.MainLoop()
  loop.run()

  #rospy.spin()
  #while not rospy.is_shutdown():
