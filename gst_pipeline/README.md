# gst_pipeline
This package provides a ROS node that hosts a gstreamer pipeline inside a ROS composable node.
This is similar to the gscam2 package, but here we use the gst-bridge package for shared-memory IO,
allowing this package to focus on supporting features we attach to the pipeline itself.


Features are added using pluginlib; you have access to the node interfaces, and the pipeline,
and you don't have to fork the node to do it.

Look at the gst_pipeline_plugins package for examples that enable shared-memory out of gst-bridge elements, pause and seek the pipeline, drop frames until services are called, and show traffic in the pipeline


Many thanks to Clyde McQueen for his work on gscam2
