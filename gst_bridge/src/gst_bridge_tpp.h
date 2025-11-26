#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER gst_bridge

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "./gst_bridge_tpp.h"

#if !defined(_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  gst_sink_render,
  TP_ARGS(
    const void *, ros_base_sink_handle_arg,
    int64_t, source_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, ros_base_sink_handle, ros_base_sink_handle_arg)
    ctf_integer(int64_t, source_timestamp, source_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  gst_sink_open,
  TP_ARGS(
    const void *, node_handle_arg,
    const void *, ros_base_sink_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, ros_base_sink_handle, ros_base_sink_handle_arg)
  )
)

#endif /* _TP_H */

#include <lttng/tracepoint-event.h>

