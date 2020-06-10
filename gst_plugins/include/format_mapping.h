#ifndef _GST_FORMAT_MAPPING_H_
#define _GST_FORMAT_MAPPING_H_


get_cv_from_gst(GstAudioInfo gst_format)
{
  const char* number_format_string = gst_format.finfo.name;
  const char* number_format_description = gst_format.finfo.description;
  int bit depth = gst_format.finfo.depth;   //amount of valid bits in width
  int sample_rate = gst_format.rate;  //sample rate in Hz


  int bit_width = gst_format.finfo.width;   //amount of bits used for one sample
  int channels = gst_format.channels;     //number of channels

  
  uint8_t endianness = gst_format.finfo.endianness
  size_t frame_size = gst_format.bpf;     //bytes per frame

  char num_stub = 'X'
  if(gst_format.finfo.flags & GST_AUDIO_FORMAT_FLAG_INTEGER)
  {
    if(gst_format.finfo.flags & GST_AUDIO_FORMAT_FLAG_SIGNED)
    {
      num_stub = "S"
    }
    else
    {
      num_stub = 'U'
    }
  }
  else if(gst_format.finfo.flags & GST_AUDIO_FORMAT_FLAG_FLOAT)
  {
    num_stub = 'F'
  }

  ("%d%cC%d", bit_width, num_stub, channels)



}




#endif //_GST_FORMAT_MAPPING_H_