extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <X11/Xlib.h>
#include "v4l2uvc.h"
#include "avilib.h"
}

#include "opencv2/opencv.hpp"

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])   // -s 1280x720   -d  /dev/video1   -a  test.avi  -i 30
{
  FILE *file = NULL;
  struct vdIn *vd;
  unsigned char *tmpbuffer;
  char *filename = NULL;
  char *avifilename = NULL;

  char *sizestring = NULL;
  char *fpsstring  = NULL;
  char *separateur = NULL;
  int width ; 
  int height;
  int fps;
  int i;

  width = 640 ; 
  height = 480 ;
  fps   = 120 ;
  filename = "/dev/video0";
  avifilename = "test.avi";

  int format = V4L2_PIX_FMT_MJPEG;
  int ret;
  int grabmethod = 1;
	  
  file = fopen(filename, "wb");
  if(file == NULL) 
  {
    printf("Unable to open file for raw frame capturing\n ");		
    exit(1);
  }

  //v4l2 init
  vd = (struct vdIn *) calloc(1, sizeof(struct vdIn));
  if(init_videoIn(vd, (char *) filename, width, height,fps,format,grabmethod,avifilename) < 0)
  {
    exit(1);
  }

  if (video_enable(vd))
  {
    exit(1);
  }

  vd->avifile = AVI_open_output_file(vd->avifilename);
  if (vd->avifile == NULL ) 
  {
    printf("Error opening avifile %s\n",vd->avifilename);
    exit(1);
  }

  AVI_set_video(vd->avifile, vd->width, vd->height, fps, "MJPG");
  printf("recording to %s\n",vd->avifilename);

  while(1)
  {		
    memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
    vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd->buf.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(vd->fd, VIDIOC_DQBUF, &vd->buf);
    if (ret < 0) 
    {
      printf("Unable to dequeue buffer");
      exit(1);
    }
    
    memcpy(vd->tmpbuffer, vd->mem[vd->buf.index],vd->buf.bytesused);
    //CvMat cvmat = cvMat(480,640,CV_8UC3,vd->tmpbuffer);
    
    
    printf("%lu\n", sizeof(vd->tmpbuffer));
//     printf("%s\n", *vd->tmpbuffer);
    //printf("%u\n", vd->buf.bytesused);
    AVI_write_frame(vd->avifile, (char*)vd->tmpbuffer,vd->buf.bytesused, vd->framecount);

    vd->framecount++;
    
    ret = ioctl(vd->fd, VIDIOC_QBUF, &vd->buf);
    if (ret < 0) 
    {
      printf("Unable to requeue buffer");
      exit(1);
    }
  }
  fclose(file);
  close_v4l2(vd);
}
