#include "read_camera.h"
namespace read_camera_xk {
using namespace std;
using namespace cv;

readCameraFast::readCameraFast():
nh_param("~")
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  if(!nh_param.getParam("advertise_topic_name",advertise_topic_name))  advertise_topic_name = "/back_camera/image_raw";
  if(!nh_param.getParam("video_location",video_location))  video_location = "/dev/video0";
  if(!nh_param.getParam("show_video",show_video)) show_video = false;
  if(!nh_param.getParam("show_fps",show_fps)) show_fps = false;
  if(!nh_param.getParam("ifUndistortImage",`))ifUndistortImage=false;
  if(!nh_param.getParam("loop_rate",loop_rate)) loop_rate = 60;
  if(!nh_param.getParam("video_width",video_width)) video_width = 640;
  if(!nh_param.getParam("video_height",video_height)) video_height = 480;
  
  if(!nh_param.getParam("cameraYML", cameraPara.cameraYML))cameraPara.cameraYML="/home/nvidia/Desktop/IARC2018_NO2TX2/src/fisheye/data/fisheye1.yml";
  if(!nh_param.getParam("focal_multiple", cameraPara.focalMultiple))cameraPara.focalMultiple=1;
  if(!nh_param.getParam("central_point_deviation_u", cameraPara.centralPointDeviation_u))cameraPara.centralPointDeviation_u=0;
  if(!nh_param.getParam("central_point_deviation_v", cameraPara.centralPointDeviation_v))cameraPara.centralPointDeviation_v=0;
  cameraPara.image_height = video_height;
  cameraPara.image_width = video_width;
  
  pub = it.advertise(advertise_topic_name, 1);
  undistort.initialize(cameraPara);
  initCamera();
}

void readCameraFast::run()
{
  ros::Rate loop_rate_class(loop_rate);
  while (ros::ok()) {
    if(show_fps){t = (double)cvGetTickCount();}
    
    //get frame-----------------------------------------
    vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
    cvmat = cvMat(IMAGEHEIGHT,IMAGEWIDTH,CV_8UC3,(void*)yuv422frame);
    img = cvDecodeImage(&cvmat,1); //need release
    if(!img){
      printf("DecodeImage error!\n");
    }
    frame  = cvarrToMat(img);
    if(frame.empty()){
      std::cout<<"empty"<<std::endl;
      continue;
    }
    //undistortImag-----------------------------------------
    if(ifUndistortImage){
      cv::Mat undistortImag;
      undistort.getUndistortImage(frame,undistortImag);
      frame = undistortImag;
    }
    if(show_video){
      cv::namedWindow("sourceVideo",CV_WINDOW_NORMAL);
      cv::imshow("sourceVideo",frame);
      cv::waitKey(1);
    }
    
    //pub frame-----------------------------------------
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    cv::waitKey(1);
    cvReleaseImage( &img );//&img need released
    vcap->backFrame();
    if((cvWaitKey(1)&255) == 27){
	    exit(0);
    }
    
    if(show_fps){
      t = (double)cvGetTickCount() - t;
      t_sum += t/(cvGetTickFrequency()*1000)/1000;
      t_count++;
      double fps = 1/(t_sum/t_count);
      std::cout<<"fps: "<<fps<<std::endl;
    }

    ros::spinOnce();
    loop_rate_class.sleep();
  }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_camera_node");
  read_camera_xk::readCameraFast Re;
  Re.run();
  return 0;
}
