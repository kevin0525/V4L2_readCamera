#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
class FisheyeModel
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param;
    FisheyeModel(ros::NodeHandle nh);
    
    ros::Subscriber sub_img;
    ros::Publisher pub_undistortionImg;
    
    void initialize();
    void undistortImage(cv::Mat & distorted, cv::Mat & undistorted);
    void FishEyeImgUndistort(cv::Mat &src_img, cv::Mat &UndistortImg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
    cv::Mat intrinsicMatrix, distortionCoeffs;
    string cameraYML;
    double focalMultiple, centralPointDeviation_u, centralPointDeviation_v;  
    cv::Mat map1, map2;
    cv::Mat new_intrinsic_mat, R;
    cv::cuda::GpuMat GPU_src_img, GPU_undistor_img, GPU_map1, GPU_map2;
};
FisheyeModel::FisheyeModel(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
  
    if(!ros::param::get("~cameraYML", cameraYML))cameraYML="/home/nvidia/Desktop/IARC2018_NO2TX2/src/fisheye/data/fisheye1.yml";
    std::cout << "cameraYML is " << cameraYML << std::endl;
  
    cv::FileStorage fs(cameraYML, cv::FileStorage::READ);
    fs["intrinsicMatrix"] >> intrinsicMatrix;
    fs["distortionCoeffs"] >> distortionCoeffs;
    
    if(!ros::param::get("~focal_multiple", focalMultiple))focalMultiple=1;
    printf("focalMultiple is %f\n", focalMultiple);
    if(!ros::param::get("~central_point_deviation_u", centralPointDeviation_u))centralPointDeviation_u=0;
    printf("centralPointDeviation_u is %f\n", centralPointDeviation_u);
    if(!ros::param::get("~central_point_deviation_v", centralPointDeviation_v))centralPointDeviation_v=0;
    printf("centralPointDeviation_v is %f\n", centralPointDeviation_v);
    
    R = cv::Mat::eye(3,3,CV_32F);
   // std::cout << focalMultiple << std::endl;
    initialize();
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    } 
}

void FisheyeModel::initialize()
{
        
    intrinsicMatrix.copyTo(new_intrinsic_mat);
    
    new_intrinsic_mat.at<float>(0, 0) *= focalMultiple;
    new_intrinsic_mat.at<float>(1, 1) *= focalMultiple;
   // std::cout << focalMultiple << std::endl;
    
    new_intrinsic_mat.at<float>(0, 2) += centralPointDeviation_u;
    new_intrinsic_mat.at<float>(1, 2) += centralPointDeviation_v;
    
    cv::Size size = cv::Size(640,480);
    sub_img = nh_.subscribe("/back_camera/image_raw",1,&FisheyeModel::imageCallback,this); 
    pub_undistortionImg = nh_.advertise<sensor_msgs::Image>("/fisheye/image", 1);
    //pub_fisheyeIntrinsic = nh_.advertise<std_msgs::Float32MultiArray>("/fisheye/intrinsic", 1);
    cv::initUndistortRectifyMap(intrinsicMatrix, distortionCoeffs, R, new_intrinsic_mat, size,  CV_32FC1, map1, map2);
    GPU_map1.upload(map1);
    GPU_map2.upload(map2);
}

//void FisheyeModel::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
void FisheyeModel::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout<<"hahah"<<std::endl;
    cv::Mat src_img, undistort_img;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(src_img);
    
    FishEyeImgUndistort(src_img, undistort_img);
}

void FisheyeModel::FishEyeImgUndistort(cv::Mat &src_img, cv::Mat &UndistortImg)
{
   std::cout << distortionCoeffs.at<float>(0, 0) << std::endl;
    sensor_msgs::ImagePtr image_msg;
    undistortImage(
	src_img,
	UndistortImg);
    
    image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", UndistortImg).toImageMsg();
    pub_undistortionImg.publish(image_msg);
}

void FisheyeModel::undistortImage(cv::Mat & distorted, cv::Mat & undistorted)
{
    cv::Mat src_img_4, distor_img_4;
    cv::cvtColor(distorted,src_img_4,CV_BGR2BGRA);//gpu
    GPU_src_img.upload(src_img_4);//gpu
    
    cv::cuda::remap(GPU_src_img, GPU_undistor_img,GPU_map1, GPU_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, Scalar(), cv::cuda::Stream::Null() );
    GPU_undistor_img.download(distor_img_4);
    cv::cvtColor(distor_img_4,undistorted,CV_BGRA2BGR);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "undistortimage_xk_node");
    ros::NodeHandle nh;
    FisheyeModel fisheye(nh);
    ros::spin();
    return 0;
}
