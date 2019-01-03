#include <iostream>

using namespace std;
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include<string>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#define WINDOW_NAME "CV GUI"
void ui_init(Mat &frame);
void ui_chooseScreenPoint(Mat &frame);
void ui_getScreenPoint();
//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//深度图对齐到彩色图函数
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
//    uint16_t depth_max=0;
//    for(int row=0;row<depth.rows;row++){
//        for(int col=0;col<depth.cols;col++){
//            if(depth_max<depth.at<uint16_t>(row,col))
//                depth_max=depth.at<uint16_t>(row,col);
//        }
//    }
    int y=0,x=0;
    //初始化结果
    //Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
    Mat result=Mat(color.rows,color.cols,CV_16U,Scalar(0));
    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
//            if(x<0||x>color.cols)
//                continue;
//            if(y<0||y>color.rows)
//                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

            //将成功映射的点用彩色图对应点的RGB数据覆盖
//            for(int k=0;k<3;k++){
//                //这里设置了只显示1米距离内的东西
//                if(depth_m<1)
//                result.at<cv::Vec3b>(y,x)[k]=
//                        color.at<cv::Vec3b>(y,x)[k];
//            }

            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    //返回一个与彩色图对齐了的深度信息图像
    return result;
}

void measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile)
{
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    //定义图像中心点
    cv::Point center(color.cols/2,color.rows/2);
    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //遍历该范围
    float distance_sum=0;
    int effective_pixel=0;
    for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
        for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth.at<uint16_t>(y,x)){
                distance_sum+=depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }
    cout<<"遍历完成，有效像素点:"<<effective_pixel<<endl;
    float effective_distance=distance_sum/effective_pixel;
    cout<<"目标距离："<<effective_distance<<" m"<<endl;
    char distance_str[30];
    sprintf(distance_str,"the distance is:%f m",effective_distance);
    cv::rectangle(color,RectRange,Scalar(0,0,255),2,8);
    cv::putText(color,(string)distance_str,cv::Point(color.cols*0.02,color.rows*0.05),
                cv::FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),2,8);
}

/******************全局变量*************************/
cv::Point camaera_start_point=cv::Point(320,10);
int setting_ScreenPoint_flag=0;
struct ScreenOnFrame{
    cv::Point left_up=cv::Point(0,0);
    cv::Point right_up=cv::Point(0,0);
    cv::Point left_down=cv::Point(0,0);
    cv::Point right_down=cv::Point(0,0);
}screenOnframe;

int main()
{
    //背景图层
    cv::Mat frame = cv::Mat(500, 960, CV_8UC3);
    cvui::init(WINDOW_NAME);

//    const char* depth_win="depth_Image";
//    namedWindow(depth_win,WINDOW_AUTOSIZE);
//    const char* color_win="color_Image";
//    namedWindow(color_win,WINDOW_AUTOSIZE);

    //深度图像颜色map
    rs2::colorizer c;                          // Helper to colorize depth images

    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);

    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);


    while (cvGetWindowHandle(WINDOW_NAME)) // Application still alive?
    {
        frame = cv::Scalar(49, 52, 49);
        ui_init(frame);
        ui_chooseScreenPoint(frame);
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=color_frame.as<rs2::video_frame>().get_width();
        const int color_h=color_frame.as<rs2::video_frame>().get_height();

        //创建OPENCV类型 并传入数据
        Mat depth_image(Size(depth_w,depth_h),
                                CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        Mat depth_image_4_show(Size(depth_w,depth_h),
                                CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        Mat color_image(Size(color_w,color_h),
                                CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);
        //实现深度图对齐到彩色图
        Mat result=align_Depth2Color(depth_image,color_image,profile);
        //measure_distance(color_image,result,cv::Size(20,20),profile);
        //显示
        //imshow(depth_win,depth_image_4_show);
        //显示图片
        cvui::image(frame,camaera_start_point.x,camaera_start_point.y,color_image);
        imshow(WINDOW_NAME,frame);
        //imshow("result",result);
        waitKey(1);
    }
    return 0;
}

void ui_init(Mat &frame)
{
    cvui::text(frame, 20, 10, "Hello, Opencv");
    cvui::window(frame, 20, 40, 280, 250, "Screen Point");
    cv::Point current_mouse = cvui::mouse();

    int current_mouse_x=current_mouse.x-camaera_start_point.x<0 ? 0:current_mouse.x-camaera_start_point.x;
    int current_mouse_y=current_mouse.y-camaera_start_point.y<0 ? 0:current_mouse.y-camaera_start_point.y;
    //sprintf(current_mouse_str_x,"%d",current_mouse_x);
    //sprintf(current_mouse_str_y,"%d",current_mouse_y);
    cvui::printf(frame, 120, 45,0.4,0x00ff00, "current mouse=(%d,%d)", current_mouse_x,current_mouse_y);

    //显示4个角的坐标
    cvui::printf(frame, 200, 105,0.4,0x00ff00, "(%d,%d)", screenOnframe.left_up.x,screenOnframe.left_up.y);
    cvui::printf(frame, 200, 145,0.4,0x00ff00, "(%d,%d)", screenOnframe.right_up.x,screenOnframe.right_up.y);
    cvui::printf(frame, 200, 185,0.4,0x00ff00, "(%d,%d)", screenOnframe.left_down.x,screenOnframe.left_down.y);
    cvui::printf(frame, 200, 225,0.4,0x00ff00, "(%d,%d)", screenOnframe.right_down.x,screenOnframe.right_down.y);
    //4个按钮
    if(cvui::button(frame, 30, 100, "Left-Up Point:")){
        setting_ScreenPoint_flag=1;
    }
    if(cvui::button(frame, 30, 140, "Right-Up Point:")){
        setting_ScreenPoint_flag=2;
    }
    if(cvui::button(frame, 30, 180, "Left-Down Point:")){
        setting_ScreenPoint_flag=3;
    }
    if(cvui::button(frame, 30, 220, "Right-Down Point:")){
        setting_ScreenPoint_flag=4;
    }
    cvui::update();
}

void ui_chooseScreenPoint(Mat &frame)
{
    //判断鼠标状态
    int status=cvui::iarea(camaera_start_point.x, camaera_start_point.y, 640, 480);
    //cvui::rect(camaera_start_point.x-1, camaera_start_point.y-1, 640+1, 480+1);
    switch (status) {
      case cvui::CLICK:  std::cout << "Clicked!" << std::endl; break;
      case cvui::DOWN:   ui_getScreenPoint();std::cout << "mouse down!" << std::endl; break;
      case cvui::OVER:   break;
      case cvui::OUT:    break;
    }
}

void ui_getScreenPoint()
{
    //鼠标按下后执行
    cv::Point current_mouse = cvui::mouse();
    int current_mouse_x=current_mouse.x-camaera_start_point.x<0 ? 0:current_mouse.x-camaera_start_point.x;
    int current_mouse_y=current_mouse.y-camaera_start_point.y<0 ? 0:current_mouse.y-camaera_start_point.y;
    cv::Point point_in_frame(current_mouse_x,current_mouse_y);
    //根据当前的flag 记录鼠标所在点
    switch (setting_ScreenPoint_flag) {
    case 1:screenOnframe.left_up=point_in_frame;break;
    case 2:screenOnframe.right_up=point_in_frame;break;
    case 3:screenOnframe.left_down=point_in_frame;break;
    case 4:screenOnframe.right_down=point_in_frame;break;
    default:break;
    }
}
