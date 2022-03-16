#ifndef VETSTOPORGO_H
#define VETSTOPORGO_H
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <sys/timeb.h>

using namespace std;
using namespace cv;



class VetStopOrGo
{
    public:

        //SG模块数据结构    本车运动状态、前车运动状态、是否发出起步提醒、
        struct STU_SG_DATA
        {
            bool this_car_state;
            bool front_car_state;
            bool go;
        };


        cv::Rect FRONT_CAR_CENTER_ROI=Rect(420,400,400,300);//前车启停状态检测的ROI设定

        int THRESHOLD_GRAY=20;//灰度二值化处理阈值

        float THRESHOLD_RATIO_LT=0.01;//本车运动差分像素的占比阈值
        float THRESHOLD_RATIO_RT=0.01;
        float THRESHOLD_RATIO_LB=0.01;
        float THRESHOLD_RATIO_RB=0.01;

        int DEQUE_MAXSIZE=20;//前车近期差分像素占比队列的MAXSIZE
        float THRESHOLD_RARIO_MITIPLE=5;//判断前车ROI差分像素占比的倍数阈值

        int N_FRAME=4;//每隔N_FRAME帧抽取一帧作为SG功能模块的输入

        unsigned char THRESHOLD_GRAY_NIGHT_PROCESSING=60;//开启夜晚图像增强模式的灰度阈值

        int frame_width=1280;
        int frame_height=720;

        STU_SG_DATA car_data;


    public:
        VetStopOrGo();
        bool remindDriver(Mat& frame,STU_SG_DATA& sg_data);
        bool start(Mat& frame,STU_SG_DATA& sg_data);
        void pre_process(cv::Mat& image,cv::Mat& pre_image,cv::Mat& dframe);

        //前车所需
        void getThisCarMotionState(cv::Mat &dframe,bool &state);
        void compute_horizontal_sum_variance_total(cv::Mat &image_gray,
            float &l_top_total_variance,float &l_bottom_total_variance,float &r_top_total_variance,float &r_bottom_total_variance);

        cv::Rect left_top_roi;
        cv::Rect left_bottom_roi;
        cv::Rect right_top_roi;
        cv::Rect right_bottom_roi;

        //后车所需
        void getFrontCarMotionState(cv::Mat &dframe_total,bool &front_car_state);
        void clear_aver_recent_ratio();

        cv::Rect center_roi;


    private:
        //工具
        void equalizehist_transformation(cv::Mat& image);
        void median_blur(cv::Mat& image);
        void bilateral_blur(cv::Mat& image);
        void get_dframe_moving_count(cv::Mat dframe,int& count);
        void compute_avg_gray(cv::Mat image,unsigned char& avg_of_gray);
        long compute_horizontal_sum_of_variance(cv::Mat &image_gray);

    private:
        cv::Mat pre_frame;
        bool front_car_stop_;
        timeb time;
        timeb last_remind_;
        int count_;
        bool night_precess;

        //本车所需变量
        int l_top_pixel_count;
        int l_bottom_pixel_count;
        int r_top_pixel_count;
        int r_bottom_pixel_count;

        int this_car_motion_state;
        bool last_this_car_motion_state;

        //前车所需变量
        std::deque<float> recent_front_car_diff_ratio;
        float aver_recent_ratio;

        void make_trapezoid(cv::Mat& center_frame);
        void update_aver_recent_ratio();

};
#endif // VETSTOPORGO_H
