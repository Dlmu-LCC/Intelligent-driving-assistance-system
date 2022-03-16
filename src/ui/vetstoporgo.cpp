#include "vetstoporgo.h"

VetStopOrGo::VetStopOrGo()
{
    front_car_stop_ = true;   //刚开始假设前车是静止的
    ftime(&last_remind_);
    count_ = 0;
    night_precess=false;


    //本车数据初始化
    int width=frame_width/4;
    int height=frame_height/3;

    //初始化四块检测区域的ROI划分
    left_top_roi=Rect(0,0,width,height);
    left_bottom_roi=Rect(0,height,width,height*2);
    right_top_roi=Rect(frame_width-width,0,width,height);
    right_bottom_roi=Rect(frame_width-width,height,width,height*2);

    //四块检测区域的总像素个数
    l_top_pixel_count=left_top_roi.area();
    r_top_pixel_count=right_top_roi.area();
    l_bottom_pixel_count=left_bottom_roi.area();
    r_bottom_pixel_count=right_bottom_roi.area();

    this_car_motion_state=1;
    last_this_car_motion_state=false;



    //前车数据初始化
    aver_recent_ratio=0;
    center_roi=FRONT_CAR_CENTER_ROI;

}

//图像预处理函数
void VetStopOrGo::pre_process(Mat &image, Mat &pre_image, Mat &dframe){

    //灰度化
    cv::cvtColor(image,image,CV_RGB2GRAY);
    cv::cvtColor(pre_image,pre_image,CV_RGB2GRAY);

    //动态阈值调整
    if(count_ %(10*N_FRAME)==0){
        float lt_hsv;float lb_hsv;float rt_hsv;float rb_hsv;
        compute_horizontal_sum_variance_total(image,lt_hsv,lb_hsv,rt_hsv,rb_hsv);
        THRESHOLD_RATIO_LB=lb_hsv/1500.0f;
        THRESHOLD_RATIO_LT=lt_hsv/1500.0f;
        THRESHOLD_RATIO_RB=rb_hsv/1500.0f;
        THRESHOLD_RATIO_RT=rt_hsv/1500.0f;
    }

    //是否进夜晚图像增强的判断
    if(count_%100==0){
        unsigned char avg_gray;
        compute_avg_gray(image,avg_gray);
        if(avg_gray<THRESHOLD_GRAY_NIGHT_PROCESSING){
            night_precess=true;
        }
        else
            night_precess=false;
    }
    //直方图均衡化+中值滤波处理
    if(night_precess){
        equalizehist_transformation(image);
        equalizehist_transformation(pre_image);
        median_blur(image);
        median_blur(pre_image);
    }


    //帧差图像获取
    cv::absdiff(image,pre_image,dframe);
}


//状态显示和提醒函数
bool VetStopOrGo::remindDriver(Mat& image,STU_SG_DATA& sg_data)
{
    if(sg_data.this_car_state == true)
    {
        return true;
    }

    if(sg_data.this_car_state == false)
    {
        if(sg_data.front_car_state == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}

bool VetStopOrGo::start(Mat& frame,STU_SG_DATA& sg_data)
{
    bool state;
    Mat temp_frame=frame.clone();
    if(count_ % N_FRAME == 0)
    {
        if(pre_frame.empty()){
            pre_frame=frame.clone();
        }
        Mat dframe;
        pre_process(temp_frame,pre_frame,dframe);//图像预处理

        //本车运动状态检测模块输入帧差图像
        sg_data.this_car_state = true;
        getThisCarMotionState(dframe,sg_data.this_car_state);
        //若本车静止，前车运动状态检测模块输入帧差图像
        if(sg_data.this_car_state == false)
        {
            getFrontCarMotionState(dframe,sg_data.front_car_state);
        }
        //本车一旦开始运动，清空前车帧差状态队列
        else
        {
            clear_aver_recent_ratio();
        }
        pre_frame=frame.clone();
    }
    //获取当前帧的SG数据，显示与提醒
    state = remindDriver(frame,sg_data);
    //帧数计数的循环
    if(count_!=65535)
        count_++;
    else
        count_=0;

    if(state){
        return true;
    } else {
        return false;
    }
}


/****************本车处理函数*************/
void VetStopOrGo::getThisCarMotionState(Mat &dframe,bool &state){

    //四块检测区域的ROI划分
    Mat left_top_dframe=dframe(left_top_roi);
    Mat left_bottom_dframe=dframe(left_bottom_roi);
    Mat right_top_dframe=dframe(right_top_roi);
    Mat right_bottom_dframe=dframe(right_bottom_roi);

    int l_top_diff_point_count=0;
    int l_bottom_diff_point_count=0;
    int r_top_diff_point_count=0;
    int r_bottom_diff_point_count=0;

    //计算差分像素的个数
    get_dframe_moving_count(left_top_dframe,l_top_diff_point_count);
    get_dframe_moving_count(left_bottom_dframe,l_bottom_diff_point_count);
    get_dframe_moving_count(right_top_dframe,r_top_diff_point_count);
    get_dframe_moving_count(right_bottom_dframe,r_bottom_diff_point_count);

    bool l_top_is_moved;
    bool r_top_is_moved;
    bool l_bottom_is_moved;
    bool r_bottom_is_moved;

    //差分像素的占比和预定的阈值做比较
    float lt_dpoint_ratio=(float)l_top_diff_point_count/(float)l_top_pixel_count;
    float rt_dpoint_ratio=(float)r_top_diff_point_count/(float)r_top_pixel_count;
    float lb_dpoint_ratio=(float)l_bottom_diff_point_count/(float)l_bottom_pixel_count;
    float rb_dpoint_ratio=(float)r_bottom_diff_point_count/(float)r_bottom_pixel_count;

    if(lt_dpoint_ratio>THRESHOLD_RATIO_LT)
        l_top_is_moved=true;
    else
        l_top_is_moved=false;
    if(rt_dpoint_ratio>THRESHOLD_RATIO_RT)
        r_top_is_moved=true;
    else
        r_top_is_moved=false;
    if(lb_dpoint_ratio>THRESHOLD_RATIO_LB)
        l_bottom_is_moved=true;
    else
        l_bottom_is_moved=false;
    if(rb_dpoint_ratio>THRESHOLD_RATIO_RB)
        r_bottom_is_moved=true;
    else
        r_bottom_is_moved=false;

    //状态机消除闪现
    if((l_top_is_moved||r_top_is_moved)&&(l_bottom_is_moved&&r_bottom_is_moved)){
        if(this_car_motion_state<3)
            this_car_motion_state++;
        else
            this_car_motion_state=3;
    }
    else{
        if(this_car_motion_state>0)
            this_car_motion_state--;
        else
            this_car_motion_state=0;
    }

    if(this_car_motion_state==3)
        state=true;
    if(this_car_motion_state==0)
        state=false;
    if(this_car_motion_state<3&&this_car_motion_state>0)
        state=last_this_car_motion_state;

    last_this_car_motion_state=state;
}

void VetStopOrGo::compute_horizontal_sum_variance_total(Mat &image_gray,
    float &l_top_total_variance,float &l_bottom_total_variance,float &r_top_total_variance,float &r_bottom_total_variance){
    Mat left_top_gray=image_gray(left_top_roi);
    Mat left_bottom_gray=image_gray(left_bottom_roi);
    Mat right_top_gray=image_gray(right_top_roi);
    Mat right_bottom_gray=image_gray(right_bottom_roi);

    l_top_total_variance=(float)compute_horizontal_sum_of_variance(left_top_gray)/(float)(left_top_gray.cols*left_top_gray.rows);
    l_bottom_total_variance=(float)compute_horizontal_sum_of_variance(left_bottom_gray)/(float)(left_bottom_gray.cols*left_bottom_gray.rows);
    r_top_total_variance=(float)compute_horizontal_sum_of_variance(right_top_gray)/(float)(right_top_gray.cols*right_top_gray.rows);
    r_bottom_total_variance=(float)compute_horizontal_sum_of_variance(right_bottom_gray)/(float)(right_bottom_gray.cols*right_bottom_gray.rows);
}



/*********************前车处理函数****************/
void VetStopOrGo::getFrontCarMotionState(cv::Mat &dframe_total, bool &front_car_ismoved){

    Mat dframe=(dframe_total(center_roi)).clone();
    //make_trapezoid(dframe);

    int c_point_diff_count=0;

    get_dframe_moving_count(dframe,c_point_diff_count);

//    cout<<recent_front_car_diff_ratio.size()<<endl;
    imshow("2",dframe);
    int c_pixel_count=dframe.cols*dframe.rows;
    float curr_ratio=(float)c_point_diff_count/(float)c_pixel_count;

//    cout<<"curr_ratio "<<curr_ratio<<endl;
//    cout<<"aver_ratio "<<aver_recent_ratio<<endl;

    //当前前车运动状态的阈值
    if(curr_ratio>THRESHOLD_RARIO_MITIPLE*aver_recent_ratio&&recent_front_car_diff_ratio.size()==DEQUE_MAXSIZE){
        front_car_ismoved=true;
    }
    else
        front_car_ismoved=false;
//    cout<<"front car state:"<<front_car_ismoved<<endl;
    if(!front_car_ismoved){
        if(recent_front_car_diff_ratio.size()<DEQUE_MAXSIZE){
            recent_front_car_diff_ratio.push_front(curr_ratio);
            update_aver_recent_ratio();
        }

        else{
            recent_front_car_diff_ratio.pop_back();
            recent_front_car_diff_ratio.push_front(curr_ratio);
            update_aver_recent_ratio();
        }
    }
}


void VetStopOrGo::make_trapezoid(Mat &center_frame){
    int width=center_frame.cols;
    int height=center_frame.rows;
    //cout<<width<<" "<<height<<endl;
    for(int i=0;i<height;i++){
        for(int j=(height-i)+width/3;j>=0;j--)
            center_frame.at<unsigned char>(i,j)=0;
        for(int k=3*width-(height-i+width/3);k<3*width;k++)
            center_frame.at<unsigned char>(i,k)=0;
    }
    for(int i=0;i<height/5;i++)
        for(int j=0;j<width*3;j++)
            center_frame.at<unsigned char>(i,j)=0;
}

void VetStopOrGo::update_aver_recent_ratio(){
    deque<float>::iterator iter;
    float sum=0;
    for(iter=recent_front_car_diff_ratio.begin();iter!=recent_front_car_diff_ratio.end();iter++)
        sum+=*iter;
    aver_recent_ratio=sum/recent_front_car_diff_ratio.size();
}

void VetStopOrGo::clear_aver_recent_ratio(){
    recent_front_car_diff_ratio.clear();
}


/*************工具函数****************************/
//直方图均衡化
void VetStopOrGo::equalizehist_transformation(Mat &image_gray){
    equalizeHist(image_gray,image_gray);
}

//中值滤波
void VetStopOrGo::median_blur(Mat& image){
    medianBlur(image,image,9);
}

//双边滤波
void VetStopOrGo::bilateral_blur(Mat& image){
    Mat image2;
    cv::bilateralFilter(image,image2, 25, 25*2, 25/2);
    image=image2.clone();
}

//计算平均灰度值
void VetStopOrGo::compute_avg_gray(Mat image_gray, unsigned char &avg_of_gray){
    int rows=image_gray.rows;
    int cols=image_gray.cols;
    unsigned long long total_gray=0;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            total_gray+=image_gray.at<unsigned char>(i,j);
        }
    }
    avg_of_gray=(unsigned char)((float)total_gray/(float)(rows*cols));
}

//计算差分图像的差分像素
void VetStopOrGo::get_dframe_moving_count(Mat dframe, int &count){
    for(int i=0;i<dframe.rows;i++)
        for(int j=0;j<dframe.cols;j++)
            if(dframe.at<unsigned char>(i,j)<=THRESHOLD_GRAY)
                dframe.at<unsigned char>(i,j)=0;
            else{
                count++;
                dframe.at<unsigned char>(i,j)=255;
            }
}

//计算ROI内水平方向像素差异性总和
long VetStopOrGo::compute_horizontal_sum_of_variance(Mat &image_gray){
    int rows=image_gray.rows;
    int cols=image_gray.cols;
    long sum_of_variance=0;
    for(int i=0;i<rows;i++){
        unsigned char aver_pix;
        long total=0;
        for(int j=0;j<cols;j++){
            total+=(long)(image_gray.at<unsigned char>(i,j));
        }
        aver_pix=(unsigned char)((float)total/(float)cols);
        for(int j=0;j<cols;j++){
            sum_of_variance+=(long)abs(aver_pix-image_gray.at<unsigned char>(i,j));
        }
    }
    return sum_of_variance;
}
