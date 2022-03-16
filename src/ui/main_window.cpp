#include "main_window.h"
#include "ui_main_window.h"
#include "configs/config.h"
#include "source.h"

using namespace std;
using namespace cv;


MainWindow::MainWindow(QWidget *parent, bool is_simulation_mode)
    : QMainWindow(parent), ui(new Ui::MainWindow), is_simulation_mode(is_simulation_mode) {
    ui->setupUi(this);


    ui->graphicsView->setScene(new QGraphicsScene(this));
    ui->graphicsView->scene()->addItem(&pixmap);

    warning_icon = cv::imread("images/collision-warning.png");
    cv::resize(warning_icon, warning_icon, cv::Size(48, 48));

    lane_departure_warning_icon = cv::imread("images/lane-departure-warning.png");
    cv::resize(lane_departure_warning_icon, lane_departure_warning_icon, cv::Size(48, 48));

    // 连接按钮和槽
    connect(ui->simulationBtn, SIGNAL(released()), this, SLOT(openSimulationSelector()));
    connect(ui->muteBtn, SIGNAL(released()), this, SLOT(toggleMute()));
    connect(ui->alertBtn, SIGNAL(released()), this, SLOT(toggleAlert()));
    connect(ui->setupBtn, SIGNAL(released()), this, SLOT(showCameraWizard()));

    car_status = std::make_shared<CarStatus>();
    camera_model = std::make_shared<CameraModel>();
    object_detector = std::make_shared<ObjectDetector>();

    // 相机矫正初始化
    this->camera_wizard = std::make_shared<CameraWizard>(this->car_status);
    this->camera_wizard->setStyleSheet("QAbstractButton { height: 50px }");
    // 避免在第一次进入模拟时发生大小错误
    this->camera_wizard->showFullScreen();
    this->camera_wizard->hide();

    connect(this->camera_wizard.get(), SIGNAL(updateCameraModel(float, float, float, float, float, float, float, float, float, float, float, float)), this, SLOT(updateCameraModel(float, float, float, float, float, float, float, float, float, float, float, float)));



    #ifndef DISABLE_LANE_DETECTOR
    lane_detector = std::make_shared<LaneDetector>();
    #endif

    #ifndef DISABLE_GPS_READER
    car_gps_reader = std::make_shared<CarGPSReader>();
    #endif

    if (USE_CAN_BUS_FOR_SIMULATION_DATA) {
        can_reader = std::make_shared<CANReader>();
    }

    collision_warning = std::make_shared<CollisionWarningController>(camera_model, car_status);

    //创建输入源选择窗口
    Source *mode = new Source();
    connect(mode,SIGNAL(SendModeAndData(bool,bool)),this,SLOT(ReceiveModeAndData(bool,bool)));
    this->showFullScreen();
    mode->showFullScreen();

    //vet_stop_or_go = std::make_shared<VetStopOrGo>();

    /*std::thread sg_thread(&MainWindow::SGThread,
        vet_stop_or_go,
        car_status,
        this);
    sg_thread.detach();*/

    // 打开对象识别主线程
    std::thread od_thread(&MainWindow::objectDetectionThread, 
        object_detector,
        car_status,
        collision_warning.get()
        );
    od_thread.detach();

#ifndef DISABLE_LANE_DETECTOR
    std::thread ld_thread(&MainWindow::laneDetectionThread, 
        lane_detector,
        car_status,
        this);
    ld_thread.detach();
#endif


    std::thread cpr_thread(&MainWindow::carPropReaderThread,
                           car_gps_reader,
                           can_reader,
                           car_status
                           );
    cpr_thread.detach();

    std::thread speed_warning_thread(&MainWindow::warningMonitorThread, car_status, this);
    speed_warning_thread.detach();

}

void MainWindow::SGThread(
        std::shared_ptr<VetStopOrGo> vet_stop_or_go,
        std::shared_ptr<CarStatus> car_status,
        MainWindow *main_window){

    cv::Mat image;
    cv::Mat original_image;
    bool state;
    int startnum = 0;
    int stopnum = 0;

    while(true){
        car_status->getCurrentImage(image, original_image);
        if(original_image.empty() || original_image.size().width != 1280)
            continue;
        //cout << image.size() << endl;
        //cout << original_image.size() << endl;

        state = vet_stop_or_go->start(original_image, vet_stop_or_go->car_data);
        if(state){
            if(stopnum != 0){
                stopnum = 0;
                startnum = 0;
            }
            startnum++;
            if(startnum > 10)
                main_window->ui->frontcarLable->setText(QString("运行中"));
        } else {
            if(startnum != 0){
                startnum = 0;
                stopnum = 0;
            }
            stopnum++;
            if(stopnum > 30)
                main_window->ui->frontcarLable->setText(QString("暂停中"));
        }
    }
}

string MainWindow::gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
           to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
           to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

void MainWindow::cameraCaptureThread(std::shared_ptr<CarStatus> car_status) {

    /******以下是CIS摄像头开启方式******/
    int capture_width = 640 ;
    int capture_height = 480 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int framerate = 30 ;
    int flip_method = 0 ;

    //创建管道
    string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
    display_width,
    display_height,
    framerate,
    flip_method);

    //管道与视频流绑定
    VideoCapture video(pipeline, CAP_GSTREAMER);

    

    if(!video.isOpened())
    {
        QMessageBox::critical(
            nullptr, "Camera Error",
            "Could not read from camera");
        return;
    }


    /****以下是USB摄像头开启方式*********/
    /*cv::VideoCapture video;


    if (!video.open(0)) {
        QMessageBox::critical(
            nullptr, "摄像头错误",
            "无法从摄像头读取数据！");
        return;
    }*/
    
    while (true) {
        Mat frame;
        video >> frame;
        //cout << frame.size() << endl;
        //翻转图像CIS时需要用到
        flip(frame, frame, -1);
        if (frame.empty()) {
            continue;
        }
        car_status->setCurrentImage(frame);
    }
}


void MainWindow::showCameraWizard() {
    this->camera_wizard->restart();
    this->camera_wizard->showFullScreen();
}


void MainWindow::updateCameraModel(
    float car_width, float carpet_width, 
    float car_to_carpet_distance, float carpet_length,
    float tl_x, float tl_y,
    float tr_x, float tr_y,
    float br_x, float br_y,
    float bl_x, float bl_y
) {

    camera_model->updateCameraModel(car_width, carpet_width, car_to_carpet_distance, carpet_length,
        tl_x, tl_y, tr_x, tr_y, br_x, br_y, bl_x, bl_y);
}


void MainWindow::warningMonitorThread(std::shared_ptr<CarStatus> car_status, MainWindow *main_window) {

    while (true) {

        // 校准警告
        if (!main_window->camera_model->isCalibrated()) {
            main_window->ui->warningText->setText(QString("警告：摄像头还未校准。请先校准摄像头来保证安全驾驶"));
            main_window->ui->warningText->setVisible(true);
            continue;
        } else {
            main_window->ui->warningText->setVisible(false);
        }

        MaxSpeedLimit speed_limit = car_status->getMaxSpeedLimit();
        main_window->setSpeedLimit(speed_limit);

        // 播放声音
        if (speed_limit.speed_limit >= 0 &&
            !speed_limit.has_notified) {
            if (speed_limit.speed_limit > 0) {
                main_window->playAudio("traffic_signs/" + std::to_string(speed_limit.speed_limit) + ".wav");
            } else {
                main_window->playAudio("traffic_signs/00.wav");
            }
        }

        if (speed_limit.overspeed_warning &&
            !speed_limit.overspeed_warning_has_notified) {
            cout << "播放超速警告声音" << endl;
            main_window->alert("traffic_signs/warning_overspeed.wav");
        }

        CollisionWarningStatus collision_warning_status = car_status->getCollisionWarning();
        if (collision_warning_status.is_warning
            && collision_warning_status.should_notify
        ) {
            main_window->alert("collision_warning.wav");
            main_window->is_collision_warning = true;
            main_window->setLastCollisionWarningTime(Timer::getCurrentTime());
        } else if (!collision_warning_status.is_warning &&
            Timer::calcTimePassed(main_window->getLastCollisionWarningTime()) > 3000
        ) {
            main_window->is_collision_warning = false;
        }

        if (main_window->is_lane_departure_warning
            && Timer::calcTimePassed(main_window->last_lane_departure_warning_time) > LANE_DEPARTURE_WARNING_INTERVAL
        ) {
            main_window->alert("lane_departure_warning.wav");
            main_window->setLastLaneDepartureWarningTime(Timer::getCurrentTime());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }
    
}

void MainWindow::objectDetectionThread(
    std::shared_ptr<ObjectDetector> object_detector,
    std::shared_ptr<CarStatus> car_status,
    CollisionWarningController *collision_warning) {
        
    cv::Mat image;
    cv::Mat original_image;

    Timer::time_point_t car_status_start_time = car_status->getStartTime();
    TrafficSignMonitor traffic_sign_monitor(car_status);

    while (true) {

        //如果车状态改变重置交通标志识别子程序
        //（如果模拟发生改变）
        if (car_status_start_time != car_status->getStartTime()) {
            cout << "车辆状态已被重置" << endl;
            car_status_start_time = car_status->getStartTime();
            traffic_sign_monitor = TrafficSignMonitor(car_status);
        }

        car_status->getCurrentImage(image, original_image);

        if (image.empty()) {
            continue;
        }

        Timer::time_point_t begin_time = Timer::getCurrentTime();
        std::vector<TrafficObject> objects = object_detector->detect(image, original_image);
        car_status->setObjectDetectionTime(Timer::calcTimePassed(begin_time));
        traffic_sign_monitor.updateTrafficSign(objects);

        if (SHOW_DISTANCES) {
            collision_warning->calculateDistance(image, objects);
        }

        car_status->setDetectedObjects(objects);

        // 碰撞警告
        car_status->setCollisionWarning(collision_warning->isInDangerSituation(image.size(), objects));
        
    }
}

void MainWindow::laneDetectionThread(
    std::shared_ptr<LaneDetector> lane_detector, std::shared_ptr<CarStatus> car_status, MainWindow *main_window) {
    cv::Mat clone_img;
    bool lane_departure;
    while (true) {

        car_status->getCurrentImage(clone_img);
        if (clone_img.empty()) {
            continue;
        }

        //当转向时不要进行道路线检测
        if (Timer::calcTimePassed(car_status->getLastActivatedTurningSignalTime()) <= 5000) {
            main_window->is_lane_departure_warning = false;
            car_status->setDetectedLaneLines(std::vector<LaneLine>(), cv::Mat(), cv::Mat(), cv::Mat());
            continue;
        }

        #if defined (DEBUG_LANE_DETECTOR_SHOW_LINES)  || defined (DEBUG_LANE_DETECTOR_SHOW_LINE_MASK)
        cv::Mat lane_line_mask;
        cv::Mat detected_line_img;
        cv::Mat reduced_line_img;

        Timer::time_point_t begin_time = Timer::getCurrentTime();
        std::vector<LaneLine> detected_lines = lane_detector->detectLaneLines(clone_img, lane_line_mask, detected_line_img, reduced_line_img, lane_departure);
        car_status->setLaneDetectionTime(Timer::calcTimePassed(begin_time));
        car_status->setDetectedLaneLines(detected_lines, lane_line_mask, detected_line_img, reduced_line_img);
        #else
        Timer::time_point_t begin_time = Timer::getCurrentTime();
        std::vector<LaneLine> detected_lines = lane_detector->detectLaneLines(clone_img, lane_departure);
        car_status->setLaneDetectionTime(Timer::calcTimePassed(begin_time));
        car_status->setDetectedLaneLines(detected_lines);
        #endif 

        if (car_status->getCarSpeed() >= MIN_SPEED_FOR_LANE_DEPARTURE_WARNING) {
            main_window->is_lane_departure_warning = lane_departure;
        } else {
            main_window->is_lane_departure_warning = false;
        }

        this_thread::sleep_for(chrono::milliseconds(80));

    }
}

void MainWindow::carPropReaderThread(
    std::shared_ptr<CarGPSReader> car_gps_reader,
    std::shared_ptr<CANReader> can_reader,
    std::shared_ptr<CarStatus> car_status
    ) {

    while (true) {

        #ifndef DISABLE_GPS_READER
        car_gps_reader->updateProps();
        #endif

        if (USE_CAN_BUS_FOR_SIMULATION_DATA) {
            can_reader->readCANSignal();
            car_status->setCarStatus(can_reader->getSpeed(),
                can_reader->getLeftTurnSignal(),
                can_reader->getRightTurnSignal());
        }

        this_thread::sleep_for(chrono::milliseconds(10));
        
    }
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::playAudio(std::string audio_file) {
    if (!is_mute && (Timer::calcTimePassed(last_audio_time) > 2000
        || last_audio_file != audio_file)
    ) {
        //播放无声音文件来给HDMI一定的初始化时间
        system(("canberra-gtk-play -f sounds/silent.wav;canberra-gtk-play -f sounds/" + audio_file + " &").c_str());
        last_audio_time = Timer::getCurrentTime();
        last_audio_file = audio_file;
    }
}

void MainWindow::alert(std::string audio_file) {
    if (is_alert)
        playAudio(audio_file);
}

void MainWindow::closeEvent(QCloseEvent *event) {
    QApplication::quit();
    exit(0);
}

void MainWindow::startVideoGrabber() {

    Mat draw_frame;
    Timer::time_point_t last_fps_show = Timer::getCurrentTime();
    Timer::time_duration_t object_detection_time =
         car_status->getObjectDetectionTime();
    Timer::time_duration_t lane_detection_time =
         car_status->getLaneDetectionTime();


    int frame_ids = 0;
    while (true) {

        // 开始处理
        car_status->getCurrentImage(draw_frame);

        if (!draw_frame.empty()) {

            #ifndef DISABLE_LANE_DETECTOR
            std::vector<LaneLine> detected_lane_lines = car_status->getDetectedLaneLines();
                
            if (!detected_lane_lines.empty()) {

                #ifdef DEBUG_LANE_DETECTOR_SHOW_LINE_MASK
                cv::Mat lane_line_mask_copy = car_status->getLineMask();
                #endif

                #ifdef DEBUG_LANE_DETECTOR_SHOW_LINES
                cv::Mat detected_line_img_copy = car_status->getDetectedLinesViz();
                cv::Mat reduced_line_img_copy = car_status->getReducedLinesViz();

                // cv::imwrite(std::to_string(frame_ids) + "-detected_line_img.png", detected_line_img_copy);
                // cv::imwrite(std::to_string(frame_ids) + "-reduced_line_img.png", reduced_line_img_copy);
                #endif

                #ifdef DEBUG_LANE_DETECTOR_SHOW_LINE_MASK
                    if (!lane_line_mask_copy.empty()) {
                        cv::resize(lane_line_mask_copy, lane_line_mask_copy, draw_frame.size());

                        cv::Mat rgb_lane_result =
                            cv::Mat::zeros(draw_frame.size(), CV_8UC3);

                        rgb_lane_result.setTo(Scalar(255, 255, 255), lane_line_mask_copy > 0.5);
                        draw_frame.setTo(Scalar(0, 0, 0), lane_line_mask_copy > 0.5);
                        
                        cv::imwrite(std::to_string(frame_ids) + "-lanemask.png", rgb_lane_result);

                        cv::addWeighted(draw_frame, 1, rgb_lane_result, 1, 0,
                                        draw_frame);
                    }
                #endif

                #ifdef DEBUG_LANE_DETECTOR_SHOW_LINES
                    if (!detected_line_img_copy.empty()) {
                        cv::namedWindow("Detected Lines", cv::WINDOW_NORMAL);
                        cv::imshow("Detected Lines", detected_line_img_copy);
                        cv::waitKey(1);
                    }
                    if (!reduced_line_img_copy.empty()) {
                        cv::namedWindow("Reduced Lines", cv::WINDOW_NORMAL);
                        cv::imshow("Reduced Lines", reduced_line_img_copy);
                        cv::waitKey(1);
                    }
                #endif
                
            }

            #endif

            if (car_status->getCarSpeed() >= MIN_SPEED_FOR_COLLISION_WARNING && camera_model->isCalibrated()) {
                float danger_distance = car_status->getDangerDistance();
                cv::Mat danger_zone = camera_model->getBirdViewModel()->getDangerZone(draw_frame.size(), danger_distance);
                cv::Mat rgb_danger_zone = cv::Mat::zeros(draw_frame.size(), CV_8UC3);
                rgb_danger_zone.setTo(Scalar(0, 0, 255), danger_zone > 0.5);
                cv::addWeighted(draw_frame, 1, rgb_danger_zone, 0.3, 0,
                                        draw_frame);
            }

            #ifdef DEBUG_SHOW_FPS

                if (Timer::calcTimePassed(last_fps_show) > 1000) {
                    object_detection_time =
                        car_status->getObjectDetectionTime();
                    lane_detection_time =
                        car_status->getLaneDetectionTime();
                    last_fps_show = Timer::getCurrentTime();
                }

                cv::putText(draw_frame, "Object detection: " +  std::to_string(object_detection_time) + " ms", Point2f(10,10), FONT_HERSHEY_PLAIN, 0.8,  Scalar(0,0,255,255), 1.5);

                #ifndef DISABLE_LANE_DETECTOR
                cv::putText(draw_frame, "Lane detection: " + std::to_string(lane_detection_time) + " ms", Point2f(10,20), FONT_HERSHEY_PLAIN, 0.8,  Scalar(0,0,255,255), 1.5);
                #endif
                
            #endif
            

            std::vector<TrafficObject> detected_objects = car_status->getDetectedObjects();

            if (!detected_objects.empty()) {
                object_detector->drawDetections(
                    detected_objects, draw_frame);
            }

            // 显示交通标示
            MaxSpeedLimit speed_limit = getSpeedLimit();
            if (speed_limit.speed_limit > 0) {
                ml_cam::place_overlay(draw_frame, traffic_sign_images.getSpeedSignImage(speed_limit.speed_limit), 32, 32);
            }

            // 显示警告
            if (is_collision_warning) {
                ml_cam::place_overlay(draw_frame, warning_icon, 32, 88);
            }
            if (is_lane_departure_warning || Timer::calcTimePassed(getLastLaneDepartureWarningTime()) < 4000) {
                ml_cam::place_overlay(draw_frame, lane_departure_warning_icon, 32, 144);
            }
    
            // 显示当前图片
            QImage qimg(draw_frame.data, static_cast<int>(draw_frame.cols),
                        static_cast<int>(draw_frame.rows),
                        static_cast<int>(draw_frame.step),
                        QImage::Format_RGB888);
            pixmap.setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
            ui->graphicsView->fitInView(&pixmap, Qt::KeepAspectRatio);

            ui->speedLabel->setText(QString("速度: ") + QString::number(car_status->getCarSpeed()) + QString(" km/h"));
        }

        qApp->processEvents();
    }
}

// 获得当前可用摄像头数量
void MainWindow::refreshCams() {
    std::vector<struct Camera> cams;
    std::string path = "/sys/class/video4linux";
    for (const auto &entry : fs::directory_iterator(path)) {
        std::string str_path = fs::canonical(entry).u8string();

        // 获得摄像头
        int v4l_id;
        std::regex v4l_id_ex("video[\\d]+$");
        std::smatch v4l_id_sm;
        if (regex_search(str_path, v4l_id_sm, v4l_id_ex)) {
            std::cout << v4l_id_sm.str() << std::endl;
            std::string tmp = v4l_id_sm.str();
            v4l_id = std::stoi(tmp.substr(5, tmp.size() - 5));
        } else {
            continue;
        }

        // 获得摄像头标识
        std::string identifier;
        std::regex identifier_ex("\\/[\\d]+\\-[\\d]+\\.[\\d]+/");
        std::smatch identifier_sm;
        if (regex_search(str_path, identifier_sm, identifier_ex)) {
            identifier = identifier_sm.str();
        } else {
            identifier = std::to_string(v4l_id);
        }

        cams.push_back(Camera(v4l_id, identifier));
    }

}

void MainWindow::setInputSource(InputSource input_source) {
    this->input_source = input_source;
}

void MainWindow::setSimulation(Simulation *simulation) {
    this->simulation = simulation;
}

void MainWindow::setSpeedLimit(MaxSpeedLimit speed_limit) {
    std::lock_guard<std::mutex> guard(speed_limit_mutex);
    this->speed_limit = speed_limit;
}

MaxSpeedLimit MainWindow::getSpeedLimit() {
    std::lock_guard<std::mutex> guard(speed_limit_mutex);
    return speed_limit;
}

void MainWindow::openSimulationSelector() {
    this->simulation->showFullScreen();
}

void MainWindow::toggleMute() {

    if (is_mute) {
        is_mute = false;
        this->ui->muteBtn->setIcon(QIcon(":/resources/images/volume.png"));
    } else {
        is_mute = true;
        this->ui->muteBtn->setIcon(QIcon(":/resources/images/mute.png"));
    }

}

void MainWindow::toggleAlert() {

    if (is_alert) {
        is_alert = false;
        this->ui->alertBtn->setText(QString("报警：关"));
    } else {
        is_alert = true;
        this->ui->alertBtn->setText(QString("报警：开"));
    }

}

std::chrono::system_clock::time_point MainWindow::getLastCollisionWarningTime() {
    std::lock_guard<std::mutex> guard(warning_time_mutex);
    return last_collision_warning_time;
}

void MainWindow::setLastCollisionWarningTime(std::chrono::system_clock::time_point time_point) {
    std::lock_guard<std::mutex> guard(warning_time_mutex);
    last_collision_warning_time = time_point;
}

std::chrono::system_clock::time_point MainWindow::getLastLaneDepartureWarningTime() {
    std::lock_guard<std::mutex> guard(warning_time_mutex);
    return last_lane_departure_warning_time;
}

void MainWindow::setLastLaneDepartureWarningTime(std::chrono::system_clock::time_point time_point) {
    std::lock_guard<std::mutex> guard(warning_time_mutex);
    last_lane_departure_warning_time = time_point;
}

void MainWindow::ReceiveModeAndData(bool mode,bool need_data){
    is_simulation_mode = mode;
    need_simulation_data = need_data;
    if(is_simulation_mode){
        // 创建模拟窗口
        QWidget *simulation;

        simulation = new Simulation(this->car_status, this->camera_model,need_simulation_data);

        // 设置当前模拟的对象
        this->setInputSource(kInputFromSimulation);
        this->setSimulation((Simulation*)simulation);

    } else {
        ui->simulationBtn->setVisible(false);
        std::thread camera_thread(&MainWindow::cameraCaptureThread, car_status);
        camera_thread.detach();
    }
    this->showFullScreen();
}
