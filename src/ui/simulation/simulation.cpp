#include "simulation.h"
#include "configs/config.h"
#include "ui_simulation.h"

using namespace std;
using namespace cv;


void Simulation::setupAndConnectComponents() {
    setupUi(this);



    // Connect buttons
    connect(this->playBtn, SIGNAL(released()), this, SLOT(playBtnClicked()));
    connect(this->simDataList, SIGNAL(itemSelectionChanged()), this,
            SLOT(simDataList_onselectionchange()));
    connect(this->closeBtn, SIGNAL(released()), this, SLOT(closeBtnClicked()));

    connect(this->softRestartBtn, SIGNAL(released()), this, SLOT(softRestart()));
    connect(this->shutdownBtn, SIGNAL(released()), this, SLOT(shutdown()));
    
    if(need_data){

        // 开启CAN
        if (USE_CAN_BUS_FOR_SIMULATION_DATA) {
            system("sh setup_vcan.sh");
        }

        // 读取模拟数据
        std::ifstream sim_list_file(SMARTCAM_SIMULATION_LIST);


        std::string line;
        std::getline(sim_list_file, line);
        int n_simulations = std::stoi(line);

        for (size_t i = 0; i < n_simulations; ++i) {
            std::string video_path;
            std::string data_path;
            std::getline(sim_list_file, video_path);
            std::getline(sim_list_file, data_path);

            sim_data.push_back(SimData(video_path, data_path));

            std::string video_name = fs::path(video_path).filename();
            QString simulation_name_qs = QString::fromUtf8(video_name.c_str());
            QListWidgetItem *new_sim_data = new QListWidgetItem(
                QIcon(":/resources/images/play.png"),
                simulation_name_qs);
            new_sim_data->setData(Qt::UserRole, QVariant(static_cast<int>(i)));
            this->simDataList->addItem(new_sim_data);
        }
    } else {

    }

}

Simulation::Simulation(std::shared_ptr<CarStatus> car_status, std::shared_ptr<CameraModel> camera_model, bool need_data, QWidget *parent): QWidget(parent)  {
    this->need_data = need_data;
    setupAndConnectComponents();
    this->car_status = car_status;
    this->camera_model = camera_model;
}

void Simulation::softRestart() {
    system("pkill OpenADAS; ./OpenADAS &");
}

void Simulation::shutdown() {
    system("sudo shutdown now");
}


int Simulation::readSimulationData(std::string video_path, std::string data_file_path, SimulationData &sim_data) {

    sim_data = SimulationData();

    cout << video_path << endl;

    // 打开视频
    if (!sim_data.capture.open(video_path)) {
        QMessageBox::critical(
            NULL, "视频路径",
            "视频文件无法打开！");
        return 1;
    }

    // 获取FPS
    float fps = sim_data.capture.get(cv::CAP_PROP_FPS);
    int n_frames = sim_data.capture.get(cv::CAP_PROP_FRAME_COUNT);
    cout << "视频帧的数量: " << n_frames << endl;

    if(need_data){
        // 读取数据文件
        std::ifstream data_file(data_file_path);

        std::string line;
        while (std::getline(data_file, line)) {

            if (line == "VideoProps") { // Video props
                std::string line;
                while (std::getline(data_file, line)) {
                    if (line != "---") { // 块结束标志
                        std::string prop_name;
                        std::string prop_value;
                        std::istringstream iss(line);
                        iss >> prop_name >> prop_value;
                        if (prop_name == "playing_speed") {
                            sim_data.playing_fps = std::stof(prop_value);
                        } else if (prop_name == "begin_frame") {
                            sim_data.begin_frame = std::stoi(prop_value);
                        } else if (prop_name == "end_frame") {
                            sim_data.end_frame = std::stoi(prop_value);
                        }
                    } else {
                        break;
                    }
                }
            } else if (line == "CarSpeed") {

                // 跳过第一行
                std::getline(data_file, line);

                if (sim_data.begin_frame < 0) {
                    sim_data.begin_frame = 0;
                }
                if (sim_data.end_frame < 0) {
                    sim_data.end_frame = n_frames - 1;
                }
                if (sim_data.playing_fps < 0) {
                    sim_data.playing_fps = fps;
                }

                sim_data.sim_frames.reserve(sim_data.end_frame + 1);

                std::string line;
                while (std::getline(data_file, line)) {
                    if (line != "---") { // End of this block
                        int begin_frame;
                        int end_frame;
                        float speed;
                        int turning_left, turning_right;
                        std::istringstream iss(line);
                        iss >> begin_frame >> end_frame >> speed >> turning_left >> turning_right;
                        sim_data.sim_frames.push_back(
                            SimFrameData(begin_frame, end_frame, speed, turning_left, turning_right));

                        assert(begin_frame <= end_frame);

                        for (size_t i = begin_frame; i <= end_frame; ++i) {
                            sim_data.sim_frames[i].car_speed = speed;
                            sim_data.sim_frames[i].turning_left = turning_left;
                            sim_data.sim_frames[i].turning_right = turning_right;
                        }

                    } else {
                        break;
                    }
                }
            } else if (line == "CameraCalibration") {

                float car_width; float carpet_width;
                float car_to_carpet_distance; float carpet_length;
                float tl_x; float tl_y;
                float tr_x; float tr_y;
                float br_x; float br_y;
                float bl_x; float bl_y;

                std::string line;
                data_file >> line >> car_width;
                data_file >> line >> carpet_width;
                data_file >> line >> car_to_carpet_distance;
                data_file >> line >> carpet_length;
                data_file >> line >> tl_x >> line >> tl_y;
                data_file >> line >> tr_x >> line >> tr_y;
                data_file >> line >> br_x >> line >> br_y;
                data_file >> line >> bl_x >> line >> bl_y;

                camera_model->updateCameraModel(
                    car_width, carpet_width, car_to_carpet_distance, carpet_length,
                    tl_x, tl_y, tr_x, tr_y, br_x, br_y, bl_x, bl_y);

            }

            if (sim_data.begin_frame < 0) {
                sim_data.begin_frame = 0;
            }
            if (sim_data.end_frame < 0) {
                sim_data.end_frame = n_frames - 1;
            }
            if (sim_data.playing_fps < 0) {
                sim_data.playing_fps = fps;
            }
        }
    } else {
        if (sim_data.begin_frame < 0) {
            sim_data.begin_frame = 0;
        }
        if (sim_data.end_frame < 0) {
            sim_data.end_frame = n_frames - 1;
        }
        if (sim_data.playing_fps < 0) {
            sim_data.playing_fps = fps;
        }
    }

    return 0;

} 


void Simulation::playingThread(Simulation * this_ptr) {

    this_ptr->playing_thread_running = true;

    std::string video_path = this_ptr->getVideoPath();
    std::string data_file_path = this_ptr->getDataFilePath();

    SimulationData sim_data;
    int read_success = this_ptr->readSimulationData(video_path, data_file_path, sim_data);

    if (read_success != 0) {
        this_ptr->playing_thread_running = false;
        return;
    }

    cv::Mat frame;
    size_t current_frame_id = 0;

    // 设置开始帧
    if (sim_data.begin_frame != 0) {
        current_frame_id = sim_data.begin_frame;
        sim_data.capture.set(cv::CAP_PROP_POS_FRAMES, sim_data.begin_frame);
    }

    // 重置车状态
    this_ptr->car_status->reset();

    if(this_ptr->need_data){

        while (this_ptr->isPlaying()) {

            if (current_frame_id > sim_data.end_frame) {
                break;
            }

            this_ptr->setCarStatus(sim_data.sim_frames[current_frame_id].car_speed,
                    sim_data.sim_frames[current_frame_id].turning_left,
                    sim_data.sim_frames[current_frame_id].turning_right
                );

            this_ptr->playing_thread_running = true;
            sim_data.capture >> frame;

            //如果帧是空帧，则立即停止
            if (frame.empty())
                break;

            this_ptr->car_status->setCurrentImage(frame);

            std::this_thread::sleep_for(std::chrono::microseconds(int(1.0 / sim_data.playing_fps * 1e6)));

            ++current_frame_id;
        }
    } else {
        while (this_ptr->isPlaying()) {

            if (current_frame_id > sim_data.end_frame) {
                break;
            }

            this_ptr->setCarStatus(29, false, false);

            this_ptr->playing_thread_running = true;
            sim_data.capture >> frame;

            //如果帧是空帧，则立即停止
            if (frame.empty())
                break;

            this_ptr->car_status->setCurrentImage(frame);

            std::this_thread::sleep_for(std::chrono::microseconds(int(1.0 / sim_data.playing_fps * 1e6)));

            ++current_frame_id;
        }
    }
    sim_data.capture.release();
    this_ptr->playing_thread_running = false;

}

void Simulation::selectVideoBtnClicked() {

    stopPlaying();

    QString video_file = QFileDialog::getOpenFileName(this,
    tr("打开文件"), "", tr("视频文件 (*.avi *.mp4 *.mov)"));

    setVideoPath(video_file.toUtf8().constData());

}

void Simulation::selectDataFileBtnClicked() {

    stopPlaying();

    QString video_file = QFileDialog::getOpenFileName(this,
    tr("打开数据文件"), "", tr("数据文件 (*.txt)"));

    setDataFilePath(video_file.toUtf8().constData());

}

void Simulation::playBtnClicked() {

    if (isPlaying()) {
        stopPlaying();
    }

    if(need_data){
        // 开始加载模拟模式
        if (selected_sim_data_indices.empty()) {
            QMessageBox::critical(
            this, "警告",
            "请先选择模拟数据文件");
        } else {
            int selected_sim_data_id = selected_sim_data_indices[0];
            cout << "视频路径: " << sim_data[selected_sim_data_id].video_path << endl;
            cout << "数据路径: " << sim_data[selected_sim_data_id].data_path << endl;
            setVideoPath(sim_data[selected_sim_data_id].video_path);
            setDataFilePath(sim_data[selected_sim_data_id].data_path);
        }
    } else {
        selectVideoBtnClicked();
        cout << "视频路径: " << getVideoPath() << endl;
        cout << "数据路径: " << "" << endl;
        setVideoPath(getVideoPath());
        setDataFilePath("");
    }
    startPlaying();
    this->hide();

}

void Simulation::startPlaying() {

    if (getVideoPath() == "") {
        QMessageBox::critical(0,"视频路径丢啦","请选择一个视频文件!");
        return;
    }

    if(need_data){
        if (getDataFilePath() == "") {
            QMessageBox::critical(0,"数据文件路径丢啦","清选择一个数据文件 !");
            return;
        }
    }

    if (isPlaying()) {
        setPlaying(false);
    }

    // 等待播放线程停止
    while (playing_thread_running);

    // 开始新播放线程
    setPlaying(true);
    std::thread playing_thread(Simulation::playingThread, this);
    playing_thread.detach(); 

}

void Simulation::stopPlaying() {
    if (isPlaying()) {

        setPlaying(false);

        // 重要!!! 等待播放线程停止
        while (playing_thread_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

    }
}

void Simulation::setVideoPath(std::string path) {
    std::lock_guard<std::mutex> guard(path_mutex);
    video_path = path;
    videoPathLabel->setText(QString::fromStdString(video_path));
}

std::string Simulation::getVideoPath() {
    std::lock_guard<std::mutex> guard(path_mutex);
    return video_path;
}

void Simulation::setDataFilePath(std::string path) {
    std::lock_guard<std::mutex> guard(path_mutex);
    data_file_path = path;
    simDataPathLabel->setText(QString::fromStdString(data_file_path));
}

std::string Simulation::getDataFilePath() {
    std::lock_guard<std::mutex> guard(path_mutex);
    return data_file_path;
}


bool Simulation::isPlaying() {
    return is_playing;
}

void Simulation::setPlaying(bool playing) {
    is_playing = playing;
}

void Simulation::setCarSpeed(float speed) {
    car_speed = speed;
    if (!USE_CAN_BUS_FOR_SIMULATION_DATA) {
        car_status->setCarSpeed(speed);
    } else {
        can_bus_emitter.sendSpeed(speed);
    }
}

void Simulation::setCarStatus(float speed, bool turning_left, bool turning_right) {
    if (!USE_CAN_BUS_FOR_SIMULATION_DATA) {
        car_status->setCarStatus(speed, turning_left, turning_right);
    } else {
        can_bus_emitter.sendSpeed(speed);
        can_bus_emitter.sendTurnSignal(turning_left, turning_right);
    }
}

void Simulation::simDataList_onselectionchange() {
    QList<QListWidgetItem *> selected_sim_data = this->simDataList->selectedItems();

    // 存储选择的模拟数据
    selected_sim_data_indices.clear();
    for (int i = 0; i < selected_sim_data.count(); ++i) {
        selected_sim_data_indices.push_back(
            selected_sim_data[i]->data(Qt::UserRole).toInt());
    }
}


void Simulation::closeBtnClicked() {
    this->hide();
}
