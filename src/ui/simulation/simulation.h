#ifndef SIMULATION_H
#define SIMULATION_H

#include <QCloseEvent>
#include <QDebug>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QImage>
#include <QMainWindow>
#include <QMessageBox>
#include <QPixmap>
#include <QShortcut>
#include <QListWidgetItem>
#include <algorithm>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <regex>
#include <sstream>
#include <thread>
#include <condition_variable>
#include <QWidget>
#include <QFileDialog>
#include <chrono>
#include <fstream>
#include <string>
#include <bits/stdc++.h> 
#include "configs/config.h"
#include "ui_simulation.h"
#include "simulation_data.h"
#include "sensors/car_status.h"
#include "utils/filesystem_include.h"
#include "perception/camera_model/camera_model.h"
#include "can_bus_emitter.h"

struct SimData {
    std::string video_path;
    std::string data_path;

    SimData(std::string video_path, std::string data_path) :
        video_path(video_path), data_path(data_path) {}
};

class Simulation : public QWidget, private Ui::Simulation {
    Q_OBJECT

   private:
    std::atomic<float> car_speed = {0.0};
    std::atomic<bool> is_playing = {false};
    std::atomic<bool> playing_thread_running = {false};
    std::mutex path_mutex;
    std::string video_path;
    std::string data_file_path;
    std::shared_ptr<CarStatus> car_status;
    std::shared_ptr<CameraModel> camera_model;

    std::vector<SimData> sim_data;
    std::vector<int> selected_sim_data_indices;
    std::atomic<bool> need_data;

    CanBusEmitter can_bus_emitter;

   public:
    explicit Simulation(std::shared_ptr<CarStatus> car_status, std::shared_ptr<CameraModel> camera_model, bool need_data, QWidget *parent = nullptr);
    void setupAndConnectComponents();
    void stopPlaying();
    void startPlaying();
    void setCarSpeed(float);
    void setCarStatus(float speed, bool turning_left, bool turning_right);

   private slots:
    void selectVideoBtnClicked();
    void selectDataFileBtnClicked();
    void playBtnClicked();
    void simDataList_onselectionchange();
    void closeBtnClicked();
    void softRestart();
    void shutdown();

   private:
    static void playingThread(Simulation * this_ptr);
    void setVideoPath(std::string);
    std::string getVideoPath();
    void setDataFilePath(std::string);
    std::string getDataFilePath();
    bool isPlaying();
    void setPlaying(bool playing);

    int readSimulationData(std::string video_path, std::string data_file_path, SimulationData &sim_data);
   
};

#endif  // MAINWINDOW_H
