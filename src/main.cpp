#include <stdlib.h>
#include <iostream>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "ui/dark_theme/dark_style.h"
#include "ui/main_window.h"
#include "ui/simulation/simulation.h"
#include "ui/input_source.h"

#include "utils/file_storage.h"
#include "utils/filesystem_include.h"


using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    
    QApplication a(argc, argv);
    const std::string keys =
        "{help h usage ? |      | print this message   }"
        "{input_source   |simulation| input source. 'camera' or 'simulation'   }"
        "{input_video_path  |      | path to video file for simulation }"
        "{input_data_path   |      | path to data file for simulation  }"
        "{on_dev_machine    |true| on development machine  }"
        ;

    CommandLineParser parser(argc, argv, keys);
    parser.about("CarSmartCam v1.0.0");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    std::string input_source = parser.get<std::string>("input_source");
    std::string input_video_path = parser.get<std::string>("input_video_path");
    std::string input_data_path = parser.get<std::string>("input_data_path");

    bool on_dev_machine = parser.get<bool>("on_dev_machine");

    // If not on development machine
    // (on Jetson Nano)
    if (!on_dev_machine) {
        // Increase volume
        system("pactl -- set-sink-volume 0 120%");
        // Remove cursor
        QApplication::setOverrideCursor(Qt::BlankCursor);
    }

    cv::setNumThreads(1);

    // Style our application with custom dark style
    //a.setStyle(new DarkStyle);


    // Create our mainwindow instance
    MainWindow *main_window = new MainWindow(0, input_source=="simulation");
    /*if (!on_dev_machine) {
        main_window->showFullScreen();
    } else {
        main_window->show();
    }*/

    main_window->startVideoGrabber();

    return a.exec();
}

