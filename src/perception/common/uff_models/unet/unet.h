#ifndef UNET_H
#define UNET_H

#include <cuda_runtime_api.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <NvInfer.h>
#include <NvUffParser.h>

#include "../common/include/BatchStream.h"
#include "../common/include/EntropyCalibrator.h"
#include "../common/include/argsParser.h"
#include "../common/include/buffers.h"
#include "../common/include/common.h"
#include "../common/include/logger.h"
#include "../common/include/uff_model.h"

#include "utils/filesystem_include.h"

struct Unet : UffModel {
 
   public:
    Unet(const UffModelParams& params);

    // Run the TensorRT inference engine
    bool infer(const cv::Mat& input_img, cv::Mat& output_img);

   private:

    // Put input to buffer
    bool processInput(const samplesCommon::BufferManager& buffers,
                      const cv::Mat& input_img);

    // Process the output
    bool processOutput(const samplesCommon::BufferManager& buffers,
                       cv::Mat& output_img);
};

#endif