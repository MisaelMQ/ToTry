#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "Lepton3.hpp"

using namespace std;

// ----> Global variables
Lepton3* lepton3 = nullptr;
static bool close = false;
static bool rgb_mode = true;
// <---- Global variables

// ----> GStreamer pipeline for Raspberry Pi Camera
std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}
// <---- GStreamer pipeline

// ----> Function to get current date and time as a string
std::string current_datetime() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}
// <---- Date and time function

int main() {
    // ----> Initialize Lepton 3.5
    lepton3 = new Lepton3("/dev/spidev0.0", "/dev/i2c-0", Lepton3::DBG_NONE);
    lepton3->start();
    set_rgb_mode(rgb_mode);
    // <---- Initialize Lepton 3.5

    // ----> Initialize Raspberry Pi Camera
    int capture_width = 1280;
    int capture_height = 720;
    int display_width = 1280;
    int display_height = 720;
    int framerate = 30;
    int flip_method = 0;
    std::string pipeline = gstreamer_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open Raspberry Pi camera." << std::endl;
        return -1;
    }
    // <---- Initialize Raspberry Pi Camera

    uint8_t w, h;
    cv::namedWindow("RGB Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("IR Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat img_rgb, img_ir;

    std::cout << "Press ESC to exit" << std::endl;

    while (!close) {
        // ----> Capture from Raspberry Pi Camera
        if (cap.read(img_rgb)) {
            cv::Mat resized_rgb;
            cv::resize(img_rgb, resized_rgb, cv::Size(), 0.5, 0.5); // Resize for quick display
            cv::imshow("RGB Camera", resized_rgb);

            // Save the RGB image with timestamp
            std::string filename_rgb = "RGB_" + current_datetime() + ".jpg";
            cv::imwrite(filename_rgb, img_rgb);
        } else {
            std::cerr << "Failed to capture RGB frame" << std::endl;
        }
        // <---- Capture from Raspberry Pi Camera

        // ----> Capture from Lepton 3.5
        const uint16_t* data16 = lepton3->getLastFrame16(w, h);
        if (data16) {
            cv::Mat frame16(h, w, CV_16UC1);
            memcpy(frame16.data, data16, w * h * sizeof(uint16_t));

            double minVal, maxVal;
            cv::minMaxLoc(frame16, &minVal, &maxVal);
            frame16.convertTo(img_ir, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

            cv::Mat resized_ir;
            cv::resize(img_ir, resized_ir, cv::Size(), 3.0, 3.0); // Resize for quick display
            cv::imshow("IR Camera", resized_ir);

            // Save the IR image with timestamp
            std::string filename_ir = "IR_" + current_datetime() + ".jpg";
            cv::imwrite(filename_ir, img_ir);
        } else {
            std::cerr << "Failed to capture IR frame" << std::endl;
        }
        // <---- Capture from Lepton 3.5

        int keycode = cv::waitKey(5) & 0xff;
        if (keycode == 27) {
            close = true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(5)); // Capture frequency of ~5 seconds
    }

    // ----> Cleanup
    cap.release();
    delete lepton3;
    cv::destroyAllWindows();
    // <---- Cleanup

    return 0;
}

void set_rgb_mode(bool enable) {
    rgb_mode = enable;
    if (lepton3->enableRadiometry(!rgb_mode) < 0) {
        cerr << "Failed to set radiometry status" << std::endl;
    }
    if (lepton3->enableAgc(rgb_mode) < 0) {
        cerr << "Failed to set AGC status" << std::endl;
    }
    if (lepton3->enableRgbOutput(rgb_mode) < 0) {
        cerr << "Failed to enable RGB output" << std::endl;
    }
}