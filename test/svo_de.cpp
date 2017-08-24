//Created by Peter Zborovsky {p.zborovsky at gmail dot com}

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include <assert.h>
#include "test_utils.h"


#include <iostream>
#include <fstream>
#include <string>

namespace svo {

    class BenchmarkNode
    {
        //pinhole camera
        vk::AbstractCamera* cam_;
        //SVO pipeline
        svo::FrameHandlerMono* vo_;

        std::ofstream outputFile_;

        Sophus::SE3 lastFrame_;
        Sophus::SE3 currentFrame_;
        Sophus::SE3 lastStep_;

        //counter of frames where no change in motion is detected
        int sameFrameCounter = 1;
        // Threshold for images where no change in motion is detected.
        // After exceeding this threshold restart of SVO pipeline is initialized.
        int sameFrameImagesThresh = 20;
        double timeStep_ = 50.101; //Default value which is never used

    public:
        string outputFileName_ = "SVO_data-enrichment_output_file.csv"; //default output file name
        string cameraCalibFileName_;
        string videoFileName_;

        ~BenchmarkNode();
        //open all necessary files
        bool openIOStreams();
        //run video file and saves output. when this is over program is over
        bool runVideoFile();
        //writes processed frame data to output file
        void writeStepToCSV();
        //writes all unprocessed frames data to output file
        //used for frames that were unprocessed due to system restart
        void writeLastPosesToCSV();
        //indicates whether current frame motion estimate same as previous frame motion estimate
        bool sameAsLastFrame();
        //reset motion estimate of last frame to default values. used when SVO pipeline
        //reset is initiated
        void resetLastFrame();
        //loads camera calibration file and create new pinhole camera model
        bool loadCameraCalibFile();
        //start SVO pipeline
        void startSVO();

        //for testing purposes - this is most likely very wrong but Im into java and I just cant help it.
        void print(string val);
        void println(string val);
        void print(double val);
        void println(double val);
        void print(int val);
        void println(int val);


    };

    BenchmarkNode::~BenchmarkNode()
    {
        if(outputFile_.is_open()) { outputFile_.close(); }
        delete vo_;
        delete cam_;
    }
    void BenchmarkNode::startSVO()
    {
        vo_ = new svo::FrameHandlerMono(cam_);
        vo_->start();
    }
    //for testing purposes
    void BenchmarkNode::print(string val) {std::cout << val << " ";}
    void BenchmarkNode::println(string val) {std::cout << val << std::endl;}
    void BenchmarkNode::print(double val) {std::cout << val << " ";}
    void BenchmarkNode::println(double val) {std::cout << val << std::endl;}
    void BenchmarkNode::print(int val) {std::cout << val << " ";}
    void BenchmarkNode::println(int val) {std::cout << val << std::endl;}

    bool BenchmarkNode::sameAsLastFrame()
    {
        for(int i = 0; i < 6; i++)
        {
            if(currentFrame_.log()[i] != lastFrame_.log()[i])
            {
                return false;
            }
        }
        return true;
    }

    bool BenchmarkNode::openIOStreams()
    {
        outputFile_.open(outputFileName_);
        if(outputFile_.is_open())
        {
            println("output file opened ");
        }
        else
        {
            println("UNABLE TO CREATE OUTPUT FILE.");
            return false;
        }
        return true;
    }

    //loads camera calibration from yaml file
    bool BenchmarkNode::loadCameraCalibFile()
    {
        std::ifstream calibFile_;
        int width, height;
        double fx, fy, cx, cy;
        double d1, d2, d3, d4;

        string val;

        calibFile_.open(cameraCalibFileName_);
        if(calibFile_.is_open())
        {
            try
            {
                string line;
                //camera model
                getline(calibFile_,line);
                println(line);
                //width
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                width = std::stoi(line);
                println(width);
                //height
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                height = std::stoi(line);
                println(height);
                //fx
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                fx = std::stod(line);
                println(fx);
                //fy
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                fy = std::stod(line);
                println(fy);
                //cx
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                cx = std::stod(line);
                println(cx);
                //cy
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                cy = std::stod(line);
                println(cy);
                //d1
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                d1 = std::stod(line);
                println(d1);
                //d2
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                d2 = std::stod(line);
                println(d2);
                //d3
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                d3 = std::stod(line);
                println(d3);
                //d2
                getline(calibFile_, line, ' ');
                getline(calibFile_, line);
                d4 = std::stod(line);
                println(d4);
                cam_ = new vk::PinholeCamera(width, height, fx, fy, cx, cy, d1, d2, d3, d4);
                println("Camera calibration loaded from file successfully");
                calibFile_.close();
            }
            catch (std::invalid_argument)
            {
                println("WRONG CALIBRATION FILE FORMAT");
                calibFile_.close();
                return false;
            }
        }

        return true;
    }

    void BenchmarkNode::writeLastPosesToCSV()
    {
        Sophus::SE3 step;
        lastStep_ = currentFrame_ * lastFrame_.inverse();
        for (int i = 0; i < sameFrameCounter - 1; i++)
        {
            outputFile_ << vo_->lastFrame()->timestamp_ - ((sameFrameCounter - i - 1) * timeStep_) << ","
                        << lastStep_.log()[0] << ","
                        << lastStep_.log()[1] << ","
                        << lastStep_.log()[2] << ","
                        << lastStep_.log()[3] << ","
                        << lastStep_.log()[4] << ","
                        << lastStep_.log()[5] << ","
                        << -1 << std::endl;
        }
    }

    void BenchmarkNode::writeStepToCSV()
    {
        if (outputFile_.is_open()) {
            Sophus::SE3 step;

            //linearly interpolate step
            lastStep_ = currentFrame_ * lastFrame_.inverse();
            lastStep_.log()[0] /= sameFrameCounter;
            lastStep_.log()[1] /= sameFrameCounter;
            lastStep_.log()[2] /= sameFrameCounter;
            lastStep_.log()[3] /= sameFrameCounter;
            lastStep_.log()[4] /= sameFrameCounter;
            lastStep_.log()[5] /= sameFrameCounter;

            for (int i = 0; i < sameFrameCounter; i++) {
                outputFile_ << vo_->lastFrame()->timestamp_ - ((sameFrameCounter - i) * timeStep_) << ","
                            << lastStep_.log()[0] << ","
                            << lastStep_.log()[1] << ","
                            << lastStep_.log()[2] << ","
                            << lastStep_.log()[3] << ","
                            << lastStep_.log()[4] << ","
                            << lastStep_.log()[5] << ","
                            << (sameFrameCounter == 1 ? 1 : 0) << std::endl;
            }
            sameFrameCounter = 1;
        }
    }

    bool BenchmarkNode::runVideoFile()
    {
        cv::VideoCapture cap_(videoFileName_);
        if (!cap_.isOpened())
        {
            println("Cannot open the video ");
            return false;
        } else {
            println("Video file opened");
        }

        cv::Mat frame_;
        cv::Mat grey_;
        int i = 0;

        //process first frame;
        cap_.read(frame_);
        timeStep_ = cap_.get(CV_CAP_PROP_POS_MSEC);

        cv::cvtColor(frame_, grey_, CV_BGR2GRAY);
        vo_->addImage(grey_, cap_.get(CV_CAP_PROP_POS_MSEC));
        lastFrame_ = vo_->lastFrame()->T_f_w_;

        while (cap_.read(frame_))
        {
            cv::cvtColor(frame_, grey_, CV_BGR2GRAY);

            // process frame
            vo_->addImage(grey_, cap_.get(CV_CAP_PROP_POS_MSEC));
            currentFrame_ = vo_->lastFrame()->T_f_w_;

            if (sameAsLastFrame())
            {
                sameFrameCounter++;
            }
            else
            {
                writeStepToCSV();
                lastFrame_ = currentFrame_;
            }

            if (sameFrameCounter >= sameFrameImagesThresh || vo_->lostForTooLong())
            {
                print("VOLOST = ");
                println(vo_->lostForTooLong());
                print("NULLIM = ");
                println(sameFrameCounter);
                writeLastPosesToCSV();
                resetLastFrame();
                sameFrameCounter = 1;
                println("Lost for too long - Restarting svo");
                delete vo_;
                vo_ = new svo::FrameHandlerMono(cam_);
                vo_->start();
            }
            i++;
        }
        writeLastPosesToCSV();
        print("frames: ");
        println(i);
        return true;
    }

    void BenchmarkNode::resetLastFrame()
    {
        lastFrame_.log()[0] = 0;
        lastFrame_.log()[1] = 0;
        lastFrame_.log()[2] = 0;
        lastFrame_.log()[3] = 0;
        lastFrame_.log()[4] = 0;
        lastFrame_.log()[5] = 0;
    }

} // namespace svo

int main(int argc, char** argv)
{
    svo::BenchmarkNode benchmark;
    switch (argc) {
        case 3:
            benchmark.cameraCalibFileName_ = argv[1];
            benchmark.videoFileName_ = argv[2];
            break;
        case 4:
            benchmark.cameraCalibFileName_ = argv[1];
            benchmark.videoFileName_ = argv[2];
            benchmark.outputFileName_ = argv[3];
            break;
        default:
            std::cout << "ERROR: Invalid number of parameters." << std::endl
                      << "      Please provide following inputs:" << std::endl
                      << "          1. camera calibration file address" << std::endl
                      << "          2. video file address" << std::endl
                      << "          3. [optional] output file address - if not specified"
                              "output file will be saved in current directory" << std::endl;
            return -1;

    }

    {
        if(!benchmark.loadCameraCalibFile())
        {
            return -1;
        }
        benchmark.openIOStreams();
        benchmark.startSVO();
        if(!benchmark.runVideoFile())
        {
            return -1;
        }
    }
    printf("BenchmarkNode finished.\n");
    return 0;
}

