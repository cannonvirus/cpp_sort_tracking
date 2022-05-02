/**
 * SORT: A Simple, Online and Realtime Tracker
 * cannonvirus
 */

#include <iostream>
#include <fstream>
#include <map>
#include <random>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "tracker.h"
#include "utils.h"

#include <stdlib.h>
#include <random>
#include <string>
#include <regex>
#include <sstream>
#include <ctime>
#include "configparser.h"

using namespace std;

vector<string> my_split(string input, char delimiter)
{
    vector<string> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter))
    {
        answer.push_back(temp);
    }

    return answer;
}

// Fill the vector with random colors
void getRandomColors(vector<cv::Scalar> &colors, int numColors)
{
    cv::RNG rng(0);
    for (int i = 0; i < numColors; i++)
        colors.push_back(cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
}

bool memory_det_making(vector<vector<My_RotatedRect>> &bbox_per_list, CConfigParser &config, int &total_frame)
{
    string filename(config.GetString("det_file_path"));
    vector<string> lines;
    string line;

    ifstream input_file(filename);
    if (!input_file.is_open())
    {
        cerr << "Could not open the file - '"
             << filename << "'" << endl;
        return EXIT_FAILURE;
    }

    //! column remover
    if (config.GetBool("is_column"))
    {
        getline(input_file, line);
    }

    while (getline(input_file, line))
    {
        lines.push_back(line);
    }

    vector<My_RotatedRect> bbox_per_frame;
    int frame_idx = 0;
    for (const auto &i : lines)
    {

        vector<string> result = my_split(i, ',');
        if (stoi(result[0]) != frame_idx)
        {
            for (int i = frame_idx; i < stoi(result[0]); i++)
            {
                bbox_per_list.emplace_back(bbox_per_frame);
                bbox_per_frame.clear();
                frame_idx++;
            }
        }
        else
        {
            // frame_num, xywht, nose, neck, tail, true_tracking_number
            bbox_per_frame.emplace_back(My_RotatedRect{
                stof(result[1]),
                stof(result[2]),
                stof(result[3]),
                stof(result[4]),
                stof(result[5]),
                stof(result[6]),
                stof(result[7]),
                stof(result[8]),
                stof(result[9]),
                stof(result[10]),
                stof(result[11])});
        }
    }

    total_frame = frame_idx;

    return EXIT_SUCCESS;
}

int main(int argc, const char *argv[])
{

    //* ANCHOR config parser -----------------------------------------------
    //? Change path for yolo Auto-labeling path
    CConfigParser config("/works/cpp_sort_tracking/config.ini");
    // CConfigParser config("/works/YOLOX/tools/config.ini");
    //* -------------------------------------------------------------

    //! ANCHOR : Data Loader ---------------------------------------------
    vector<vector<My_RotatedRect>> bbox_per_list;
    int total_frame;
    memory_det_making(bbox_per_list, config, total_frame);
    cout << bbox_per_list.size() << endl;
    //! ------------------------------------------------------------------

    //! ANCHOR : Param Loader --------------------------------------------
    vector<cv::Scalar> colors;
    getRandomColors(colors, config.GetInt("color_number"));

    Tracker tracker;
    cv::Mat img;

    cv::VideoWriter out_video(config.GetString("video_output_path"), cv::VideoWriter::fourcc('X', '2', '6', '4'), 20, cv::Size(config.GetInt("s_video_width"), config.GetInt("s_video_height")));

    clock_t current_ticks, delta_ticks;
    clock_t fps = 0;
    float total_FPS = 0.0;

    ofstream ofile(config.GetString("out_trk_path"));

    //! ANCHOR : Video Loader --------------------------------------------
    string video_path = config.GetString("video_file_path");
    cv::VideoCapture capture(video_path);

    if (!capture.isOpened())
    {
        printf("AVI file can not open.\n");
        return 0;
    }

    //! ------------------------------------------------------------------

    for (int i = 0; i < 9999999999; i++)
    {
        capture >> img;
        if (img.empty())
        {
            ofile.close();
            if (config.GetBool("video_write"))
            {
                out_video.release();
            }
            cout << "[M-FPS] : " << total_FPS / i << endl;
            break;
        }

        current_ticks = clock();

        // if (i % 4 == 0) {
        //     tracker.Run(bbox_per_list[i]);
        // }
        tracker.Run(bbox_per_list[i]);

        const auto tracks = tracker.GetTracks();
        for (auto &trk : tracks)
        {
            const auto &bbox = trk.second.GetStateAsBbox();
            string write_line("");
            if (trk.second.coast_cycles_ < kMaxCoastCycles && (trk.second.hit_streak_ >= kMinHits || i < kMinHits))
            {
                if (config.GetBool("video_write"))
                {
                    float fix_angle;
                    if (bbox.angle >= 3.14)
                    {
                        fix_angle = (bbox.angle - 3.14) * 180 / 3.14;
                    }
                    else
                    {
                        fix_angle = bbox.angle * 180 / 3.14;
                    }
                    // ANCHOR : Overlay Part **
                    if (bbox.center_x > 0 && bbox.center_y > 0)
                    {
                        cv::ellipse(img, cv::Point2f(bbox.center_x, bbox.center_y), cv::Size2f(bbox.width / 2, bbox.height / 2), fix_angle, 0, 360, colors[trk.first], 5, 8);
                        cv::circle(img, cv::Point2f(bbox.center_x, bbox.center_y), 5, colors[trk.first], cv::FILLED, 8, 0);
                        cv::putText(img, to_string(trk.first), cv::Point2i(bbox.center_x, bbox.center_y), cv::FONT_HERSHEY_SIMPLEX, 1, colors[trk.first], 2);
                    }
                }
                write_line += to_string(i);
                write_line += ",";
                write_line += to_string(int(bbox.center_x));
                write_line += ",";
                write_line += to_string(int(bbox.center_y));
                write_line += ",";
                write_line += to_string(int(bbox.width));
                write_line += ",";
                write_line += to_string(int(bbox.height));
                write_line += ",";
                write_line += to_string(round(bbox.angle * 100) / 100).substr(0, 4);
                write_line += ",";
                write_line += to_string(int(bbox.nose_x));
                write_line += ",";
                write_line += to_string(int(bbox.nose_y));
                write_line += ",";
                write_line += to_string(int(bbox.neck_x));
                write_line += ",";
                write_line += to_string(int(bbox.neck_y));
                write_line += ",";
                write_line += to_string(int(bbox.tail_x));
                write_line += ",";
                write_line += to_string(int(bbox.tail_y));
                write_line += ",";
                write_line += to_string(trk.first);
                write_line += "\n";

                if (ofile.is_open())
                {
                    ofile << write_line;
                }
            }
        }

        delta_ticks = clock() - current_ticks;
        if (delta_ticks > 0)
        {
            fps = CLOCKS_PER_SEC / delta_ticks;
            total_FPS += fps;
        }
        cout << "[FPS] : " << fps << " Frame : " << i << endl;

        // * 이미지를 저장하는 코드를 실행할 때, 메모리를 계속 잡아놓고 있어야 하므로?
        // * 혹은 메모리를 계속 재배열해야 해서 속도가 느려짐
        // * CPU 사용량 높음
        if (config.GetBool("video_write"))
        {
            cv::Mat resized_img;
            cv::resize(img, resized_img, cv::Size(config.GetInt("s_video_width"), config.GetInt("s_video_height")), 0, 0, cv::INTER_CUBIC);
            out_video.write(resized_img);
        }

        //* Stop 조건
        if (config.GetInt("stop_frame") > 0)
        {
            if (i == config.GetInt("stop_frame"))
            {
                if (config.GetBool("video_write"))
                    out_video.release();
                break;
            }
        }
    }
    return 0;
}
