#include "timer.h"

#include <glog/logging.h>
#include <fstream>
#include <numeric>

namespace zjloc::common
{

    std::map<std::string, Timer::TimerRecord> Timer::records_;

    void Timer::PrintAll()
    {
        std::lock_guard<std::mutex> lock(records_mutex_);
        std::cout << ANSI_COLOR_GREEN_BOLD << ">>> ===== Printing run time =====" << std::endl;
        const auto frame_it = records_.find("frame_callback_to_done");
        if (frame_it != records_.end() && !frame_it->second.time_usage_in_ms_.empty())
        {
            const auto &samples = frame_it->second.time_usage_in_ms_;
            const double mean_ms = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
            std::cout << "> [main LiDAR frame: callback entry to LIO processing done] average: "
                      << mean_ms << " ms, processed frames: " << samples.size() << std::endl;
        }
        else
        {
            std::cout << "> [main LiDAR frame: callback entry to LIO processing done] N/A, processed frames: 0" << std::endl;
        }
        for (const auto &r : records_)
        {
            if (r.first == "frame_callback_to_done") continue;
            std::cout << ANSI_COLOR_GREEN << "> [ " << r.first << " ] average time usage: "
                      << std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0) /
                             double(r.second.time_usage_in_ms_.size())
                      << " ms , called times: " << r.second.time_usage_in_ms_.size() << std::endl;
        }
        std::cout << "[timing note] frame_callback_to_done is one wall-clock sample per completed main LiDAR frame: callback entry through conversion, queue/IMU wait and LIO processing. It excludes ROS message delivery before the callback and asynchronous publication after processing." << std::endl;
        std::cout << "[timing note] processMeasurement measures only LIO processing. Other rows count stage calls and need not have one sample per frame." << std::endl;
        std::cout << "[timing note] optimize: optimize(p_frame) only, including rejected candidates; skipped for map seeding. Call selection differs from legacy index_frame > 1." << std::endl;
        std::cout << "[timing note] eskf_obs: ObserveSE3 only, including failed corrections; called only for non-seed candidates passing match validation, unlike the legacy unconditional observation." << std::endl;
        std::cout << "[timing note] poseEstimate: accepted-frame poseEstimation only (aux output transform, map maintenance, FOV segmentation). Optimization moved outside this function; NOT directly comparable to legacy poseEstimate, which included optimize." << std::endl;
        std::cout << "[timing note] match_ms in lio_quality also includes validation, correction and state callback; it excludes poseEstimate and is NOT equivalent to optimize." << std::endl;
        for (const auto &name : {"optimize", "eskf_obs", "poseEstimate"}) {
            if (records_.find(name) == records_.end()) {
                std::cout << "> [ " << name << " ] average time usage: N/A , called times: 0 (execution condition not reached)" << std::endl;
            }
        }
        std::cout << ">>> ===== Printing run time end =====" << ANSI_COLOR_RESET << std::endl;
    }

    void Timer::DumpIntoFile(const std::string &file_name)
    {
        std::lock_guard<std::mutex> lock(records_mutex_);
        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open())
        {
            std::cout << ANSI_COLOR_RED_BOLD << "Failed to open file: " << file_name << std::endl;
            return;
        }
        else
        {
            std::cout << ANSI_COLOR_GREEN_BOLD << "Dump Time Records into file: " << file_name << ANSI_COLOR_RESET << std::endl;
        }

        size_t max_length = 0;
        for (const auto &iter : records_)
        {
            ofs << iter.first << ", ";
            if (iter.second.time_usage_in_ms_.size() > max_length)
            {
                max_length = iter.second.time_usage_in_ms_.size();
            }
        }
        ofs << std::endl;

        for (size_t i = 0; i < max_length; ++i)
        {
            for (const auto &iter : records_)
            {
                if (i < iter.second.time_usage_in_ms_.size())
                {
                    ofs << iter.second.time_usage_in_ms_[i] << ",";
                }
                else
                {
                    ofs << ",";
                }
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    double Timer::GetMeanTime(const std::string &func_name)
    {
        std::lock_guard<std::mutex> lock(records_mutex_);
        if (records_.find(func_name) == records_.end())
        {
            return 0.0;
        }

        auto r = records_[func_name];
        return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) /
               double(r.time_usage_in_ms_.size());
    }

} // namespace zjloc::utils
