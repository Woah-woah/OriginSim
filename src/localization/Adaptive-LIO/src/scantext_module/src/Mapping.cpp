#include "scantext_module/Mapping.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

namespace scantext
{
namespace
{
constexpr uint32_t kCacheVersion = 1U;
constexpr uint32_t kCacheMagic = 0x53434348U; // SCCH

template <typename T>
void writeBinary(std::ostream &os, const T &value)
{
    os.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template <typename T>
bool readBinary(std::istream &is, T &value)
{
    is.read(reinterpret_cast<char *>(&value), sizeof(T));
    return static_cast<bool>(is);
}

uint64_t checksumAppend(uint64_t hash, const void *data, size_t bytes)
{
    const auto *ptr = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < bytes; ++i)
    {
        hash ^= static_cast<uint64_t>(ptr[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

uint64_t computeDescriptorChecksum(const ScanContext::SCDescriptor &sc,
                                  const ScanContext::RingKey &rk,
                                  const ScanContext::CartDescriptor &cart)
{
    uint64_t hash = 1469598103934665603ULL;
    const int sc_rows = sc.rows();
    const int sc_cols = sc.cols();
    const int rk_size = rk.size();
    const int cart_rows = cart.rows();
    const int cart_cols = cart.cols();
    hash = checksumAppend(hash, &sc_rows, sizeof(sc_rows));
    hash = checksumAppend(hash, &sc_cols, sizeof(sc_cols));
    hash = checksumAppend(hash, &rk_size, sizeof(rk_size));
    hash = checksumAppend(hash, &cart_rows, sizeof(cart_rows));
    hash = checksumAppend(hash, &cart_cols, sizeof(cart_cols));
    if (sc.size() > 0)
    {
        hash = checksumAppend(hash, sc.data(), sizeof(double) * static_cast<size_t>(sc.size()));
    }
    if (rk.size() > 0)
    {
        hash = checksumAppend(hash, rk.data(), sizeof(double) * static_cast<size_t>(rk.size()));
    }
    if (cart.size() > 0)
    {
        hash = checksumAppend(hash, cart.data(), sizeof(double) * static_cast<size_t>(cart.size()));
    }
    return hash;
}

std::string descriptorFileName(int index)
{
    return "descriptor_" + std::to_string(index) + ".bin";
}

bool writeDescriptorCache(const std::string &path,
                         const KeyFrame &kf,
                         const SCParams &params)
{
    const std::string tmp_path = path + ".tmp";
    std::ofstream os(tmp_path, std::ios::binary | std::ios::trunc);
    if (!os.is_open())
    {
        return false;
    }

    const uint32_t index = static_cast<uint32_t>(kf.index);
    const uint32_t cloud_path_len = static_cast<uint32_t>(kf.cloud_path.size());
    const uint32_t sc_rows = static_cast<uint32_t>(kf.descriptor.rows());
    const uint32_t sc_cols = static_cast<uint32_t>(kf.descriptor.cols());
    const uint32_t rk_size = static_cast<uint32_t>(kf.ring_key.size());
    const uint32_t cart_rows = static_cast<uint32_t>(kf.cart_descriptor.rows());
    const uint32_t cart_cols = static_cast<uint32_t>(kf.cart_descriptor.cols());
    const uint64_t checksum = computeDescriptorChecksum(kf.descriptor, kf.ring_key, kf.cart_descriptor);
    const int32_t num_ring = params.num_ring;
    const int32_t num_sector = params.num_sector;
    const double max_radius = params.max_radius;
    const double lidar_height = params.lidar_height;
    const uint8_t use_scpp = params.use_scpp ? 1U : 0U;
    const double scpp_search_ratio = params.scpp_search_ratio;
    const double cart_x_unit = params.cart_x_unit;
    const double cart_y_unit = params.cart_y_unit;
    const double cart_x_max = params.cart_x_max;
    const double cart_y_max = params.cart_y_max;

    const Eigen::Quaterniond q(kf.pose.rotation());
    const Eigen::Vector3d t = kf.pose.translation();
    const double pose_data[8] = {kf.time, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};

    writeBinary(os, kCacheMagic);
    writeBinary(os, kCacheVersion);
    writeBinary(os, index);
    writeBinary(os, cloud_path_len);
    writeBinary(os, sc_rows);
    writeBinary(os, sc_cols);
    writeBinary(os, rk_size);
    writeBinary(os, cart_rows);
    writeBinary(os, cart_cols);
    writeBinary(os, num_ring);
    writeBinary(os, num_sector);
    writeBinary(os, max_radius);
    writeBinary(os, lidar_height);
    writeBinary(os, use_scpp);
    writeBinary(os, scpp_search_ratio);
    writeBinary(os, cart_x_unit);
    writeBinary(os, cart_y_unit);
    writeBinary(os, cart_x_max);
    writeBinary(os, cart_y_max);
    writeBinary(os, checksum);
    os.write(reinterpret_cast<const char *>(pose_data), sizeof(pose_data));
    os.write(kf.cloud_path.data(), static_cast<std::streamsize>(kf.cloud_path.size()));
    if (kf.descriptor.size() > 0)
    {
        os.write(reinterpret_cast<const char *>(kf.descriptor.data()),
                 sizeof(double) * static_cast<std::streamsize>(kf.descriptor.size()));
    }
    if (kf.ring_key.size() > 0)
    {
        os.write(reinterpret_cast<const char *>(kf.ring_key.data()),
                 sizeof(double) * static_cast<std::streamsize>(kf.ring_key.size()));
    }
    if (kf.cart_descriptor.size() > 0)
    {
        os.write(reinterpret_cast<const char *>(kf.cart_descriptor.data()),
                 sizeof(double) * static_cast<std::streamsize>(kf.cart_descriptor.size()));
    }
    os.close();
    if (!os)
    {
        std::filesystem::remove(tmp_path);
        return false;
    }

    std::filesystem::rename(tmp_path, path);
    return true;
}
} // namespace

MappingCore::MappingCore()
{
    last_keyframe_pose_ = Eigen::Isometry3d::Identity();
    global_map_ = pcl::make_shared<ScanContext::PointCloudType>();
}

void MappingCore::initDatabaseIfNeeded()
{
    if (db_initialized_)
        return;

    db_dir_ = config_.map_save_path + "_db";

    if (std::filesystem::exists(db_dir_))
    {
        std::filesystem::remove_all(db_dir_);
    }

    std::filesystem::create_directories(db_dir_);

    pose_ofs_.open(db_dir_ + "/poses.txt", std::ios::out | std::ios::trunc);
    if (!pose_ofs_.is_open())
    {
        throw std::runtime_error("Failed to open poses.txt in " + db_dir_);
    }

    frame_index_ = 0;
    accumulated_dist_since_save_ = 0.0;
    db_initialized_ = true;
    std::filesystem::remove(db_dir_ + "/sc_cache_manifest.yaml");
}

void MappingCore::setConfig(const Config &config)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    config_ = config;
}

void MappingCore::setScanContextParams(const SCParams &params)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    scan_context_ = ScanContext(params);
}

bool MappingCore::isKeyFrame(const Eigen::Isometry3d &pose)
{
    if (keyframes_.empty())
        return true;

    Eigen::Isometry3d delta = last_keyframe_pose_.inverse() * pose;
    double dist = delta.translation().norm();
    double angle = Eigen::AngleAxisd(delta.rotation()).angle();
    return dist >= config_.keyframe_dist_thresh || angle >= config_.keyframe_angle_thresh;
}

bool MappingCore::saveMap(const std::string &path)
{
    ScanContext::PointCloudType::Ptr map_copy(new ScanContext::PointCloudType);
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!global_map_ || global_map_->empty())
            return false;
        *map_copy = *global_map_;
    }

    map_copy->width = static_cast<uint32_t>(map_copy->points.size());
    map_copy->height = 1;
    map_copy->is_dense = false;

    int ret = pcl::io::savePCDFileBinary(path, *map_copy);
    if (ret < 0)
        return false;

    std::cout << "[Mapping] Saved map with " << map_copy->size() << " points to " << path << std::endl;
    return true;
}

void MappingCore::saveMapAsync(const std::string &path)
{
    if (saving_.exchange(true))
    {
        std::cout << "[Mapping] Save already in progress, skip request." << std::endl;
        return;
    }

    std::thread([this, path]() {
        std::cout << "[Mapping] Async saving map..." << std::endl;
        const bool ok = this->saveMap(path);
        std::cout << (ok ? "[Mapping] Async map saved." : "[Mapping] Async save failed.") << std::endl;
        saving_ = false;
    }).detach();
}

bool MappingCore::saveDatabase(const std::string &directory)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (keyframes_.empty())
        return false;

    std::filesystem::create_directories(directory);

    std::filesystem::remove(directory + "/sc_cache_manifest.yaml");

    std::ofstream ofs(directory + "/poses.txt", std::ios::out | std::ios::trunc);
    if (!ofs.is_open())
        return false;

    for (const auto &kf : keyframes_)
    {
        Eigen::Quaterniond q(kf->pose.rotation());
        Eigen::Vector3d t = kf->pose.translation();
        ofs << kf->index << " " << std::fixed << kf->time << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

        const std::string cloud_name = "cloud_" + std::to_string(kf->index) + ".pcd";
        const std::string cloud_path = directory + "/" + cloud_name;
        pcl::io::savePCDFileBinary(cloud_path, *kf->cloud);

        const std::string desc_path = directory + "/" + descriptorFileName(kf->index);
        KeyFrame tmp = *kf;
        tmp.cloud_path = cloud_name;
        tmp.descriptor_cache_path = descriptorFileName(kf->index);
        writeDescriptorCache(desc_path, tmp, scan_context_.getParams());
    }

    return true;
}

std::vector<std::shared_ptr<KeyFrame>> MappingCore::getKeyFrames()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    return keyframes_;
}

bool MappingCore::addFrameWithSC(const ScanContext::PointCloudType::Ptr &cloud,
                                 const Eigen::Isometry3d &pose,
                                 double time,
                                 const ScanContext::SCDescriptor &sc,
                                 const ScanContext::RingKey &rk,
                                 std::shared_ptr<KeyFrame> *out_kf)
{
    if (!config_.enable_mapping)
        return false;
    if (!isKeyFrame(pose))
        return false;

    std::lock_guard<std::mutex> lock(map_mutex_);
    initDatabaseIfNeeded();

    auto kf = std::make_shared<KeyFrame>();
    kf->index = frame_index_++;
    kf->time = time;
    kf->pose = pose;
    kf->cloud = cloud;
    kf->descriptor = sc;
    kf->ring_key = rk;
    kf->cart_descriptor = scan_context_.makeCartContext(*cloud);
    kf->cloud_path = "cloud_" + std::to_string(kf->index) + ".pcd";
    kf->descriptor_cache_path = descriptorFileName(kf->index);
    keyframes_.push_back(kf);

    Eigen::Quaterniond q(kf->pose.rotation());
    Eigen::Vector3d t = kf->pose.translation();
    pose_ofs_ << kf->index << " " << std::fixed << time << " "
              << t.x() << " " << t.y() << " " << t.z() << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    pose_ofs_.flush();

    const std::string cloud_path = db_dir_ + "/" + kf->cloud_path;
    pcl::io::savePCDFileBinary(cloud_path, *cloud);

    const std::string desc_path = db_dir_ + "/" + kf->descriptor_cache_path;
    if (!writeDescriptorCache(desc_path, *kf, scan_context_.getParams()))
    {
        std::cerr << "[Mapping] Failed to write descriptor cache for keyframe " << kf->index << std::endl;
    }

    global_map_->points.reserve(global_map_->points.size() + cloud->points.size());
    for (const auto &p : cloud->points)
    {
        ScanContext::PointType q_pt;
        q_pt.getVector4fMap() = pose.matrix().cast<float>() * p.getVector4fMap();
        global_map_->points.emplace_back(q_pt);
    }

    const double dist = (pose.translation() - last_keyframe_pose_.translation()).norm();
    accumulated_dist_since_save_ += dist;
    if (config_.auto_save_interval > 0 && accumulated_dist_since_save_ >= config_.auto_save_interval)
    {
        saveMapAsync(config_.map_save_path);
        accumulated_dist_since_save_ = 0.0;
    }
    last_keyframe_pose_ = pose;

    if (out_kf)
    {
        *out_kf = kf;
    }
    return true;
}

} // namespace scantext
