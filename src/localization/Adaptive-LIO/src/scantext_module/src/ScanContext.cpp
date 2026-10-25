#include "scantext_module/ScanContext.hpp"

#include <limits>
#include <set>

namespace scantext {

ScanContext::ScanContext(const SCParams& params) : params_(params) {}

ScanContext::SCDescriptor ScanContext::makeScanContext(const PointCloudType& scan) {
    int num_ring = params_.num_ring;
    int num_sector = params_.num_sector;
    double max_radius = params_.max_radius;
    double gap_ring = max_radius / num_ring;
    double gap_sector = 2.0 * M_PI / num_sector;

    SCDescriptor sc_desc = Eigen::MatrixXd::Zero(num_ring, num_sector);

    for (const auto& pt : scan.points) {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;

        float range = std::sqrt(x * x + y * y);
        if (range > max_radius)
            continue;

        float angle = std::atan2(y, x); // [-PI, PI]
        if (angle < 0) angle += 2.0 * M_PI; // [0, 2PI]

        int idx_ring = std::min(std::max((int)(range / gap_ring), 0), num_ring - 1);
        int idx_sector = std::min(std::max((int)(angle / gap_sector), 0), num_sector - 1);

        // ScanContext stores the maximum height in each bin
        // We add an offset to z to ensure positive values if needed, 
        // but typically just taking max z is fine. 
        // Some implementations use z + 2.0 to make it positive.
        if (sc_desc(idx_ring, idx_sector) == 0.0)
            sc_desc(idx_ring, idx_sector) = z;
        else
            sc_desc(idx_ring, idx_sector) = std::max(sc_desc(idx_ring, idx_sector), (double)z);
    }

    return sc_desc;
}

ScanContext::RingKey ScanContext::makeRingKey(const SCDescriptor& sc) {
    Eigen::VectorXd ring_key = Eigen::VectorXd::Zero(sc.rows());
    constexpr double kEps = 1e-6;
    for (int r = 0; r < sc.rows(); ++r) {
        double sum = 0.0;
        int cnt = 0;
        for (int c = 0; c < sc.cols(); ++c) {
            const double v = sc(r, c);
            if (std::abs(v) > kEps) {
                sum += v;
                cnt++;
            }
        }
        if (cnt > 0) {
            ring_key[r] = sum / static_cast<double>(cnt);
        }
    }
    return ring_key;
}

ScanContext::SectorKey ScanContext::makeSectorKey(const SCDescriptor& sc) {
    Eigen::VectorXd sector_key = Eigen::VectorXd::Zero(sc.cols());
    constexpr double kEps = 1e-6;
    for (int c = 0; c < sc.cols(); ++c) {
        double sum = 0.0;
        int cnt = 0;
        for (int r = 0; r < sc.rows(); ++r) {
            const double v = sc(r, c);
            if (std::abs(v) > kEps) {
                sum += v;
                cnt++;
            }
        }
        if (cnt > 0) {
            sector_key[c] = sum / static_cast<double>(cnt);
        }
    }
    return sector_key;
}

ScanContext::CartDescriptor ScanContext::makeCartContext(const PointCloudType& scan, double yaw_offset) {
    const double x_unit = params_.cart_x_unit;
    const double y_unit = params_.cart_y_unit;
    const double x_max = params_.cart_x_max;
    const double y_max = params_.cart_y_max;

    const int num_x = std::max(1, static_cast<int>(std::round((2.0 * x_max) / x_unit)));
    const int num_y = std::max(1, static_cast<int>(std::round((2.0 * y_max) / y_unit)));

    CartDescriptor cc = Eigen::MatrixXd::Zero(num_x, num_y);

    const double c = std::cos(yaw_offset);
    const double s = std::sin(yaw_offset);

    for (const auto& pt : scan.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        double x = static_cast<double>(pt.x);
        double y = static_cast<double>(pt.y);
        const double z = static_cast<double>(pt.z);

        if (std::abs(yaw_offset) > 1e-9) {
            const double xr = c * x - s * y;
            const double yr = s * x + c * y;
            x = xr;
            y = yr;
        }

        if (!(x > -x_max && x < x_max && y > -y_max && y < y_max)) {
            continue;
        }

        const double xs = (x >= 0.0 ? 1.0 : -1.0) * std::floor(std::abs(x) / x_unit) + std::floor(num_x / 2.0);
        const double ys = (y >= 0.0 ? 1.0 : -1.0) * std::floor(std::abs(y) / y_unit) + std::floor(num_y / 2.0);

        const int xi = static_cast<int>(xs);
        const int yi = static_cast<int>(ys);
        if (xi < 0 || xi >= num_x || yi < 0 || yi >= num_y) {
            continue;
        }

        if (cc(xi, yi) < z) {
            cc(xi, yi) = z;
        }
    }

    return cc;
}

double ScanContext::distanceBtnCartContext(const CartDescriptor& cc1, const CartDescriptor& cc2) const {
    if (cc1.rows() != cc2.rows() || cc1.cols() != cc2.cols()) {
        return 1.0;
    }

    const Eigen::Map<const Eigen::VectorXd> v1(cc1.data(), cc1.size());
    const Eigen::Map<const Eigen::VectorXd> v2(cc2.data(), cc2.size());

    const double n1 = v1.norm();
    const double n2 = v2.norm();
    if (n1 < 1e-9 || n2 < 1e-9) {
        return 1.0;
    }

    const double sim = std::clamp(v1.dot(v2) / (n1 * n2), -1.0, 1.0);
    return 1.0 - sim;
}

ScanContext::SCDescriptor ScanContext::circshift(const SCDescriptor& sc, int shift) {
    int rows = sc.rows();
    int cols = sc.cols();
    SCDescriptor shifted_sc(rows, cols);
    
    // Handle negative shift
    if(shift < 0) shift = cols + shift;
    shift = shift % cols;

    if (shift == 0) return sc;

    shifted_sc.leftCols(cols - shift) = sc.rightCols(cols - shift);
    shifted_sc.rightCols(shift) = sc.leftCols(shift);

    return shifted_sc;
}

std::pair<double, int> ScanContext::distanceBtnScanContext(const SCDescriptor& sc1, const SCDescriptor& sc2) {
    int num_sector = params_.num_sector;
    int best_shift = 0;
    double min_dist = std::numeric_limits<double>::max();

    int center_shift = 0;
    int search_half = num_sector / 2;
    if (params_.use_scpp) {
        const auto sector1 = makeSectorKey(sc1);
        const auto sector2 = makeSectorKey(sc2);
        center_shift = fastAlignUsingSectorKey(sector1, sector2);
        const int ratio_half = static_cast<int>(std::round(std::max(0.0, params_.scpp_search_ratio) * num_sector));
        search_half = std::max(1, std::min(num_sector / 2, ratio_half));
    }

    std::set<int> shifts;
    if (params_.use_scpp) {
        shifts.insert(center_shift);
        for (int i = 1; i <= search_half; ++i) {
            shifts.insert((center_shift - i + num_sector) % num_sector);
            shifts.insert((center_shift + i) % num_sector);
        }
    } else {
        for (int i = 0; i < num_sector; ++i) {
            shifts.insert(i);
            }
        }
        
    for (const int shift : shifts) {
        SCDescriptor sc1_shifted = circshift(sc1, shift);
        const double current_dist = distDirectSC(sc1_shifted, sc2);
        
        if (current_dist < min_dist) {
            min_dist = current_dist;
            best_shift = shift;
        }
    }

    if (!std::isfinite(min_dist)) {
        min_dist = 1.0;
    }

    return {min_dist, best_shift};
}

int ScanContext::fastAlignUsingSectorKey(const SectorKey& vkey_ref, const SectorKey& vkey_query) const {
    const int cols = static_cast<int>(vkey_ref.size());
    if (cols == 0 || vkey_query.size() != cols) {
        return 0;
    }

    int best_shift = 0;
    double min_norm = std::numeric_limits<double>::max();
    for (int shift = 0; shift < cols; ++shift) {
        double sum_sq = 0.0;
        for (int c = 0; c < cols; ++c) {
            const double a = vkey_ref[c];
            const double b = vkey_query[(c - shift + cols) % cols];
            const double d = a - b;
            sum_sq += d * d;
        }

        const double diff_norm = std::sqrt(sum_sq);
        if (diff_norm < min_norm) {
            min_norm = diff_norm;
            best_shift = shift;
        }
    }
    return best_shift;
}

double ScanContext::distDirectSC(const SCDescriptor& sc1, const SCDescriptor& sc2) const {
    if (sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols()) {
        return 1.0;
    }

    double sum_sector_similarity = 0.0;
    int num_eff_cols = 0;
    for (int c = 0; c < sc1.cols(); ++c) {
        const auto col1 = sc1.col(c);
        const auto col2 = sc2.col(c);

        const double n1 = col1.norm();
        const double n2 = col2.norm();
        if (n1 < 1e-9 || n2 < 1e-9) {
            continue;
        }

        const double cos_similarity = std::clamp(col1.dot(col2) / (n1 * n2), -1.0, 1.0);
        sum_sector_similarity += cos_similarity;
        num_eff_cols++;
    }

    if (num_eff_cols == 0) {
        return 1.0;
    }

    const double sc_sim = sum_sector_similarity / static_cast<double>(num_eff_cols);
    return 1.0 - sc_sim;
}

} // namespace scantext
