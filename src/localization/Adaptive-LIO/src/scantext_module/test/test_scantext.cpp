#include <gtest/gtest.h>
#include "scantext_module/ScanContext.hpp"

using namespace scantext;

TEST(ScanContextTest, BasicConstruction) {
    SCParams params;
    params.num_ring = 20;
    params.num_sector = 60;
    ScanContext sc(params);
    
    EXPECT_EQ(sc.getParams().num_ring, 20);
    EXPECT_EQ(sc.getParams().num_sector, 60);
}

TEST(ScanContextTest, DescriptorGeneration) {
    ScanContext sc;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    
    // Add a point at radius 10, angle 0
    pcl::PointXYZI p1; p1.x = 10.0; p1.y = 0.0; p1.z = 2.0;
    cloud.push_back(p1);
    
    auto desc = sc.makeScanContext(cloud);
    
    // Check dimensions
    EXPECT_EQ(desc.rows(), sc.getParams().num_ring);
    EXPECT_EQ(desc.cols(), sc.getParams().num_sector);
    
    // Check value
    // Radius 10, max 80, rings 20 -> 80/20 = 4m per ring. 10/4 = 2.5 -> index 2
    // Angle 0 -> index 0
    EXPECT_NEAR(desc(2, 0), 2.0, 1e-4);
}

TEST(ScanContextTest, RotationInvariance) {
    ScanContext sc;
    pcl::PointCloud<pcl::PointXYZI> cloud1;
    pcl::PointCloud<pcl::PointXYZI> cloud2;
    
    // Cloud 1: Point at x=10, y=0
    pcl::PointXYZI p1; p1.x = 10.0; p1.y = 0.0; p1.z = 1.0;
    cloud1.push_back(p1);
    
    // Cloud 2: Point rotated 90 deg (x=0, y=10)
    pcl::PointXYZI p2; p2.x = 0.0; p2.y = 10.0; p2.z = 1.0;
    cloud2.push_back(p2);
    
    auto desc1 = sc.makeScanContext(cloud1);
    auto desc2 = sc.makeScanContext(cloud2);
    
    auto res = sc.distanceBtnScanContext(desc1, desc2);
    
    // Distance should be 0 (perfect match with shift)
    EXPECT_NEAR(res.first, 0.0, 1e-4);
    
    // Shift should be num_sector / 4 (90 degrees)
    // 60 sectors / 4 = 15
    EXPECT_EQ(res.second, 15);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
