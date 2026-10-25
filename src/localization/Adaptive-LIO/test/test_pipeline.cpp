#include "lio/lidarodom.h"
#include <iostream>
#include <stdexcept>
namespace zjloc {
struct LidarOdomTestAccess {
    static void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
    static size_t MapSize(const lidarodom_m& l){size_t n=0;for(const auto& v:l.voxel_map)n+=v.second.points.size();return n;}
    static void Run(const std::string& config) {
        lidarodom_m l;require(l.init(config),"init");l.imu_need_init_=false;
        ESKFD::Options noise;l.eskf_.SetInitialConditions(noise,Vec3d::Zero(),Vec3d::Zero());l.eskf_.SetInitialTime(1.);
        l.last_imu_=std::make_shared<IMU>(1.,Vec3d::Zero(),Vec3d(0,0,9.8));
        l.TIL_=SE3(SO3::exp(Vec3d(.2,-.3,.1)),Vec3d(.1,-.2,.05));
        l.options_.surf_res=.08;l.options_.sampling_rate=1.;l.options_.min_number_neighbors=8;
        l.options_.max_number_neighbors=12;l.options_.min_num_residuals=30;l.options_.max_num_residuals=150;
        l.options_.size_voxel_map=.5;l.options_.min_distance_points=.04;l.options_.sparse_min_planarity=.05;
        l.options_.map_update_translation_trigger=0.;l.options_.enable_aux_bundle_fusion=false;
        l.cloud_pub_options.min_z_filter=-100;l.cloud_pub_options.max_z_filter=100;
        l.cloud_pub_options.enable_body_filter=false;l.cloud_pub_options.space_down_sample=.05;
        LioStateUpdate update;int updates=0,products=0,keyframes=0;CloudPtr main_world;
        SE3 keyframe_pose;
        l.SetStateCallback([&](const LioStateUpdate& s){update=s;++updates;});
        std::function<bool(std::string&,double,double)> scalar=[](auto&,double,double){return true;};l.setFunc(scalar);
        std::function<bool(std::string&,CloudPtr&,double)> cloud=[&](auto&,CloudPtr&,double stamp){
            require(std::abs(stamp-update.state.nav.timestamp_)<1e-12,"cloud time");++products;return true;};l.setFunc(cloud);
        std::function<bool(const CloudPtr&,const SE3&,double)> keyframe=[&](const auto& c,const SE3& pose,double stamp){
            require(std::abs(stamp-update.state.nav.timestamp_)<1e-12,"keyframe time");
            main_world.reset(new pcl::PointCloud<pcl::PointXYZI>(*c));keyframe_pose=pose;++keyframes;return true;};l.setFunc(keyframe);
        std::vector<Vec3d> world;
        for(int axis=0;axis<3;++axis)for(int a=-8;a<=8;++a)for(int b=-8;b<=8;++b){
            Vec3d q;q[axis]=2.;q[(axis+1)%3]=a*.08;q[(axis+2)%3]=b*.08;world.push_back(q);}
        auto measure=[&](double begin,const Vec3d& displacement,bool empty=false){
            MeasureGroup m;m.lidar_begin_time_=begin;m.lidar_end_time_=begin+.04;
            for(int i=1;i<=8;++i)m.imu_.push_back(std::make_shared<IMU>(begin+i*.005,Vec3d::Zero(),Vec3d(0,0,9.8)));
            if(!empty) for(const auto& w:world){point3D pt;pt.raw_point=l.TIL_.inverse()*(w-displacement);
                pt.timestamp=m.lidar_end_time_;pt.alpha_time=.37;pt.ring=0;pt.intensity=1.;m.lidar_.push_back(pt);}
            point3D aux;aux.raw_point=Vec3d(20,20,20);aux.timestamp=m.lidar_end_time_;aux.ring=0;
            m.aux_lidar_.push_back(aux);return m;
        };
        auto seed=measure(1.,Vec3d::Zero());l.ProcessMeasurements(seed);
        require(updates==1 && products==1 && keyframes==1,"seed products");
        require(l.match_quality_.residuals==0 && !l.geometry_quality_.valid,"seed invented geometry");
        require(l.eskf_.GetNominalState().p_.norm()<1e-12,"seed corrected pose");
        for(const auto& v:l.voxel_map)for(const auto& pt:v.second.points)require(pt.norm()<5,"publish-only aux entered map");
        auto accepted=measure(1.04,Vec3d(.04,-.03,.02));l.ProcessMeasurements(accepted);
        require(updates==2 && products==2 && keyframes==2,"accepted frame rejected");
        require(l.geometry_quality_.rank==6,"synthetic full rank");
        require(update.state.nav.p_.norm()>1e-6,"accepted frame had no correction");
        require((keyframe_pose.translation()-update.state.nav.p_).norm()<1e-12,"keyframe candidate leak");
        double max_error=0;
        for(size_t i=0;i<world.size();++i){const Vec3d expected=update.state.nav.GetSE3()*(world[i]-Vec3d(.04,-.03,.02));
            const auto& actual=main_world->points[i];max_error=std::max(max_error,(expected-Vec3d(actual.x,actual.y,actual.z)).norm());}
        require(max_error<2e-6,"registered cloud not posterior");
        const size_t map_before=MapSize(l);const double last=update.last_accepted_lidar_stamp;
        auto rejected=measure(1.08,Vec3d::Zero(),true);l.ProcessMeasurements(rejected);
        require(updates==3 && products==2 && keyframes==2,"rejected frame published");
        require(MapSize(l)==map_before && update.last_accepted_lidar_stamp==last,"rejected frame changed map/freshness");
        require((l.current_state->translation-update.state.nav.p_).norm()<1e-12,"rejected cache stale");
        auto recover=measure(1.12,Vec3d(.04,-.03,.02));l.ProcessMeasurements(recover);
        require(updates==4 && products==3 && keyframes==3,"recovery failed");
        require(update.last_accepted_lidar_stamp>last,"recovery freshness");
        require((l.current_state->translation-update.state.nav.p_).norm()<1e-12,"accepted cache stale");
        const size_t before_bad_candidate=MapSize(l);const double last_good=update.last_accepted_lidar_stamp;
        l.options_.match_max_translation=1e-6;
        auto bad_candidate=measure(1.16,Vec3d(.15,-.1,.08));l.ProcessMeasurements(bad_candidate);
        require(updates==5 && products==3 && keyframes==3,"gated candidate published");
        require(l.match_quality_.usable,"candidate rejection did not reach gate");
        require(MapSize(l)==before_bad_candidate && update.last_accepted_lidar_stamp==last_good,"gated candidate contaminated map");
        require((l.current_state->translation-update.state.nav.p_).norm()<1e-12,"failed candidate leaked to cache");
        l.options_.match_max_translation=2.;
        auto gate_recover=measure(1.20,Vec3d(.04,-.03,.02));l.ProcessMeasurements(gate_recover);
        require(updates==6 && products==4 && keyframes==4,"gated candidate recovery failed");
        require(!l.stopping_,"pipeline faulted");
        std::cout<<"PASS real ProcessMeasurements seed/accept/support-reject/gate-reject/recover; posterior cloud error="<<max_error
                 <<"; map/cache/keyframe consistency; publish-only aux exclusion\n";
    }
};
}
int main(int argc,char** argv){try{if(argc!=2)return 2;zjloc::LidarOdomTestAccess::Run(argv[1]);}
catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
