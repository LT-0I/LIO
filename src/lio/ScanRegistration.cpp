#include "LidarFeatureExtractor/LidarFeatureExtractor.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <ros/package.h>

typedef pcl::PointXYZINormal PointType;

ros::Publisher pubFullLaserCloud;
ros::Publisher pubSharpCloud;
ros::Publisher pubFlatCloud;
ros::Publisher pubNonFeature;

LidarFeatureExtractor* lidarFeatureExtractor;
pcl::PointCloud<PointType>::Ptr laserCloud;
pcl::PointCloud<PointType>::Ptr laserConerCloud;
pcl::PointCloud<PointType>::Ptr laserSurfCloud;
pcl::PointCloud<PointType>::Ptr laserNonFeatureCloud;
int Lidar_Type = 0;
int N_SCANS = 6;
bool Feature_Mode = false;
bool Use_seg = false;

namespace {
struct TicToc {
  using clock = std::chrono::steady_clock;
  clock::time_point t0;
  inline void tic() { t0 = clock::now(); }
  inline double toc() const {
    return std::chrono::duration<double, std::milli>(clock::now() - t0).count();
  }
};

bool EnsureDir(const std::string& path) {
  struct stat st {};
  if (stat(path.c_str(), &st) == 0) {
    return S_ISDIR(st.st_mode);
  }
  if (mkdir(path.c_str(), 0755) == 0 || errno == EEXIST) {
    return true;
  }
  ROS_WARN_STREAM("Failed to create dir: " << path << " err=" << strerror(errno));
  return false;
}

std::ofstream scan_time_log;
bool scan_time_ready = false;

void InitScanTimeLog() {
  if (scan_time_ready) return;
  std::string base_path = ros::package::getPath("lio_livox");
  std::string log_dir = "/tmp/lio_livox_csv";
  if (!base_path.empty()) {
    log_dir = base_path + "/logs/@csv_logs";
  }
  if (!EnsureDir(log_dir)) {
    ROS_WARN_STREAM("Scan timing log disabled: cannot prepare dir " << log_dir);
    return;
  }
  std::time_t now = std::time(nullptr);
  std::tm tm_now{};
  localtime_r(&now, &tm_now);
  std::ostringstream oss;
  oss << log_dir << "/scan_timing_" << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".csv";
  scan_time_log.open(oss.str(), std::ios::out | std::ios::trunc);
  if (scan_time_log.is_open()) {
    scan_time_log << "timestamp,total_ms,feature_extract_ms\n";
    scan_time_ready = true;
  } else {
    ROS_WARN_STREAM("Failed to open scan timing csv: " << oss.str());
  }
}
}  // namespace

void lidarCallBackHorizon(const livox_ros_driver::CustomMsgConstPtr &msg) {

  InitScanTimeLog();
  TicToc t_frame;
  t_frame.tic();
  double feat_ms = 0.0;
  sensor_msgs::PointCloud2 msg2;

  TicToc t_feat;
  t_feat.tic();
  if(Use_seg){
    lidarFeatureExtractor->FeatureExtract_with_segment(msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, msg2,N_SCANS);
  }
  else{
    lidarFeatureExtractor->FeatureExtract(msg, laserCloud, laserConerCloud, laserSurfCloud,N_SCANS,Lidar_Type);
  } 
  feat_ms = t_feat.toc();

  sensor_msgs::PointCloud2 laserCloudMsg;
  pcl::toROSMsg(*laserCloud, laserCloudMsg);
  laserCloudMsg.header = msg->header;
  laserCloudMsg.header.stamp.fromNSec(msg->timebase+msg->points.back().offset_time);
  pubFullLaserCloud.publish(laserCloudMsg);

  if (scan_time_ready) {
    scan_time_log << std::fixed << std::setprecision(6) << msg->header.stamp.toSec() << ','
                  << std::setprecision(3) << t_frame.toc() << ','
                  << feat_ms << '\n';
  }

}

void lidarCallBackHAP(const livox_ros_driver::CustomMsgConstPtr &msg) {

  InitScanTimeLog();
  TicToc t_frame;
  t_frame.tic();
  double feat_ms = 0.0;
  sensor_msgs::PointCloud2 msg2;

  TicToc t_feat;
  t_feat.tic();
  if(Use_seg){
    lidarFeatureExtractor->FeatureExtract_with_segment_hap(msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, msg2,N_SCANS);
  }
  else{
    lidarFeatureExtractor->FeatureExtract_hap(msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, N_SCANS);
  } 
  feat_ms = t_feat.toc();

  sensor_msgs::PointCloud2 laserCloudMsg;
  pcl::toROSMsg(*laserCloud, laserCloudMsg);
  laserCloudMsg.header = msg->header;
  laserCloudMsg.header.stamp.fromNSec(msg->timebase+msg->points.back().offset_time);
  pubFullLaserCloud.publish(laserCloudMsg);

  if (scan_time_ready) {
    scan_time_log << std::fixed << std::setprecision(6) << msg->header.stamp.toSec() << ','
                  << std::setprecision(3) << t_frame.toc() << ','
                  << feat_ms << '\n';
  }

}

void lidarCallBackPc2(const sensor_msgs::PointCloud2ConstPtr &msg) {
    InitScanTimeLog();
    TicToc t_frame;
    t_frame.tic();
    double feat_ms = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laser_cloud_custom(new pcl::PointCloud<pcl::PointXYZINormal>());

    pcl::fromROSMsg(*msg, *laser_cloud);

    for (uint64_t i = 0; i < laser_cloud->points.size(); i++)
    {
        auto p=laser_cloud->points.at(i);
        pcl::PointXYZINormal p_custom;
        if(Lidar_Type == 0||Lidar_Type == 1)
        {
            if(p.x < 0.01) continue;
        }
        else if(Lidar_Type == 2)
        {
            if(std::fabs(p.x) < 0.01) continue;
        }
        p_custom.x=p.x;
        p_custom.y=p.y;
        p_custom.z=p.z;
        p_custom.intensity=p.intensity;
        p_custom.normal_x=float (i)/float(laser_cloud->points.size());
        p_custom.normal_y=i%4;
        laser_cloud_custom->points.push_back(p_custom);
    }

    TicToc t_feat;
    t_feat.tic();
    lidarFeatureExtractor->FeatureExtract_Mid(laser_cloud_custom, laserConerCloud, laserSurfCloud);
    feat_ms = t_feat.toc();

    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*laser_cloud_custom, laserCloudMsg);
    laserCloudMsg.header = msg->header;
    pubFullLaserCloud.publish(laserCloudMsg);

    if (scan_time_ready) {
      scan_time_log << std::fixed << std::setprecision(6) << msg->header.stamp.toSec() << ','
                    << std::setprecision(3) << t_frame.toc() << ','
                    << feat_ms << '\n';
    }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ScanRegistration");
  ros::NodeHandle nodeHandler("~");

  ros::Subscriber customCloud,pc2Cloud;

  std::string config_file;
  int msg_type=0;
  nodeHandler.getParam("config_file", config_file);
  nodeHandler.getParam("msg_type", msg_type);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cout << "config_file error: cannot open " << config_file << std::endl;
    return false;
  }
  Lidar_Type = static_cast<int>(fsSettings["Lidar_Type"]);
  N_SCANS = static_cast<int>(fsSettings["Used_Line"]);
  Feature_Mode = static_cast<int>(fsSettings["Feature_Mode"]);
  Use_seg = static_cast<int>(fsSettings["Use_seg"]);

  int NumCurvSize = static_cast<int>(fsSettings["NumCurvSize"]);
  float DistanceFaraway = static_cast<float>(fsSettings["DistanceFaraway"]);
  int NumFlat = static_cast<int>(fsSettings["NumFlat"]);
  int PartNum = static_cast<int>(fsSettings["PartNum"]);
  float FlatThreshold = static_cast<float>(fsSettings["FlatThreshold"]);
  float BreakCornerDis = static_cast<float>(fsSettings["BreakCornerDis"]);
  float LidarNearestDis = static_cast<float>(fsSettings["LidarNearestDis"]);
  float KdTreeCornerOutlierDis = static_cast<float>(fsSettings["KdTreeCornerOutlierDis"]);


  laserCloud.reset(new pcl::PointCloud<PointType>);
  laserConerCloud.reset(new pcl::PointCloud<PointType>);
  laserSurfCloud.reset(new pcl::PointCloud<PointType>);
  laserNonFeatureCloud.reset(new pcl::PointCloud<PointType>);

  if (Lidar_Type == 0)
  {
    customCloud = nodeHandler.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, &lidarCallBackHorizon);
  }
  else if (Lidar_Type == 1)
  {
    customCloud = nodeHandler.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, &lidarCallBackHAP);
  }
  else if(Lidar_Type==2){
      if (msg_type==0)
          customCloud = nodeHandler.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, &lidarCallBackHorizon);
      else if(msg_type==1)
          pc2Cloud=nodeHandler.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &lidarCallBackPc2);
  }
  pubFullLaserCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_full_cloud", 10);
  pubSharpCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_less_sharp_cloud", 10);
  pubFlatCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_less_flat_cloud", 10);
  pubNonFeature = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_nonfeature_cloud", 10);

  lidarFeatureExtractor = new LidarFeatureExtractor(N_SCANS,NumCurvSize,DistanceFaraway,NumFlat,PartNum,
                                                    FlatThreshold,BreakCornerDis,LidarNearestDis,KdTreeCornerOutlierDis);

  ros::spin();

  return 0;
}

