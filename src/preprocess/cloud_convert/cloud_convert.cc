#include "cloud_convert.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
// #include <execution>

namespace zjloc
{

    void CloudConvert::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, std::vector<point3D> &pcl_out)
    {
        AviaHandler(msg);
        pcl_out = cloud_out_;
    }

    void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               std::vector<point3D> &pcl_out)
    {
        switch (lidar_type_)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

        case LidarType::PANDAR:
            PandarHandler(msg);
            break;

        case LidarType::LIVOX:
            LivoxHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        pcl_out = cloud_out_;
    }

    void CloudConvert::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        // 点的总数目
        int plsize = msg->point_num;
        cloud_out_.reserve(plsize);

        // ns
        static double tm_scale = 1e9;

        // 点云帧的时间戳
        double headertime = msg->header.stamp.toSec();

        // 点云的时间跨度
        timespan_ = msg->points.back().offset_time / tm_scale;

        // std::cout << "span:" << timespan_ << ",0: " << msg->points[0].offset_time / tm_scale
        //           << " , 100: " << msg->points[100].offset_time / tm_scale << std::endl;

        // 遍历每一个点
        for (int i = 0; i < plsize; i++){

            // 判断点坐标是不是存在finite
            if (!(std::isfinite(msg->points[i].x) &&
                  std::isfinite(msg->points[i].y) &&
                  std::isfinite(msg->points[i].z)))
                continue;

            // 判断每个点的时间戳是否正常
            if (msg->points[i].offset_time / tm_scale > timespan_)
                std::cout << "------" << __FUNCTION__ << ", " << __LINE__ << ", error timespan:" << timespan_ << " < " << msg->points[i].offset_time << std::endl;

            // 对点云进行下采样
            if (i % point_filter_num_ != 0)
                continue;

            //计算点的距离
            double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                           msg->points[i].z * msg->points[i].z;

            // 过滤掉盲区以内的点以及超过距离限制的点
            if (range > 250 * 250 || range < blind * blind)
                continue;

            // 利用tag信息进行判断
            if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                point3D point_temp;
                // 点坐标
                point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                point_temp.point = point_temp.raw_point;

                // 每个点的相对时间起始点时间戳(ms)
                point_temp.relative_time = msg->points[i].offset_time / tm_scale; // curvature unit: ms

                // 每个点的反射率
                point_temp.intensity = msg->points[i].reflectivity;

                // 每个点的绝对时间戳
                point_temp.timestamp = headertime + point_temp.relative_time;

                // 每个点的相对时间(percent)
                point_temp.alpha_time = point_temp.relative_time / timespan_;

                // 点云的时间跨度
                point_temp.timespan = timespan_;

                // 每个点的线数// 对应行数
                point_temp.ring = msg->points[i].line;

                cloud_out_.push_back(point_temp);
            }
        }
    }

    void CloudConvert::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = pl_orig.points.back().t / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale
        //           << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << timespan_ << std::endl;

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            // point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.timestamp = pl_orig.points[i].timestamp;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1e6; //   1e6 - nclt kaist or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().time / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " << pl_orig.points[100].time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::PandarHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<pandar_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6

        auto time_list_pandar = [&](pandar_ros::Point &point_1, pandar_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
        //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::LivoxHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<liovx_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6 - nclt kaist or 1

        auto time_list_livox = [&](liovx_ros::Point &point_1, liovx_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_livox);
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().time / tm_scale;
        std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " << pl_orig.points[100].time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        point_filter_num_ = yaml["preprocess"]["point_filter_num"].as<int>();
        blind = yaml["preprocess"]["blind"].as<double>();

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else if (lidar_type == 5)
        {
            lidar_type_ = LidarType::PANDAR;
            LOG(INFO) << "Using Pandar LIdar";
        }
        else if (lidar_type == 6)
        {
            lidar_type_ = LidarType::LIVOX;
            LOG(INFO) << "Using Livox Lidar (sensor_msgs/PointCloud2)";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
    }

} // namespace zjloc
