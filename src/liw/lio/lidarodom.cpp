#include <yaml-cpp/yaml.h>
#include "lidarodom.h"

namespace zjloc
{
#define USE_ANALYTICAL_DERIVATE 1 //    是否使用解析求导

     lidarodom::lidarodom(/* args */)
     {
          laser_point_cov = 0.001;
          current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

          CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          CT_ICP::CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          index_frame = 1;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
     }

     lidarodom::~lidarodom()
     {
     }

     void lidarodom::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);
          options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
          options_.log_print = yaml["odometry"]["log_print"].as<bool>();
          options_.max_num_iteration = yaml["odometry"]["max_num_iteration"].as<int>();

          options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
          options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
          options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
          options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
          options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
          options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
          options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
          options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
          options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
          options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
          options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
          options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
          options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
          options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
          options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

          options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
          options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
          std::string str_motion_compensation = yaml["odometry"]["motion_compensation"].as<std::string>();
          if (str_motion_compensation == "NONE")
               options_.motion_compensation = MotionCompensation::NONE;
          else if (str_motion_compensation == "CONSTANT_VELOCITY")
               options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
          else if (str_motion_compensation == "ITERATIVE")
               options_.motion_compensation = MotionCompensation::ITERATIVE;
          else if (str_motion_compensation == "CONTINUOUS")
               options_.motion_compensation = MotionCompensation::CONTINUOUS;
          else
               std::cout << "The `motion_compensation` " << str_motion_compensation << " is not supported." << std::endl;

          std::string str_icpmodel = yaml["odometry"]["icpmodel"].as<std::string>();
          if (str_icpmodel == "POINT_TO_PLANE")
               options_.icpmodel = POINT_TO_PLANE;
          else if (str_icpmodel == "CT_POINT_TO_PLANE")
               options_.icpmodel = CT_POINT_TO_PLANE;
          else
               std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;

          options_.beta_location_consistency = yaml["odometry"]["beta_location_consistency"].as<double>();
          options_.beta_orientation_consistency = yaml["odometry"]["beta_orientation_consistency"].as<double>();
          options_.beta_constant_velocity = yaml["odometry"]["beta_constant_velocity"].as<double>();
          options_.beta_small_velocity = yaml["odometry"]["beta_small_velocity"].as<double>();
          options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
          options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();
     }

     bool lidarodom::init(const std::string &config_yaml)
     {
          config_yaml_ = config_yaml;

          auto yaml = YAML::LoadFile(config_yaml_);
          
          delay_time_ = yaml["delay_time"].as<double>();
          // lidar和IMU外参
          std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["extrinsic_R"] << std::endl;
          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          q_IL.normalized();
          lidar_R_wrt_IMU = q_IL;
          // init TIL
          TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;

          //   TODO: need init here
          CT_ICP::LidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::LidarPlaneNormFactor::q_il = TIL_.rotationMatrix();
          CT_ICP::CTLidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::CTLidarPlaneNormFactor::q_il = TIL_.rotationMatrix();

          loadOptions();
          switch (options_.motion_compensation)
          {
          case NONE:
          case CONSTANT_VELOCITY:
               options_.point_to_plane_with_distortion = false;
               options_.icpmodel = POINT_TO_PLANE;
               break;
          case ITERATIVE:
               options_.point_to_plane_with_distortion = true;
               options_.icpmodel = POINT_TO_PLANE;
               break;
          case CONTINUOUS:
               options_.point_to_plane_with_distortion = true;
               options_.icpmodel = CT_POINT_TO_PLANE;
               break;
          }
          LOG(WARNING) << "motion_compensation:" << options_.motion_compensation << ", model: " << options_.icpmodel;

          return true;
     }

     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data){
          
          if (data.first < last_timestamp_lidar_){
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::run(){

          while (true){

               // 获取数据
               std::vector<MeasureGroup> measurements;
               std::unique_lock<std::mutex> lk(mtx_buf);
               cond.wait(lk, [&]
                         { return (measurements = getMeasureMents()).size() != 0; });
               lk.unlock();

               for (auto &m : measurements){
                    // 处理量测数据
                    zjloc::common::Timer::Evaluate([&](){ ProcessMeasurements(m); },"processMeasurement");
                    {
                         auto real_time = std::chrono::high_resolution_clock::now();
                         static std::chrono::system_clock::time_point prev_real_time = real_time;

                         // 计算处理帧率
                         if (real_time - prev_real_time > std::chrono::seconds(5)){
                              auto data_time = m.lidar_end_time_;
                              static double prev_data_time = data_time;
                              auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                              auto delta_sim = data_time - prev_data_time;
                              printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                              prev_data_time = data_time;
                              prev_real_time = real_time;
                         }
                    }
               }
          }
     }

     /**
      * run 函数
     */
     void lidarodom::ProcessMeasurements(MeasureGroup &meas){

          // 观测数据, lidar， lidar_begin_time, time_span
          measures_ = meas;

          // std::cout << ANSI_DELETE_LAST_LINE;
          std::cout << ANSI_COLOR_GREEN << "============== process frame: "<< index_frame << ANSI_COLOR_RESET << std::endl;

          // 状态初始化
          zjloc::common::Timer::Evaluate([&](){ stateInitialization(); },"state init");

          //std::cout << ANSI_COLOR_GREEN << "============== stateInitialization: SUCCESS"<<ANSI_COLOR_RESET << std::endl;
          std::vector<point3D> const_surf;
          const_surf.insert(const_surf.end(), meas.lidar_.begin(), meas.lidar_.end());

          cloudFrame *p_frame;
          zjloc::common::Timer::Evaluate([&](){ 
                                               p_frame = buildFrame(const_surf, current_state,
                                                                    meas.lidar_begin_time_,
                                                                    meas.lidar_end_time_); 
                                              },
                                         "build frame"
                                        );

          //std::cout << ANSI_COLOR_GREEN << "============== buildFrame: SUCCESS"<<ANSI_COLOR_RESET << std::endl;

          //   lio
          zjloc::common::Timer::Evaluate([&]()
                                         { poseEstimation(p_frame); },
                                         "poseEstimate");

          //std::cout << ANSI_COLOR_GREEN << "============== poseEstimate: SUCCESS"<<ANSI_COLOR_RESET << std::endl;



          SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);
          zjloc::common::Timer::Evaluate([&]()
                                         {
               std::string laser_topic = "laser";
               pub_pose_to_ros(laser_topic, pose_of_lo_, meas.lidar_end_time_);

               // laser_topic = "velocity";
               // SE3 pred_pose = eskf_.GetNominalSE3();
               // Eigen::Vector3d vel_world = eskf_.GetNominalVel();
               // Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse()*vel_world;
               // pub_data_to_ros(laser_topic, vel_base.x(), 0);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pose_of_lo_.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, 0);
               } },
                                         "pub cloud");

          //std::cout << ANSI_COLOR_GREEN << "============== pub cloud: SUCCESS"<<ANSI_COLOR_RESET << std::endl;
                               

          p_frame->p_state = new state(current_state, true);

          state *tmp_state = new state(current_state, true);
          all_state_frame.push_back(tmp_state); // 将当前的state 添加到all_state_frame当中
          current_state = new state(current_state, false);

          index_frame++;
          p_frame->release(); // 销毁p_frame

          std::vector<point3D>().swap(meas.lidar_);
          std::vector<point3D>().swap(const_surf);

          //std::cout << ANSI_COLOR_GREEN << "============== ProcessMeasurements: SUCCESS"<<ANSI_COLOR_RESET << std::endl;
     }

     void lidarodom::poseEstimation(cloudFrame *p_frame){
         
          //   TODO: check current_state data
          // 从第二帧开始进行优化
          if (index_frame > 1){
               zjloc::common::Timer::Evaluate([&](){ optimize(p_frame); },"optimize");
          }

          // 当前帧的点添加到地图当中
          bool add_points = true;
          if (add_points){ //   update map here
               zjloc::common::Timer::Evaluate([&](){ map_incremental(p_frame); },"map update");
          }

          // 利用当前帧的位置，对地图进行裁剪，从而将地图维持在一个可限范围之内options_.max_distance
          zjloc::common::Timer::Evaluate([&](){ lasermap_fov_segment(); },"fov segment");
     }

     void lidarodom::optimize(cloudFrame *p_frame){

          state *previous_state = nullptr;
          Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
          Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
          Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

          state *curr_state = p_frame->p_state;
          Eigen::Quaterniond begin_quat = Eigen::Quaterniond(curr_state->rotation_begin);
          Eigen::Quaterniond end_quat = Eigen::Quaterniond(curr_state->rotation);
          Eigen::Vector3d begin_t = curr_state->translation_begin;
          Eigen::Vector3d end_t = curr_state->translation;

          if (p_frame->frame_id > 1){
               if (options_.log_print)
                    std::cout << "all_state_frame.size():" << all_state_frame.size() << ", " << p_frame->frame_id << std::endl;

               previous_state = all_state_frame[p_frame->frame_id - 2];
               previous_translation = previous_state->translation;
               previous_velocity = previous_state->translation - previous_state->translation_begin;
               previous_orientation = previous_state->rotation;
          }
          if (options_.log_print){
               std::cout << "prev end: " << previous_translation.transpose() << std::endl;
               std::cout << "curr begin: " << p_frame->p_state->translation_begin.transpose()
                         << "\ncurr end: " << p_frame->p_state->translation.transpose() << std::endl;
          }

          //  对当前p_frame的点进行下采样，将下采样之后的点存在surf_keypoints中
          std::vector<point3D> surf_keypoints;
          gridSampling(p_frame->point_surf, 
                       surf_keypoints, 
                       options_.sampling_rate * options_.surf_res
                      );

          // 点的个数
          size_t num_size = p_frame->point_surf.size();
          
          // 定义transofrmeKeypoints
          auto transformKeypoints = [&](std::vector<point3D> &point_frame){
               
               Eigen::Matrix3d R;
               Eigen::Vector3d t;
               for (auto &keypoint : point_frame){

                    if (options_.point_to_plane_with_distortion ||
                        options_.icpmodel == IcpModel::CT_POINT_TO_PLANE){
                         
                         double alpha_time = keypoint.alpha_time;

                         Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
                         q.normalize();
                         R = q.toRotationMatrix();
                         t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
                    }
                    else{
                         R = end_quat.normalized().toRotationMatrix();
                         t = end_t;
                    }

                    // 将原始点变换到world frame中
                    keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
               }
          };

          for (int iter(0); iter < options_.max_num_iteration; iter++){

               // 将点云中的每个点变换到worldframe中，注意原始点坐标保留，每次都是从原始点进行变换
               // 和前面的计算重复了，在buildFrame中已经计算过一次了，放在后面更合适
               transformKeypoints(surf_keypoints);

               // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));
               ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
               ceres::Problem::Options problem_options;
               ceres::Problem problem(problem_options);
#ifdef USE_ANALYTICAL_DERIVATE
               ceres::LocalParameterization *parameterization = new RotationParameterization();
#else
               auto *parameterization = new ceres::EigenQuaternionParameterization();
#endif

               switch (options_.icpmodel){

                    case IcpModel::CT_POINT_TO_PLANE:
                         // 优化参数 R_b, R_e, t_b, t_e
                         problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);
                         problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                         problem.AddParameterBlock(&begin_t.x(), 3);
                         problem.AddParameterBlock(&end_t.x(), 3);
                         break;
                    case IcpModel::POINT_TO_PLANE:
                         problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                         problem.AddParameterBlock(&end_t.x(), 3);
                         break;
               }

               // ICP constraint
               std::vector<ceres::CostFunction *> surfFactor;
               std::vector<Eigen::Vector3d> normalVec;
               addSurfCostFactor(surfFactor, normalVec, surf_keypoints, p_frame);
               int surf_num = 0;
               if (options_.log_print)
                    std::cout << "get factor: " << surfFactor.size() << std::endl;
               for (auto &e : surfFactor){
                    surf_num++;
                    switch (options_.icpmodel){
                    case IcpModel::CT_POINT_TO_PLANE:
                         problem.AddResidualBlock(e, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
                         break;
                    case IcpModel::POINT_TO_PLANE:
                         problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
                         break;
                    }
                    // if (surf_num > options_.max_num_residuals)
                    //      break;
               }
               //   release
               std::vector<Eigen::Vector3d>().swap(normalVec);
               std::vector<ceres::CostFunction *>().swap(surfFactor);


               // add constraint factor
               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE){

                    if (options_.beta_location_consistency > 0.){ //   location consistency
                    
                         auto *cost_location_consistency =
                             CT_ICP::LocationConsistencyFunctor::Create(previous_translation, sqrt(surf_num * options_.beta_location_consistency));

                         problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());
                    }

                    if (options_.beta_orientation_consistency > 0.){ // orientation consistency
                    
                         auto *cost_rotation_consistency =
                             CT_ICP::OrientationConsistencyFunctor::Create(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency));

                         problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());
                    }

                    if (options_.beta_small_velocity > 0.){ //     small velocity
                    
                         auto *cost_small_velocity =
                             CT_ICP::SmallVelocityFunctor::Create(sqrt(surf_num * options_.beta_small_velocity));
                         problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());
                    }

                    if (options_.beta_constant_velocity > 0.){ //  const velocity
                    
                         auto *cost_velocity_consistency =
                             CT_ICP::ConstantVelocityFunctor::Create(previous_velocity, sqrt(surf_num * options_.beta_constant_velocity * laser_point_cov));
                         problem.AddResidualBlock(cost_velocity_consistency, nullptr, &begin_t.x(), &end_t.x());
                    }
               }

               if (surf_num < options_.min_num_residuals){
                    std::stringstream ss_out;
                    ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
                    ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
                    std::cout << "ERROR: " << ss_out.str();
               }

               ceres::Solver::Options options;
               options.max_num_iterations = 5;
               options.num_threads = 3;
               options.minimizer_progress_to_stdout = false;
               options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

               // ceres::Solver::Options options;
               // options.linear_solver_type = ceres::DENSE_SCHUR;
               // options.trust_region_strategy_type = ceres::DOGLEG;
               // options.max_num_iterations = 10;
               // options.minimizer_progress_to_stdout = false;
               // options.num_threads = 6;

               ceres::Solver::Summary summary;

               ceres::Solve(options, &problem, &summary);

               if (!summary.IsSolutionUsable()){
                    std::cout << summary.FullReport() << std::endl;
                    throw std::runtime_error("Error During Optimization");
               }

               begin_quat.normalize();
               end_quat.normalize();

               // begin_t，end_t, begin_quat, end_quat随着优化在不断更新
               double diff_trans = 0, diff_rot = 0;
               diff_trans += (current_state->translation_begin - begin_t).norm();
               diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);

               diff_trans += (current_state->translation - end_t).norm();
               diff_rot += AngularDistance(current_state->rotation, end_quat);

               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE){

                    p_frame->p_state->translation_begin = begin_t;
                    p_frame->p_state->rotation_begin = begin_quat;
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation_begin = begin_t;
                    current_state->translation = end_t;
                    current_state->rotation_begin = begin_quat;
                    current_state->rotation = end_quat;
               }
               if (options_.icpmodel == IcpModel::POINT_TO_PLANE){

                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation = end_t;
                    current_state->rotation = end_quat;
               }

               if (diff_rot < options_.thres_orientation_norm &&
                   diff_trans < options_.thres_translation_norm){
                    if (options_.log_print)
                         std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                    break;
               }
          }

          // 将surf_keypoints删除
          std::vector<point3D>().swap(surf_keypoints);
          if (options_.log_print){
               std::cout << "opt: " << p_frame->p_state->translation_begin.transpose()
                         << ",end: " << p_frame->p_state->translation.transpose() << std::endl;
          }

          //   transpose point before added
          // 将point_surf的raw_point 变换到世界坐标系下的 point
          transformKeypoints(p_frame->point_surf);
     }

     Neighborhood lidarodom::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }

     void lidarodom::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                       std::vector<point3D> &keypoints, const cloudFrame *p_frame)
     {

          auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                               Eigen::Vector3d &location, double &planarity_weight)
          {
               auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
               planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);

               if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
               {
                    neighborhood.normal = -1.0 * neighborhood.normal;
               }
               return neighborhood;
          };

          double lambda_weight = std::abs(options_.weight_alpha);
          double lambda_neighborhood = std::abs(options_.weight_neighborhood);
          const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
          const double sum = lambda_weight + lambda_neighborhood;

          lambda_weight /= sum;
          lambda_neighborhood /= sum;

          const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames
                                              ? 2
                                              : options_.voxel_neighborhood;

          const int kThresholdCapacity = p_frame->frame_id < options_.init_num_frames
                                             ? 1
                                             : options_.threshold_voxel_occupancy;

          size_t num = keypoints.size();
          int num_residuals = 0;

          for (int k = 0; k < num; k++)
          {
               auto &keypoint = keypoints[k];
               auto &raw_point = keypoint.raw_point;

               std::vector<voxel> voxels;
               auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
                                                       nb_voxels_visited,
                                                       options_.size_voxel_map,
                                                       options_.max_number_neighbors,
                                                       kThresholdCapacity,
                                                       options_.estimate_normal_from_neighborhood
                                                           ? nullptr
                                                           : &voxels);

               if (vector_neighbors.size() < options_.min_number_neighbors)
                    continue;

               double weight;

               Eigen::Vector3d location = TIL_ * raw_point;

               auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);

               weight = lambda_weight * weight + lambda_neighborhood *
                                                     std::exp(-(vector_neighbors[0] -
                                                                keypoint.point)
                                                                   .norm() /
                                                              (kMaxPointToPlane *
                                                               options_.min_number_neighbors));

               double point_to_plane_dist;
               std::set<voxel> neighbor_voxels;
               for (int i(0); i < options_.num_closest_neighbors; ++i)
               {
                    point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

                    if (point_to_plane_dist < options_.max_dist_to_plane_icp)
                    {

                         num_residuals++;

                         Eigen::Vector3d norm_vector = neighborhood.normal;
                         norm_vector.normalize();

                         normals.push_back(norm_vector); //   record normal

                         double norm_offset = -norm_vector.dot(vector_neighbors[i]);

                         switch (options_.icpmodel)
                         {
                         case IcpModel::CT_POINT_TO_PLANE:
                         {
#ifdef USE_ANALYTICAL_DERIVATE
                              CT_ICP::CTLidarPlaneNormFactor *cost_function =
                                  new CT_ICP::CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
#else
                              auto *cost_function = CT_ICP::CTPointToPlaneFunctor::Create(vector_neighbors[0],
                                                                                          keypoints[k].raw_point,
                                                                                          norm_vector,
                                                                                          keypoints[k].alpha_time,
                                                                                          weight);
#endif
                              surf.push_back(cost_function);
                              // problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
                              break;
                         }
                         case IcpModel::POINT_TO_PLANE:
                         {
                              Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * keypoints[k].point -
                                                          p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
#ifdef USE_ANALYTICAL_DERIVATE
                              CT_ICP::LidarPlaneNormFactor *cost_function =
                                  new CT_ICP::LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
#else
                              auto *cost_function = CT_ICP::PointToPlaneFunctor::Create(vector_neighbors[0],
                                                                                        point_end, norm_vector, weight);
#endif
                              surf.push_back(cost_function);
                              // problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());
                              break;
                         }
                         }
                    }
               }

               if (num_residuals >= options_.max_num_residuals)
                    break;
          }
     }

     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };

     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     lidarodom::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                                int nb_voxels_visited, double size_voxel_map,
                                int max_num_neighbors, int threshold_voxel_capacity,
                                std::vector<voxel> *voxels)
     {

          if (voxels != nullptr)
               voxels->reserve(max_num_neighbors);

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          priority_queue_t priority_queue;

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search != map.end())
                         {
                              const auto &voxel_block = search.value();
                              if (voxel_block.NumPoints() < threshold_voxel_capacity)
                                   continue;
                              for (int i(0); i < voxel_block.NumPoints(); ++i)
                              {
                                   auto &neighbor = voxel_block.points[i];
                                   double distance = (neighbor - point).norm();
                                   if (priority_queue.size() == max_num_neighbors)
                                   {
                                        if (distance < std::get<0>(priority_queue.top()))
                                        {
                                             priority_queue.pop();
                                             priority_queue.emplace(distance, neighbor, voxel_temp);
                                        }
                                   }
                                   else
                                        priority_queue.emplace(distance, neighbor, voxel_temp);
                              }
                         }
                    }
               }
          }

          auto size = priority_queue.size();
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
          if (voxels != nullptr)
          {
               voxels->resize(size);
          }
          for (auto i = 0; i < size; ++i)
          {
               closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
               if (voxels != nullptr)
                    (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
               priority_queue.pop();
          }

          return closest_neighbors;
     }

     void lidarodom::addPointToMap(voxelHashMap &map, 
                                   const Eigen::Vector3d &point,
                                   const double &intensity, 
                                   double voxel_size,
                                   int max_num_points_in_voxel, 
                                   double min_distance_points,
                                   int min_num_points, 
                                   cloudFrame *p_frame
                                   ){

          // 计算point 位于哪个voxel中，并在map中搜索对应的voxel
          short kx = static_cast<short>(point[0] / voxel_size);
          short ky = static_cast<short>(point[1] / voxel_size);
          short kz = static_cast<short>(point[2] / voxel_size);
          voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

          // 如果找到了对用的voxel
          if (search != map.end()){

               // 对应的voxel
               auto &voxel_block = (search.value());

               // 如果voxel中点的个数没有达到最大的个数, 即没有被填满
               if (!voxel_block.IsFull()){

                    // 计算voxel中，离point最近的点
                    double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                    for (int i(0); i < voxel_block.NumPoints(); ++i){
                         auto &_point = voxel_block.points[i];
                         double sq_dist = (_point - point).squaredNorm();
                         if (sq_dist < sq_dist_min_to_points){
                              sq_dist_min_to_points = sq_dist;
                         }
                    }
                    // 在voxel中，point离最近的点的距离都超过了min_distance_points，才考虑将point添加到voxel中
                    if (sq_dist_min_to_points > (min_distance_points * min_distance_points)){
                         // min_num_points 默认为0
                         if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points){
                              voxel_block.AddPoint(point);
                              // addPointToPcl(points_world, point, intensity, p_frame);
                         }
                    }
               }
          }
          //  point不在map当中
          else{
               // min_num_points默认为0, 则为map创建一个block, 并将point放入block中
               if (min_num_points <= 0){
                    voxelBlock block(max_num_points_in_voxel);
                    block.AddPoint(point);
                    map[voxel(kx, ky, kz)] = std::move(block);
               }
          }

          // 将点放入pcl
          addPointToPcl(points_world, point, intensity, p_frame);
     }

     void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, 
                                   const Eigen::Vector3d &point,
                                   const double &intensity, 
                                   cloudFrame *p_frame){

          pcl::PointXYZI cloudTemp;

          cloudTemp.x = point.x();
          cloudTemp.y = point.y();
          cloudTemp.z = point.z();
          cloudTemp.intensity = intensity;
          // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
          pcl_points->points.push_back(cloudTemp);
     }

     /**
      * min_num_points 默认为0
     */
     void lidarodom::map_incremental(cloudFrame *p_frame, int min_num_points){
          
          //   only surf
          for (const auto &point : p_frame->point_surf){
               // point是世界坐标系下的点
               addPointToMap(voxel_map, point.point, point.intensity,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points, p_frame);
          }

          {
               std::string laser_topic = "laser";
               pub_cloud_to_ros(laser_topic, points_world, p_frame->time_frame_end);
          }

          // 发布之后将points_world清空
          points_world->clear();
     }

     /**
      * 利用当前帧的位置，对地图进行裁剪，从而将地图维持在一个可限范围之内options_.max_distance
     */
     void lidarodom::lasermap_fov_segment(){

          //   use predict pose here
          // 当前帧的位置
          Eigen::Vector3d location = current_state->translation;
          std::vector<voxel> voxels_to_erase;
          for (auto &pair : voxel_map){
               Eigen::Vector3d pt = pair.second.points[0];
               if ((pt - location).squaredNorm() > (options_.max_distance * options_.max_distance)){
                    voxels_to_erase.push_back(pair.first);
               }
          }
          for (auto &vox : voxels_to_erase)
               voxel_map.erase(vox);
          std::vector<voxel>().swap(voxels_to_erase);
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, 
                                       state *cur_state,
                                       double timestamp_begin, 
                                       double timestamp_end){

          std::vector<point3D> frame_surf(const_surf);
          if (index_frame <= 2){
               for (auto &point_temp : frame_surf){
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }
          if (index_frame > 2){
               if (options_.motion_compensation == CONSTANT_VELOCITY){
                    distortFrame(frame_surf, cur_state->rotation_begin, cur_state->rotation, cur_state->translation_begin, cur_state->translation, R_imu_lidar, t_imu_lidar);
               }
          }

          // 对点做变换，将点变换到世界坐标系之下，实际上就是对点进行了运动补偿
          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, 
                              point_temp, 
                              cur_state->rotation_begin,
                              cur_state->rotation, 
                              cur_state->translation_begin,
                              cur_state->translation,
                              R_imu_lidar, 
                              t_imu_lidar);

          
          // 创建cloud_frame， frame_surf是去畸变的点, const_surf是原始的点
          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);
          // 起始时间戳
          p_frame->time_frame_begin = timestamp_begin;
          // 终止时间戳
          p_frame->time_frame_end = timestamp_end;
          p_frame->dt_offset = 0;
          // frame id
          p_frame->frame_id = index_frame;
          return p_frame;
     }

     void lidarodom::stateInitialization(){

          if (index_frame <= 2){ //   front 2 frame
               current_state->rotation_begin = Eigen::Quaterniond(1, 0, 0, 0);
               current_state->translation_begin = Eigen::Vector3d(0, 0, 0);
               current_state->rotation = Eigen::Quaterniond(1, 0, 0, 0);
               current_state->translation = Eigen::Vector3d(0, 0, 0);
          }
          else{
               //   use last pose
               current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
               current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;

               current_state->rotation = (all_state_frame[all_state_frame.size() - 1]->rotation) *
                                         (all_state_frame[all_state_frame.size() - 2]->rotation).inverse() *
                                         (all_state_frame[all_state_frame.size() - 1]->rotation);
               current_state->translation = all_state_frame[all_state_frame.size() - 1]->translation +
                                            (all_state_frame[all_state_frame.size() - 1]->rotation) *
                                                (all_state_frame[all_state_frame.size() - 2]->rotation).inverse() *
                                                (all_state_frame[all_state_frame.size() - 1]->translation -
                                                 all_state_frame[all_state_frame.size() - 2]->translation);
          }
     }

     std::vector<MeasureGroup> lidarodom::getMeasureMents(){
          std::vector<MeasureGroup> measurements;
          while (true){
               if (lidar_buffer_.empty())
                    return measurements;

               MeasureGroup meas;

               // lidar数据
               meas.lidar_ = lidar_buffer_.front();
               // lidar 起始时间
               meas.lidar_begin_time_ = time_buffer_.front().first;
               // lidar 终止时间
               meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
               lidar_buffer_.pop_front();
               time_buffer_.pop_front();
               // 将meas放入measurements
               measurements.push_back(meas);
          }
     }

}
