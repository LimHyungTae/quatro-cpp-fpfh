// An example showing TEASER++ registration with FPFH features with the Stanford bunny model
#include "quatro/quatro_utils.h"

int main() {
//   Load the .ply file
  teaser::PLYReader reader;
  teaser::PointCloud src_cloud;
  auto status = reader.read("./example_data/bun_zipper_res3.ply", src_cloud);
  int N = src_cloud.size();

  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  for (size_t i = 0; i < N; ++i) {
    src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
  }

  // Homogeneous coordinates
  Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
  src_h.resize(4, src.cols());
  src_h.topRows(3) = src;
  src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);


  // Apply an arbitrary SE(3) transformation
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d random_rot = get3DRot(60, 0, 0);
  T.block<3, 3>(0, 0) = random_rot;
  T(0, 3) = -0.15576939;
  T(1, 3) = -0.187705398;

  // Apply transformation
  Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T * src_h;
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

  // Convert to teaser point cloud
  teaser::PointCloud tgt_cloud;
  for (size_t i = 0; i < tgt.cols(); ++i) {
    tgt_cloud.push_back({static_cast<float>(tgt(0, i)), static_cast<float>(tgt(1, i)),
                         static_cast<float>(tgt(2, i))});
  }

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  auto obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, 0.02, 0.04);
  auto scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, 0.02, 0.04);

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);

  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params quatro_param, teaser_param;
  getParams(NOISE_BOUND / 2, "Quatro", quatro_param);
  std::chrono::steady_clock::time_point begin_q = std::chrono::steady_clock::now();
  teaser::RobustRegistrationSolver Quatro(quatro_param);
  Quatro.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end_q = std::chrono::steady_clock::now();
  auto solution_by_quatro = Quatro.getSolution();

  getParams(NOISE_BOUND / 2, "TEASER", teaser_param);
  std::chrono::steady_clock::time_point begin_t = std::chrono::steady_clock::now();
  teaser::RobustRegistrationSolver TEASER(teaser_param);
  TEASER.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
  auto solution_by_teaser = TEASER.getSolution();

  std::cout << "=====================================" << std::endl;
  std::cout << "           Quatro Results            " << std::endl;
  std::cout << "=====================================" << std::endl;
  double rot_error_quatro, ts_error_quatro;
  calcErrors(T, solution_by_quatro.rotation, solution_by_quatro.translation,
             rot_error_quatro, ts_error_quatro);
  // Compare results
  std::cout << "Error (deg): " << rot_error_quatro << std::endl;
  std::cout << "Estimated translation (m): " << ts_error_quatro << std::endl;
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_q - begin_q).count() /
               1000000.0 << std::endl;

  std::cout << "=====================================" << std::endl;
  std::cout << "          TEASER++ Results           " << std::endl;
  std::cout << "=====================================" << std::endl;
  double rot_error_teaser, ts_error_teaser;
  calcErrors(T, solution_by_teaser.rotation, solution_by_teaser.translation,
             rot_error_teaser, ts_error_teaser);
  // Compare results
  std::cout << "Error (deg): " << rot_error_teaser << std::endl;
  std::cout << "Estimated translation (m): " << ts_error_teaser << std::endl;
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_t - begin_t).count() /
               1000000.0 << std::endl;

  // Visualization
  pcl::PointCloud<pcl::PointXYZ> src_raw;
  pcl::PointCloud<pcl::PointXYZ> tgt_raw;
  pcl::PointCloud<pcl::PointXYZ> est_q, est_t;

  for (int k = 0; k < src_cloud.size(); ++k) {
    src_raw.push_back(pcl::PointXYZ(src_cloud[k].x, src_cloud[k].y, src_cloud[k].z));
    tgt_raw.push_back(pcl::PointXYZ(tgt_cloud[k].x, tgt_cloud[k].y, tgt_cloud[k].z));
  }
  Eigen::Matrix4f solution_eigen = Eigen::Matrix4f::Identity();
  solution_eigen.block<3, 3>(0, 0) = solution_by_quatro.rotation.cast<float>();
  solution_eigen.topRightCorner(3, 1) = solution_by_quatro.translation.cast<float>();
  std::cout << solution_eigen << std::endl;
  pcl::transformPointCloud(src_raw, est_q, solution_eigen);

  solution_eigen.block<3, 3>(0, 0) = solution_by_teaser.rotation.cast<float>();
  solution_eigen.topRightCorner(3, 1) = solution_by_teaser.translation.cast<float>();
  std::cout << solution_eigen << std::endl;
  pcl::transformPointCloud(src_raw, est_t, solution_eigen);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_q_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_t_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

  colorize(src_raw, *src_colored, {255, 0, 0});
  colorize(tgt_raw, *tgt_colored, {0, 255, 0});
  colorize(est_q, *est_q_colored, {0, 0, 255});
  colorize(est_t, *est_t_colored, {255, 0, 255});

  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
  viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_green");
  viewer1.addPointCloud<pcl::PointXYZRGB>(est_q_colored, "est_q_blue");
  viewer1.addPointCloud<pcl::PointXYZRGB>(est_t_colored, "est_t_magenta");

  while (!viewer1.wasStopped()) {
    viewer1.spinOnce();
  }
}



