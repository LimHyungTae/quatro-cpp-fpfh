// An example showing TEASER++ registration with FPFH features with the Stanford bunny model
#include "quatro/quatro_utils.h"

int main() {
//   Load the .ply file
  teaser::PLYReader reader;
  teaser::PointCloud src_cloud;
  teaser::PointCloud tgt_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_vox(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_vox(new pcl::PointCloud<pcl::PointXYZ>);

  loadCloud("./example_data/000540.bin", *src_pcl);
  loadCloud("./example_data/001319.bin", *tgt_pcl);
  voxelize(src_pcl, *src_vox, 0.4);
  voxelize(tgt_pcl, *tgt_vox, 0.4);

  teaser::PointXYZ pt_teaser;
  for (const auto& pt: (*src_vox).points) {
    pt_teaser.x = pt.x;
    pt_teaser.y = pt.y;
    pt_teaser.z = pt.z;
    src_cloud.push_back(pt_teaser);
  }
  for (const auto& pt: (*tgt_vox).points) {
    pt_teaser.x = pt.x;
    pt_teaser.y = pt.y;
    pt_teaser.z = pt.z;
    tgt_cloud.push_back(pt_teaser);
  }

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  std::chrono::steady_clock::time_point begin_extraction = std::chrono::steady_clock::now();
  auto obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, 0.6, 0.9);
  auto scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, 0.6, 0.9);

  teaser::Matcher matcher;

  std::chrono::steady_clock::time_point begin_match = std::chrono::steady_clock::now();
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, true, true, true, 0.95, true);
  std::chrono::steady_clock::time_point end_match = std::chrono::steady_clock::now();

  std::cout << "Extraction taken "
            << std::chrono::duration_cast<std::chrono::microseconds>(begin_match - begin_extraction).count() /
               1000000.0 << " sec" << std::endl;
  std::cout << "Matching taken "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_match - begin_match).count() /
               1000000.0 << " sec for getting " << correspondences.size() << " correspondences" << std::endl;


  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params quatro_param, teaser_param;
  getParams(0.3, "Quatro", quatro_param);
  std::chrono::steady_clock::time_point begin_q = std::chrono::steady_clock::now();
  teaser::RobustRegistrationSolver Quatro(quatro_param);
  Quatro.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end_q = std::chrono::steady_clock::now();
  auto solution_by_quatro = Quatro.getSolution();

  getParams(0.3, "TEASER", teaser_param);
  std::chrono::steady_clock::time_point begin_t = std::chrono::steady_clock::now();
  teaser::RobustRegistrationSolver TEASER(teaser_param);
  TEASER.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
  auto solution_by_teaser = TEASER.getSolution();

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
  std::cout << "=====================================" << std::endl;
  std::cout << "           Quatro Results            " << std::endl;
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_q - begin_q).count() /
               1000000.0 << std::endl;
  std::cout << solution_eigen << std::endl;
  std::cout << "=====================================" << std::endl;
  pcl::transformPointCloud(src_raw, est_q, solution_eigen);

  solution_eigen.block<3, 3>(0, 0) = solution_by_teaser.rotation.cast<float>();
  solution_eigen.topRightCorner(3, 1) = solution_by_teaser.translation.cast<float>();
  std::cout << "=====================================" << std::endl;
  std::cout << "          TEASER++ Results           " << std::endl;
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_t - begin_t).count() /
               1000000.0 << std::endl;
  std::cout << solution_eigen << std::endl;
  std::cout << "=====================================" << std::endl;
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



