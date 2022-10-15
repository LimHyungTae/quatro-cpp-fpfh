//
// Created by Hyungtae Lim on 22. 10. 13.
//
#include <iostream>
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/geometry.h>
#include <teaser/utils.h>
// Due to the version of PCL (Original: 1.9 / Mine: 1.8 in Ubuntu 18.04)
// I copied raw files of TEASER++
// The original files can be found in:
// github.com/MIT-SPARK/TEASER-plusplus/tree/master/teaser/include/teaser
#include <quatro/matcher.h>

int main() {
  std::cout << "Include check" << std::endl;
  teaser::PLYReader reader;
  teaser::PointCloud cloudSrc;
  teaser::PointXYZ pt;
  pt.x = 1.0; pt.y = 2.0; pt.z = 3.0;
  cloudSrc.push_back(pt);
  auto status = reader.read("/home/shapelim/git/quatro_cpp_fpfh/example_data/bun_zipper_res3.ply", cloudSrc);

  teaser::FPFHEstimation fpfh;
  auto obj_descriptors = fpfh.computeFPFHFeatures(cloudSrc, 0.02, 0.04);

  // registration.h test
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.05;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
  params.rotation_cost_threshold = 0.005;
  teaser::RobustRegistrationSolver solver(params);

  // utils test
  Eigen::Matrix<double, 4, Eigen::Dynamic> eigen_test;
  eigen_test.resize(4, 10);
  eigen_test = Eigen::Matrix<double, 4, 10>::Ones(4, 10);
  teaser::utils::removeRow(eigen_test, 1);

  std::cout << "Check complete!" << std::endl;
  return 0;
}