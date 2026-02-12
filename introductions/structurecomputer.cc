#include "structurecomputer.h"

#include <Eigen/LU>

void pr(Eigen::MatrixXd m) { std::cout << m << std::endl; }
void pr(Eigen::VectorXd m) { std::cout << m << std::endl; }
void pr(Eigen::Matrix3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector2d m) { std::cout << m << std::endl; }

Eigen::Vector2d backProject(const Eigen::Matrix3d& RCI,
                            const Eigen::Vector3d& rc_I,
                            const Eigen::Vector3d& X3d) {
  using namespace Eigen;
  Vector3d t = -RCI * rc_I;
  MatrixXd Pextrinsic(3, 4);
  Pextrinsic << RCI, t;
  SensorParams sp;
  MatrixXd Pc = sp.K() * Pextrinsic;
  VectorXd X(4, 1);
  X.head(3) = X3d;
  X(3) = 1;
  Vector3d x = Pc * X;
  Vector2d xc_pixels = (x.head(2) / x(2)) / sp.pixelSize();
  return xc_pixels;
}

Eigen::Vector3d pixelsToUnitVector_C(const Eigen::Vector2d& rPixels) {
  using namespace Eigen;
  SensorParams sp;
  // Convert input vector to meters
  Vector2d rMeters = rPixels * sp.pixelSize();
  // Write as a homogeneous vector, with a 1 in 3rd element
  Vector3d rHomogeneous;
  rHomogeneous.head(2) = rMeters;
  rHomogeneous(2) = 1;
  // Invert the projection operation through the camera intrinsic matrix K to
  // yield a vector rC in the camera coordinate frame that has a Z value of 1
  Vector3d rC = sp.K().lu().solve(rHomogeneous);
  // Normalize rC so that output is a unit vector
  return rC.normalized();
}

void StructureComputer::clear() {
  // Zero out contents of point_
  point_.rXIHat.fill(0);
  point_.Px.fill(0);
  // Clear bundleVec_
  bundleVec_.clear();
}

void StructureComputer::push(std::shared_ptr<const CameraBundle> bundle) {
  bundleVec_.push_back(bundle);
}

// This function is where the computation is performed to estimate the
// contents of point_.  The function returns a copy of point_.
Point StructureComputer::computeStructure() {
  // Throw an error if there are fewer than 2 CameraBundles in bundleVec_,
  // since in this case structure computation is not possible.
  auto N = bundleVec_.size();

  if (N < 2) {
    throw std::runtime_error(
        "At least 2 CameraBundle objects are "
        "needed for structure computation.");
  }

  // *********************************************************************
  // Fill in here the required steps to calculate the 3D position of the
  // feature point and its covariance.  Put these respectively in
  // point_.rXIHat and point_.Px
  // *********************************************************************

  // Initializing H and R
  Eigen::MatrixXd H(2*N, 4);
  Eigen::VectorXd Rv(2*N);
  
  for (int i = 0; i < N; i++) {
    // Set sensor params
    SensorParams sp;
    auto K = sp.K();
    auto Rc = sp.Rc()(1,1);
    auto ps = sp.pixelSize();

    // Populate given values of CameraBundle object
    auto rxp = bundleVec_[i]->rx;
    auto rx = ps*rxp;
    auto RCI = bundleVec_[i]->RCI;
    auto tI = bundleVec_[i]->rc_I;

    // Populating H and R
    auto tC = -RCI*tI;
    Eigen::MatrixXd Pextrinsic(3, 4);
    Pextrinsic << RCI, tC;
    Eigen::MatrixXd Pc = K*Pextrinsic;
    auto P1 = Pc.row(0);
    auto P2 = Pc.row(1);
    auto P3 = Pc.row(2);
    int first = (i*2);
    int second = first+1;
    H.row(first) = rx(0)*P3 - P1;
    H.row(second) = rx(1)*P3 - P2;
    Rv[first] = Rc;
    Rv[second] = Rc;
  }

  // Creating R
  auto R = Rv.asDiagonal();
  
  // Create Hr and z 
  Eigen::MatrixXd Hr(2*N, 3);
  Eigen::MatrixXd z(2*N, 1);
  Hr << H.col(0), H.col(1), H.col(2);
  z << -1*H.col(3);

  // Solve for error covarience matrix
  Eigen::Matrix3d Px = (Hr.transpose()*R.inverse()*Hr).inverse();

  // Solve for 3D location
  Eigen::Vector3d rXIHat = Px*Hr.transpose()*R.inverse()*z;

  // Allocate to point
  point_.Px = Px;
  point_.rXIHat = rXIHat;

  return point_;
}


