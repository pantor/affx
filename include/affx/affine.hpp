#pragma once

#include <random>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


namespace affx {

struct Affine {
private:
  using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>;

  Eigen::Vector3d get_angles() const {
    Eigen::Vector3d angles = Euler::FromRotation<false, false, false>(data.rotation()).angles();
    Eigen::Vector3d angles_equal;
    angles_equal << angles[0] - M_PI, M_PI - angles[1], angles[2] - M_PI;

    if (angles_equal[1] > M_PI) {
      angles_equal[1] -= 2 * M_PI;
    }
    if (angles_equal[2] < -M_PI) {
      angles_equal[2] += 2 * M_PI;
    }

    if (angles.norm() < angles_equal.norm()) {
      return angles;
    }
    return angles_equal;
  }


public:
  Eigen::Isometry3d data;

  Affine() {
    this->data = Eigen::Isometry3d::Identity();
  }

  Affine(const Eigen::Isometry3d& data) {
    this->data = data;
  }

  Affine(double x, double y, double z, double a, double b, double c) {
    data.translation() = Eigen::Vector3d(x, y, z);
    data.linear() = Euler(a, b, c).toRotationMatrix();
  }

  Affine(double x, double y, double z, double q_w, double q_x, double q_y, double q_z) {
    data.translation() = Eigen::Vector3d(x, y, z);
    data.linear() = Eigen::Quaterniond(q_w, q_x, q_y, q_z).toRotationMatrix();
  }

  Affine operator *(const Affine &a) const {
    Eigen::Isometry3d result;
    result = data * a.data;
    return Affine(result);
  }

  Affine inverse() const {
    return Affine(data.inverse());
  }

  bool isApprox(const Affine &a) const {
    return data.isApprox(a.data);
  }

  void translate(const Eigen::Vector3d &v) {
    data.translate(v);
  }

  void pretranslate(const Eigen::Vector3d &v) {
    data.pretranslate(v);
  }

  Eigen::Vector3d translation() const {
    Eigen::Vector3d v;
    v << data.translation();
    return v;
  }

  double x() const {
    return data.translation().x();
  }

  void setX(double x) {
    data.translation().x() = x;
  }

  double y() const {
    return data.translation().y();
  }

  void setY(double y) {
    data.translation().y() = y;
  }

  double z() const {
    return data.translation().z();
  }

  void setZ(double z) {
    data.translation().z() = z;
  }

  void rotate(const Eigen::Isometry3d::LinearMatrixType &r) {
    data.rotate(r);
  }

  void prerotate(const Eigen::Isometry3d::LinearMatrixType &r) {
    data.prerotate(r);
  }

  Eigen::Isometry3d::LinearMatrixType rotation() const {
    Eigen::Isometry3d::LinearMatrixType result;
    result << data.rotation();
    return result;
  }

  double a() const {
    return get_angles()(0);
  }

  double b() const {
    return get_angles()(1);
  }

  double c() const {
    return get_angles()(2);
  }

  void setA(double a) {
    Eigen::Matrix<double, 3, 1> angles;
    angles << get_angles();
    data.linear() = Euler(a, angles(1), angles(2)).toRotationMatrix();
  }

  void setB(double b) {
    Eigen::Matrix<double, 3, 1> angles;
    angles << get_angles();
    data.linear() = Euler(angles(0), b, angles(2)).toRotationMatrix();
  }

  void setC(double c) {
    Eigen::Matrix<double, 3, 1> angles;
    angles << get_angles();
    data.linear() = Euler(angles(0), angles(1), c).toRotationMatrix();
  }

  Eigen::Quaterniond quaternion() const {
    Eigen::Quaterniond q(data.rotation());
    return q;
  }

  std::array<double, 4> py_quaternion() const {
    auto q = quaternion();
    return {q.x(), q.y(), q.z(), q.w()};
  }

  void setQuaternion(double w, double x, double y, double z) {
    data.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  }

  double qW() const {
    return quaternion().w();
  }

  double qX() const {
    return quaternion().x();
  }

  double qY() const {
    return quaternion().y();
  }

  double qZ() const {
    return quaternion().z();
  }

  Affine getInnerRandom() const {
    std::random_device r;
    std::default_random_engine engine(r());

    Eigen::Matrix<double, 6, 1> max, random;
    max << data.translation(), get_angles();

    for (int i = 0; i < 6; i++) {
      std::uniform_real_distribution<double> distribution(-max(i), max(i));
      random(i) = distribution(engine);
    }

    return Affine(random(0), random(1), random(2), random(3), random(4), random(5));
  }

  std::string toString() const {
    Eigen::Matrix<double, 6, 1> v;
    v << data.translation(), get_angles();

    return "[" + std::to_string(v(0)) + ", " + std::to_string(v(1)) + ", " + std::to_string(v(2))
      + ", " + std::to_string(v(3)) + ", " + std::to_string(v(4)) + ", " + std::to_string(v(5)) + "]";
  }
};

} // namespace affx
