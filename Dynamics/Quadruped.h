
#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include <vector>
#include "FloatingBaseModel.h"
#include "SpatialInertia.h"

#include "../third-party/eigen3/Eigen/StdVector"


namespace cheetah {
constexpr size_t num_act_joint = 12;
constexpr size_t num_q = 19;
constexpr size_t dim_config = 18;
constexpr size_t num_leg = 4;
constexpr size_t num_leg_joint = 3;
}  // namespace cheetah


namespace linkID {
constexpr size_t FR = 9;   // Front Right Foot
constexpr size_t FL = 11;  // Front Left Foot
constexpr size_t HR = 13;  // Hind Right Foot
constexpr size_t HL = 15;  // Hind Left Foot

constexpr size_t FR_abd = 2;  // Front Right Abduction
constexpr size_t FL_abd = 0;  // Front Left Abduction
constexpr size_t HR_abd = 3;  // Hind Right Abduction
constexpr size_t HL_abd = 1;  // Hind Left Abduction
}  // namespace linkID

using std::vector;

/*!

 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 
 */
template <typename T>
class Quadruped {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotType _robotType;
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T _abadGearRatio, _hipGearRatio, _kneeGearRatio;
  T _abadLinkLength, _hipLinkLength, _kneeLinkLength, _kneeLinkY_offset, _maxLegLength;

  SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia,
      _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
  Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
      _kneeLocation, _kneeRotorLocation;

  // FloatingBaseModel<T> buildModel();
  // bool buildModel(FloatingBaseModel<T>& model);

  static T getSideSign(int leg) {
    const T sideSigns[4] = {-1, 1, -1, 1};
    assert(leg >= 0 && leg < 4);
    return sideSigns[leg];
  }


  Vec3<T> getHipLocation(int leg) {
    assert(leg >= 0 && leg < 4);
    Vec3<T> pHip((leg == 0 || leg == 1) ? _abadLocation(0) : -_abadLocation(0),
                 (leg == 1 || leg == 3) ? _abadLocation(1) : -_abadLocation(1),
                 _abadLocation(2));
    // std::cout << "hip location : " << pHip << std::endl;
    return pHip;
  }

  float getAbadLength(){
    return _abadLinkLength;
  }

  // void computeLegJacobianAndPosition(Vec3<T>& q, Mat3<T>* J, Vec3<T>* p, int leg){};
};


template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID) {
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID) {
    case 0:
      return Vec3<T>(v[0], -v[1], v[2]);
    case 1:
      return Vec3<T>(v[0], v[1], v[2]);
    case 2:
      return Vec3<T>(-v[0], -v[1], v[2]);
    case 3:
      return Vec3<T>(-v[0], v[1], v[2]);
    default:
      throw std::runtime_error("Invalid leg id!");
  }
}


#endif  // LIBBIOMIMETICS_QUADRUPED_H
