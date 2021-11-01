/*! @file Quadruped.cpp
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the Quadruped class.  This stores all the parameters for
 * a quadruped robot.  There are utility functions to generate Quadruped objects
 * for Cheetah 3 (and eventually mini-cheetah). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the quadruped.
 */

#include "Quadruped.h"
#include "spatial.h"
#include "../Math/orientation_tools.h"

using namespace ori;
using namespace spatial;

/*!
 * Build a FloatingBaseModel of the quadruped
 */
// template <typename T>
// bool Quadruped<T>::buildModel(FloatingBaseModel<T>& model) {

//   Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
//   model.addBase(_bodyInertia);
//   model.addGroundContactBoxPoints(5, bodyDims);

//   const int baseID = 5;
//   int bodyID = baseID;
//   T sideSign = -1;

//   Mat3<T> I3 = Mat3<T>::Identity();

//   // loop over 4 legs
//   for (int legID = 0; legID < 4; legID++) {
//     // std::cout << "--- leg id: " << legID << "start 0 " << std::endl;
//     bodyID++;
//     Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, legID));
//     Mat6<T> xtreeAbadRotor = createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));

//     // std::cout << "--- leg id: " << legID << "start 1 " << std::endl;
//     if (sideSign < 0) {

//       SpatialInertia<T> temp1 = _abadInertia.flipAlongAxis(CoordinateAxis::Y);
//       // std::cout << "--- leg id -1 : " << legID << "start 2 " << std::endl;
//       model.addBody(temp1,
//                     _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
//                     _abadGearRatio, baseID, JointType::Revolute,
//                     CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
//     // std::cout << "--- leg id: -1 " << legID << "start 3 " << std::endl;
//     } else {
//       // std::cout << "--- leg id 1 : " << legID << "start 2 " << std::endl;
//       model.addBody(_abadInertia, _abadRotorInertia, _abadGearRatio, baseID,
//                     JointType::Revolute, CoordinateAxis::X, xtreeAbad,
//                     xtreeAbadRotor);
//       // std::cout << "--- leg id 1 : " << legID << "start 3 " << std::endl;
//     }

//     // Hip Joint
//     bodyID++;
//     Mat6<T> xtreeHip =
//         createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
//                      withLegSigns<T>(_hipLocation, legID));

//     Mat6<T> xtreeHipRotor =
//         createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
//                      withLegSigns<T>(_hipRotorLocation, legID));
//     if (sideSign < 0) {
//       model.addBody(_hipInertia.flipAlongAxis(CoordinateAxis::Y),
//                     _hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
//                     _hipGearRatio, bodyID - 1, JointType::Revolute,
//                     CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
//     } else {
//       model.addBody(_hipInertia, _hipRotorInertia, _hipGearRatio, bodyID - 1,
//                     JointType::Revolute, CoordinateAxis::Y, xtreeHip,
//                     xtreeHipRotor);
//     }

    
//     // add knee ground contact point
//     model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_hipLinkLength));

//     // Knee Joint
//     bodyID++;
//     Mat6<T> xtreeKnee = createSXform(I3, _kneeLocation);
//     Mat6<T> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
//     if (sideSign < 0) {
//       model.addBody(_kneeInertia.flipAlongAxis(CoordinateAxis::Y),
//                     _kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
//                     _kneeGearRatio, bodyID - 1, JointType::Revolute,
//                     CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);

//       model.addGroundContactPoint(bodyID, Vec3<T>(0, _kneeLinkY_offset, -_kneeLinkLength), true);
//     } else {
//       model.addBody(_kneeInertia, _kneeRotorInertia, _kneeGearRatio, bodyID - 1,
//                     JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
//                     xtreeKneeRotor);

//       model.addGroundContactPoint(bodyID, Vec3<T>(0, -_kneeLinkY_offset, -_kneeLinkLength), true);
//     }

//     // add foot
//     //model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

//     sideSign *= -1;
//   }

//   Vec3<T> g(0, 0, -9.81);
//   model.setGravity(g);

//   return true;
// }

/*!
 * Build a FloatingBaseModel of the quadruped
 */
// template <typename T>
// FloatingBaseModel<T> Quadruped<T>::buildModel() {
//   FloatingBaseModel<T> model;
//   buildModel(model);
//   return model;
// }



template class Quadruped<double>;
template class Quadruped<float>;
