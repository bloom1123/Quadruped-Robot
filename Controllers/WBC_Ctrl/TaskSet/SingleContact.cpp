#include "SingleContact.hpp"

// [ Fx, Fy, Fz ]
template <typename T>
SingleContact<T>::SingleContact(const FloatingBaseModel<T>* robot, int pt)
    : ContactSpec<T>(3), _max_Fz(1500.), _muv(0.4), _contact_pt(pt), _dim_U(6) {
  Contact::idx_Fz_ = 2;
  robot_sys_ = robot;
  Contact::Jc_ = DMat<T>(Contact::dim_contact_, cheetah::dim_config);
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
  Contact::Uf_(0, 2) = 1.;
  Contact::Uf_(1, 0) = 1.;
  Contact::Uf_(1, 2) = _muv;
  Contact::Uf_(2, 0) = -1.;
  Contact::Uf_(2, 2) = _muv;
  Contact::Uf_(3, 1) = 1.;
  Contact::Uf_(3, 2) = _muv;
  Contact::Uf_(4, 1) = -1.;
  Contact::Uf_(4, 2) = _muv;
  // Upper bound of normal force
  Contact::Uf_(5, 2) = -1.;
}

template <typename T>
SingleContact<T>::~SingleContact() {}

template <typename T>
bool SingleContact<T>::_UpdateJc() {
  Contact::Jc_ = robot_sys_->_Jc[_contact_pt];

  return true;
}

template <typename T>
bool SingleContact<T>::_UpdateJcDotQdot() {
  Contact::JcDotQdot_ = robot_sys_->_Jcdqd[_contact_pt];
  return true;
}

template <typename T>
bool SingleContact<T>::_UpdateUf() {
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
  Contact::Uf_(0, 2) = 1.;
  Contact::Uf_(1, 0) = 1.;
  Contact::Uf_(1, 2) = _muv;
  Contact::Uf_(2, 0) = -1.;
  Contact::Uf_(2, 2) = _muv;
  Contact::Uf_(3, 1) = 1.;
  Contact::Uf_(3, 2) = _muv;
  Contact::Uf_(4, 1) = -1.;
  Contact::Uf_(4, 2) = _muv;
  // Upper bound of normal force
  Contact::Uf_(5, 2) = -1.;
  Contact::Uf_=Contact::Uf_*_Rpla.transpose();

  return true;
}

template <typename T>
bool SingleContact<T>::_UpdateInequalityVector() {
  Contact::ieq_vec_ = DVec<T>::Zero(_dim_U);
  Contact::ieq_vec_[5] = -_max_Fz;
  return true;
}

template class SingleContact<double>;
template class SingleContact<float>;
