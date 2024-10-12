#include "SingleContact.hpp"
#include "utilities/Utilities_print.h"

// [ Fx, Fy, Fz ]

SingleContact::SingleContact(std::shared_ptr<WBCDataBuffer> wbc_data, int pt)
    : ContactSpec<float>(3), _max_Fz(1500.), _contact_pt(pt), _dim_U(6) {
  Contact::idx_Fz_ = 2;
  Contact::Jc_ = DMat<float>(Contact::dim_contact_, 18);
  Contact::JcDotQdot_ = DVec<float>::Zero(Contact::dim_contact_);
  Contact::Uf_ = DMat<float>::Zero(_dim_U, Contact::dim_contact_);

  float mu(0.4);

  Contact::Uf_(0, 2) = 1.;

  Contact::Uf_(1, 0) = 1.;
  Contact::Uf_(1, 2) = mu;
  Contact::Uf_(2, 0) = -1.;
  Contact::Uf_(2, 2) = mu;

  Contact::Uf_(3, 1) = 1.;
  Contact::Uf_(3, 2) = mu;
  Contact::Uf_(4, 1) = -1.;
  Contact::Uf_(4, 2) = mu;

  // Upper bound of normal force
  Contact::Uf_(5, 2) = -1.;

  wbc_data_ = wbc_data;
}

SingleContact::~SingleContact() {}

bool SingleContact::_UpdateJc() {
  Contact::Jc_ = wbc_data_->Jc_.block<3, 18>(_contact_pt*3, 0);
  return true;
}

bool SingleContact::_UpdateJcDotQdot() {
    Contact::JcDotQdot_ = wbc_data_->JcDotQdot_.block<3, 1>(0, _contact_pt);
  return true;
}

bool SingleContact::_UpdateUf() {
  return true;
}

bool SingleContact::_UpdateInequalityVector() {
  Contact::ieq_vec_ = DVec<float>::Zero(_dim_U);
  Contact::ieq_vec_[5] = -_max_Fz;
  return true;
}

