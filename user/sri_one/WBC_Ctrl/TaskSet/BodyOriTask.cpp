#include "BodyOriTask.hpp"
// (Rx, Ry, Rz)

#include "../../utils/orientation_tools.h"
#include "utilities/Utilities_print.h"


BodyOriTask::BodyOriTask(std::shared_ptr<WBCDataBuffer> wbc_data)
    : Task<float>(3){
  TK::Jt_ = DMat<float>::Zero(TK::dim_task_, 18);
  TK::Jt_.block(0, 0, 3, 3).setIdentity();
  TK::JtDotQdot_ = DVec<float>::Zero(TK::dim_task_);

  _Kp_kin = DVec<float>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<float>::Constant(TK::dim_task_, 50.);
  _Kd = DVec<float>::Constant(TK::dim_task_, 1.);
  wbc_data_ = wbc_data;
}

BodyOriTask::~BodyOriTask() {}


bool BodyOriTask::_UpdateCommand(const void* pos_des, const DVec<float>& vel_des,
                                    const DVec<float>& acc_des) {
  Quat<float>* ori_cmd = (Quat<float>*)pos_des;
  Quat<float> link_ori = (wbc_data_->bodyOrientation);

  Quat<float> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];
  // link_ori_inv /= link_ori.norm();

  // Explicit because operational space is in global frame
  Quat<float> ori_err = ori::quatProduct(*ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3<float> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);
  SVec<float> curr_vel = wbc_data_->bodyVelocity;

  // Configuration space: Local
  // Operational Space: Global
  Mat3<float> Rot = ori::quaternionToRotationMatrix(link_ori);
  Vec3<float> vel_err = Rot.transpose()*(TK::vel_des_ - curr_vel.head(3));

  // Rx, Ry, Rz
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] +
                     _Kd[i] * vel_err[i] + TK::acc_des_[i];
  }

  return true;
}

bool BodyOriTask::_UpdateTaskJacobian() {
  Quat<float> quat = wbc_data_->bodyOrientation;
  Mat3<float> Rot = ori::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 0, 3, 3) = Rot.transpose();
  //pretty_print(Rot, std::cout, "Rot mat");
  return true;
}

bool BodyOriTask::_UpdateTaskJDotQdot() {
  return true;
}

