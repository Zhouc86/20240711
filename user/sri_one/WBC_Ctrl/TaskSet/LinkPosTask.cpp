#include "LinkPosTask.hpp"
// (X, Y, Z)
#include "utilities/Utilities_print.h"


LinkPosTask::LinkPosTask(std::shared_ptr<WBCDataBuffer> wbc_data, int link_idx,
                            bool virtual_depend)
    : Task<float>(3),
      link_idx_(link_idx),
      virtual_depend_(virtual_depend) {
  TK::Jt_ = DMat<float>::Zero(TK::dim_task_, 18);
  TK::JtDotQdot_ = DVec<float>::Zero(TK::dim_task_);

  _Kp = DVec<float>::Constant(TK::dim_task_, 100.);
  _Kd = DVec<float>::Constant(TK::dim_task_, 5.);
  _Kp_kin = DVec<float>::Constant(TK::dim_task_, 1.);

  wbc_data_ = wbc_data;
}

LinkPosTask::~LinkPosTask() {}

bool LinkPosTask::_UpdateCommand(const void* pos_des, const DVec<float>& vel_des,
                                    const DVec<float>& acc_des) {
  Vec3<float>* pos_cmd = (Vec3<float>*)pos_des;
  Vec3<float> link_pos;

  link_pos = wbc_data_->foot_position_.block<3, 1>(0, link_idx_);

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i]* ( (*pos_cmd)[i] - link_pos[i] );
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
  }

  Vec3<float> link_vel = wbc_data_->foot_velocity_.block<3, 1>(0, link_idx_);
  // Op acceleration command
  for (size_t i(0); i < TK::dim_task_; ++i) {
    TK::op_cmd_[i] =
        _Kp[i] * TK::pos_err_[i] +
        _Kd[i] * (TK::vel_des_[i] - wbc_data_->foot_velocity_.block<3, 1>(0, link_idx_)(i)) +
        TK::acc_des_[i];
  }
     /*printf("[Link Pos Task]\n");
     pretty_print(acc_des, std::cout, "acc_des");
     pretty_print(TK::pos_err_, std::cout, "pos_err_");
     pretty_print(*pos_cmd, std::cout, "pos cmd");
     pretty_print(link_vel, std::cout, "velocity");
     pretty_print(TK::op_cmd_, std::cout, "op cmd");
     //TK::op_cmd_.setZero();
     pretty_print(TK::Jt_, std::cout, "Jt");*/

  return true;
}

bool LinkPosTask::_UpdateTaskJacobian() {
  TK::Jt_ = wbc_data_->Jc_.block<3, 18>(link_idx_*3, 0);
  if (!virtual_depend_) {
    TK::Jt_.block(0, 0, 3, 6) = DMat<float>::Zero(3, 6);
  }
  return true;
}

bool LinkPosTask::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_ = wbc_data_->JcDotQdot_.block<3, 1>(0, link_idx_);
  return true;
}
