#include "BodyPosTask.hpp"
#include "utilities/Utilities_print.h"
#include "../../utils/orientation_tools.h"


BodyPosTask::BodyPosTask(std::shared_ptr<WBCDataBuffer> wbc_data)
    : Task(3){
  TK::Jt_ = DMat<float>::Zero(TK::dim_task_, 18);
  TK::Jt_.block(0, 3, 3, 3).setIdentity();
  TK::JtDotQdot_ = DVec<float>::Zero(TK::dim_task_);

  _Kp_kin = DVec<float>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<float>::Constant(TK::dim_task_, 50.);
  _Kd = DVec<float>::Constant(TK::dim_task_, 1.0);

  wbc_data_ = wbc_data;
}

BodyPosTask::~BodyPosTask() {}


bool BodyPosTask::_UpdateCommand(const void* pos_des, const DVec<float>& vel_des,
                                    const DVec<float>& acc_des) {
  Vec3<float>* pos_cmd = (Vec3<float>*)pos_des;
  Vec3<float> link_pos = wbc_data_->bodyPosition;

  Quat<float> quat = wbc_data_->bodyOrientation;
  Mat3<float> Rot = ori::quaternionToRotationMatrix(quat);

  SVec<float> curr_vel = wbc_data_->bodyVelocity;
  curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3);

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ((*pos_cmd)[i] - link_pos[i]);
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * ((*pos_cmd)[i] - link_pos[i]) +
                     _Kd[i] * (TK::vel_des_[i] - curr_vel[i + 3]) +
                     TK::acc_des_[i];
  }

  return true;
}

bool BodyPosTask::_UpdateTaskJacobian() {
  Quat<float> quat = wbc_data_->bodyOrientation;
  Mat3<float> Rot = ori::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 3, 3, 3) = Rot.transpose();
  // TK::Jt_.block(0,3, 3,3) = Rot;
  // pretty_print(TK::Jt_, std::cout, "Jt");
  // TK::Jt_.block(0,3, 3,3) = Rot*TK::Jt_.block(0,3,3,3);
  return true;
}

bool BodyPosTask::_UpdateTaskJDotQdot() {
  return true;
}
