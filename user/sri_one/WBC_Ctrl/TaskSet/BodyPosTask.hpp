#ifndef BODY_POS_TASK
#define BODY_POS_TASK

// (X, Y, Z)
#include "../../WBC/Task.hpp"
#include "../WBCDataBuffer.h"
#include "memory"


class BodyPosTask : public Task<float> {
 public:
  BodyPosTask(std::shared_ptr<WBCDataBuffer> wbc_data);
  virtual ~BodyPosTask();

  DVec<float> _Kp_kin;
  DVec<float> _Kp, _Kd;

 protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<float>& vel_des,
                              const DVec<float>& acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }
    std::shared_ptr<WBCDataBuffer> wbc_data_;
};

#endif
