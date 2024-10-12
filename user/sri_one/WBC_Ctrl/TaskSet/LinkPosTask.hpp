#ifndef LINK_POS_TASK
#define LINK_POS_TASK

// (X, Y, Z)
#include "../../WBC/Task.hpp"
#include "../WBCDataBuffer.h"
#include "memory"


class LinkPosTask : public Task<float> {
 public:
  LinkPosTask( std::shared_ptr<WBCDataBuffer> wbc_data, int link_idx,
              bool virtual_depend = true);
  virtual ~LinkPosTask();

  DVec<float> _Kp, _Kd, _Kp_kin;

 protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<float>& vel_des,
                              const DVec<float>& acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }

  int link_idx_;
  bool virtual_depend_;
    std::shared_ptr<WBCDataBuffer> wbc_data_;
};

#endif
