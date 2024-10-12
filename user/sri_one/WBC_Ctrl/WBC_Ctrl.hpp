#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include "cppTypes.h"
#include "../WBC/WBIC/WBIC.hpp"
#include "../WBC/WBIC/KinWBC.hpp"

#include "WBCDataBuffer.h"
#include "memory"

#define WBCtrl WBC_Ctrl


class WBC_Ctrl{
  public:
    WBC_Ctrl(std::shared_ptr<WBCDataBuffer> wbc_data);
    virtual ~WBC_Ctrl();

    void run(std::shared_ptr<WBCDataBuffer> wbc_data);
    void setFloatingBaseWeight(const float & weight){
      _wbic_data->_W_floating = DVec<float>::Constant(6, weight);
    }

    double torque_[12];
    double kp_[3];
    double kd_[3];
    double q_des_[12];
    double dotq_des_[12];


    DVec<float> _full_config;
    DVec<float> _tau_ff;
    DVec<float> _des_jpos;
    DVec<float> _des_jvel;

  protected:
    virtual void _ContactTaskUpdate(std::shared_ptr<WBCDataBuffer> wbc_data) = 0;
    void _ComputeWBC();
    void _UpdateModel(std::shared_ptr<WBCDataBuffer> wbc_data);
    void _UpdateLegCMD();

    KinWBC<float>* _kin_wbc;
    WBIC<float>* _wbic;
    WBIC_ExtraData<float>* _wbic_data;

    std::vector<ContactSpec<float> * > _contact_list;
    std::vector<Task<float> * > _task_list;

    DMat<float> _A;
    DMat<float> _Ainv;
    DVec<float> _grav;
    DVec<float> _coriolis;

    std::vector<float> _Kp_joint, _Kd_joint;
    //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

    unsigned long long _iter;

};
#endif
