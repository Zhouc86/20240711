#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER


#include "../WBC_Ctrl.hpp"


class LocomotionCtrl: public WBC_Ctrl{
  public:
    LocomotionCtrl(std::shared_ptr<WBCDataBuffer> wbc_data);
    virtual ~LocomotionCtrl();

  protected:
    virtual void _ContactTaskUpdate(std::shared_ptr<WBCDataBuffer> wbc_data);
    //virtual void _ContactTaskUpdateTEST(void * input, ControlFSMData<T> & data);
    //void _ParameterSetup(const MIT_UserParameters* param);
    void _CleanUp();
    //virtual void _LCM_PublishData();

    //LocomotionCtrlData<T>* _input_data;

    Task<float>* _body_pos_task;
    Task<float>* _body_ori_task;

    Task<float>* _foot_task[4];
    ContactSpec<float>* _foot_contact[4];

    Vec3<float> pre_foot_vel[4];

    Vec3<float> _Fr_result[4];
    Quat<float> _quat_des;
};

#endif

