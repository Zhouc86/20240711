#include "WBC_Ctrl.hpp"


WBC_Ctrl::WBC_Ctrl(std::shared_ptr<WBCDataBuffer> wbc_data):
  _full_config(18),
  _tau_ff(12),
  _des_jpos(12),
  _des_jvel(12)
{
  _iter = 0;
  _full_config.setZero();

  _kin_wbc = new KinWBC<float>(18);

  _wbic = new WBIC<float>(18, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<float>();

  _wbic_data->_W_floating = DVec<float>::Constant(6, 0.1);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[5] = 0.1;
  _wbic_data->_W_rf = DVec<float>::Constant(12, 1.);

  _Kp_joint.resize(3, 5.);
  _Kd_joint.resize(3, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);

}

WBC_Ctrl::~WBC_Ctrl(){
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<float> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<float> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

void WBC_Ctrl::_ComputeWBC() {
  // TEST
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  //std::cout<<_des_jpos.transpose()<<std::endl;
  //std::cout<<_full_config.transpose()<<std::endl;

  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
  //std::cout<<_tau_ff.transpose()<<std::endl;
}

void WBC_Ctrl::run(std::shared_ptr<WBCDataBuffer> wbc_data){
  ++_iter;

  // Update Model
  _UpdateModel(wbc_data);

  // Task & Contact Update
  _ContactTaskUpdate(wbc_data);

  // WBC Computation
  _ComputeWBC();

  _UpdateLegCMD();
  
  // TEST
  //T dt(0.002);
  //for(size_t i(0); i<12; ++i){
    //_des_jpos[i] = _state.q[i] + _state.qd[i] * dt + 0.5 * _wbic_data->_qddot[i+6] * dt * dt;
    //_des_jvel[i] = _state.qd[i] + _wbic_data->_qddot[i+6]*dt;
  //}

  //_ContactTaskUpdateTEST(input, data);
  //_ComputeWBC();
  // END of TEST

  // Update Leg Command
  //_UpdateLegCMD(wbc_data);

}

void WBC_Ctrl::_UpdateModel(std::shared_ptr<WBCDataBuffer> wbc_data)
{
    _A = wbc_data->A_;
    _grav = wbc_data->grav_;
    _coriolis = wbc_data->coriolis_;
    _Ainv = _A.inverse();
    _full_config = wbc_data->q_;
}

void WBC_Ctrl::_UpdateLegCMD()
{
    for (int i = 0; i < 12; ++i) {
        torque_[i] = _tau_ff(i);
        q_des_[i] = _des_jpos(i);
        dotq_des_[i] = _des_jvel(i);
    }
    for (int i = 0; i < 3; ++i) {
        kp_[i] = _Kp_joint[i];
        kd_[i] = _Kd_joint[i];
    }
}



/*void WBC_Ctrl::_UpdateLegCMD(){
LegControllerCommand<T> * cmd = data._legController->commands;
 //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;

 for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
   cmd[leg].zero();
   for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
     cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
     cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
     cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];

       cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
       cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];

      //if(contact[leg] > 0.){ // Contact
       //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
       //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
     //}else{
       //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint_swing[jidx];
       //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint_swing[jidx];
     //}

   }
 }


 // Knee joint non flip barrier
 for(size_t leg(0); leg<4; ++leg){
   if(cmd[leg].qDes[2] < 0.3){
     cmd[leg].qDes[2] = 0.3;
   }
   if(data._legController->datas[leg].q[2] < 0.3){
     T knee_pos = data._legController->datas[leg].q[2];
     cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
   }
 }
}*/

