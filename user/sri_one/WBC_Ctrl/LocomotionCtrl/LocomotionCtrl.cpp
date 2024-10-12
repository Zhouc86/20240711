#include "LocomotionCtrl.hpp"
#include "../ContactSet/SingleContact.hpp"
#include "../TaskSet/BodyOriTask.hpp"
#include "../TaskSet/BodyPosTask.hpp"
#include "../TaskSet/LinkPosTask.hpp"
#include "../../utils/orientation_tools.h"

LocomotionCtrl::LocomotionCtrl(std::shared_ptr<WBCDataBuffer> wbc_data):
  WBC_Ctrl(wbc_data)
{
  _body_pos_task = new BodyPosTask(wbc_data);
  _body_ori_task = new BodyOriTask(wbc_data);


  _foot_contact[0] = new SingleContact(wbc_data, 0);
  _foot_contact[1] = new SingleContact(wbc_data, 1);
  _foot_contact[2] = new SingleContact(wbc_data, 2);
  _foot_contact[3] = new SingleContact(wbc_data, 3);

  _foot_task[0] = new LinkPosTask(wbc_data, 0);
  _foot_task[1] = new LinkPosTask(wbc_data, 1);
  _foot_task[2] = new LinkPosTask(wbc_data, 2);
  _foot_task[3] = new LinkPosTask(wbc_data, 3);

    for(size_t i(0); i<3; ++i){
        ((BodyPosTask*)_body_pos_task)->_Kp[i] = 10;
        ((BodyPosTask*)_body_pos_task)->_Kd[i] = 3;

        ((BodyOriTask*)_body_ori_task)->_Kp[i] = 10;
        ((BodyOriTask*)_body_ori_task)->_Kd[i] = 3;

        for(size_t j(0); j<4; ++j){
            ((LinkPosTask*)_foot_task[j])->_Kp[i] = 70;
            ((LinkPosTask*)_foot_task[j])->_Kd[i] = 3;
            //((LinkPosTask<T>*)_foot_task[j])->_Kp_kin[i] = 1.5;
        }
        //WBCtrl::_Kp_joint_swing[i] = param->Kp_joint_swing[i];
        //WBCtrl::_Kd_joint_swing[i] = param->Kd_joint_swing[i];
    }

    WBCtrl::_Kp_joint[0] = 1;
}

LocomotionCtrl::~LocomotionCtrl(){
  delete _body_pos_task;
  delete _body_ori_task;

  for(size_t i (0); i<4; ++i){
    delete _foot_contact[i];
    delete _foot_task[i];
  }
}

void LocomotionCtrl::_ContactTaskUpdate(std::shared_ptr<WBCDataBuffer> wbc_data){
  //_input_data = static_cast<LocomotionCtrlData<T>* >(input);

  //_ParameterSetup(data.userParameters);
  
  // Wash out the previous setup
  _CleanUp();

  _quat_des = ori::rpyToQuat(wbc_data->robot_des_info_.pBody_RPY_des);

  Vec3<float> zero_vec3; zero_vec3.setZero();
  _body_ori_task->UpdateTask(&_quat_des, wbc_data->robot_des_info_.vBody_Ori_des, zero_vec3);
  _body_pos_task->UpdateTask(&(wbc_data->robot_des_info_.pBody_des), wbc_data->robot_des_info_.vBody_des, wbc_data->robot_des_info_.aBody_des);

  WBCtrl::_task_list.push_back(_body_ori_task);
  WBCtrl::_task_list.push_back(_body_pos_task);

  for(size_t leg(0); leg<4; ++leg){
    if(wbc_data->robot_des_info_.contact_state[leg] > 0.1){ // Contact
      _foot_contact[leg]->setRFDesired((DVec<float>)(wbc_data->robot_des_info_.Fr_des[leg]));
      _foot_contact[leg]->UpdateContactSpec();
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);
    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(&(wbc_data->robot_des_info_.pFoot_des[leg]), wbc_data->robot_des_info_.vFoot_des[leg],
                                  wbc_data->robot_des_info_.aFoot_des[leg]);
          //zero_vec3);
      WBCtrl::_task_list.push_back(_foot_task[leg]);
    }
  }
}

/*template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdateTEST(std::shared_ptr<WBCDataBuffer> wbc_data){
  (void)data;
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);

  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = 10.;
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = 3.;

    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = 10.;
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = 3.;

    for(size_t j(0); j<4; ++j){
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = 70;
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = 3.;
    }  
  }
  // Wash out the previous setup
  _CleanUp();

  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);

  Vec3<T> zero_vec3; zero_vec3.setZero();
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
  _body_pos_task->UpdateTask(
      &(_input_data->pBody_des), 
      _input_data->vBody_des, 
      _input_data->aBody_des);

  WBCtrl::_task_list.push_back(_body_ori_task);
  WBCtrl::_task_list.push_back(_body_pos_task);

  for(size_t leg(0); leg<4; ++leg){
    if(_input_data->contact_state[leg] > 0.){ // Contact
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));
      _foot_contact[leg]->UpdateContactSpec();
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);

    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(
          &(_input_data->pFoot_des[leg]), 
          _input_data->vFoot_des[leg], 
          _input_data->aFoot_des[leg]);
          //zero_vec3);
      WBCtrl::_task_list.push_back(_foot_task[leg]);
    }
  }
}*/

/*template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const MIT_UserParameters* param){

  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];

    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];

    for(size_t j(0); j<4; ++j){
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->Kd_foot[i];
      //((LinkPosTask<T>*)_foot_task[j])->_Kp_kin[i] = 1.5;
    }

    WBCtrl::_Kp_joint[i] = param->Kp_joint[i];
    WBCtrl::_Kd_joint[i] = param->Kd_joint[i];

    //WBCtrl::_Kp_joint_swing[i] = param->Kp_joint_swing[i];
    //WBCtrl::_Kd_joint_swing[i] = param->Kd_joint_swing[i];
   }
}*/


void LocomotionCtrl::_CleanUp(){
  WBCtrl::_contact_list.clear();
  WBCtrl::_task_list.clear();
}