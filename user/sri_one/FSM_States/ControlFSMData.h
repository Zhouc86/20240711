//
// Created by han on 23-2-15.
//

#ifndef BAER_ETHERCAT_CONTROLFSMDATA_H
#define BAER_ETHERCAT_CONTROLFSMDATA_H

#include "memory"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "../cmpc/cmpc_databuffer.h"
#include "../utils/sri_kin.h"

#include "../utils/PositionVelocityEstimator.h"

struct ControlFSMData {
    ControlFSMData() : position_(19), velocity_(18){
        plant_ = std::make_shared<drake::multibody::MultibodyPlant<double>>(0.0);
        drake::multibody::Parser parser(plant_.get(), nullptr);
        parser.AddModels("sri.urdf");
        plant_->Finalize();
        context_ = plant_->CreateDefaultContext();
        sri_kin_ = std::make_shared<SriKin>();

        //todo: add confirm file
        qpos_init_[0] = 0.242495;
        qpos_init_[1] = -0.545462;
        qpos_init_[2] = 1.36813;
        qpos_init_[3] = -0.242495;
        qpos_init_[4] = -0.545462;
        qpos_init_[5] = 1.36813;
        qpos_init_[6] = 0.242495;
        qpos_init_[7] = -0.545462;
        qpos_init_[8] = 1.36813;
        qpos_init_[9] = -0.242495;
        qpos_init_[10] = -0.545462;
        qpos_init_[11] = 1.36813;

        // init
        pos_vel_estimator_ = std::make_shared<LinearKFPositionVelocityEstimator>();
        state_estimator_data_ = std::make_shared<StateEstimatorData>();
    }

    int state_no_{0};

    double begin_time_{0.0};
    double time_{0.0};

    // using drake
    std::shared_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    std::unique_ptr<drake::systems::Context<double>> context_;

    std::shared_ptr<SriKin> sri_kin_;

    // WBIC
    CMPCDataBuffer data_buffer_;

    double roll_compensate_ = 0;
    double pitch_compensate_ = 0;
    double x_comp_integral_ = 0;

    Eigen::VectorXd position_;
    Eigen::VectorXd velocity_;

    //output
    double motor_torque_[12];
    double kp_[12];
    double kd_[12];
    double q_des_[12];
    double dq_des_[12];
    double tau_[12];

    double qpos_init_[12];

    // for position velocity estimator
    std::shared_ptr<LinearKFPositionVelocityEstimator> pos_vel_estimator_;
    std::shared_ptr<StateEstimatorData> state_estimator_data_;

    bool is_state_estimator_init_{false};

};


#endif //BAER_ETHERCAT_CONTROLFSMDATA_H
