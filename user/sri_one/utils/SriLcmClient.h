//
// Created by han on 24-10-10.
//

#ifndef SRILCMCLIENT_H
#define SRILCMCLIENT_H

#include "string"

// lcm
#include "lcm/lcm-cpp.hpp"
#include "robot_in_t.hpp"
#include "robot_out_t.hpp"

class SriLcmClient {
public:
    SriLcmClient(const std::string& lcm_url, const std::string& lcm_command_channel,
                  const std::string& lcm_status_channel);

    void HandleDataMessage(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const robot_out_t* data);

    int GetLcmFileno() {return lcm_.getFileno();}

    int PollForCommandMessage() {
        return lcm_.handleTimeout(0);
    }

    void Publish();

    robot_in_t lcm_cmd_{};
    robot_out_t lcm_data_{};

private:
    lcm::LCM lcm_;
    std::string lcm_status_channel_;
    std::string lcm_command_channel_;

};



#endif //SRILCMCLIENT_H
