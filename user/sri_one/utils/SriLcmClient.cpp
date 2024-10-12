//
// Created by han on 24-10-10.
//

#include "SriLcmClient.h"


SriLcmClient::SriLcmClient(const std::string &lcm_url, const std::string &lcm_command_channel,
                           const std::string &lcm_status_channel): lcm_(lcm_url),
                                                                   lcm_status_channel_(lcm_status_channel),
lcm_command_channel_(lcm_command_channel)
{
    // init
    lcm_data_.timestamp = -1;
    lcm_cmd_.timestamp = -1;

    // sub ?
    lcm::Subscription *sub = lcm_.subscribe(lcm_status_channel,
                                            &SriLcmClient::HandleDataMessage, this);
    // Only pay attention to the latest command.
    sub->setQueueCapacity(1);
}

void SriLcmClient::HandleDataMessage(const lcm::ReceiveBuffer *rbuf,
                                     const std::string &chan,
                                     const robot_out_t *data) {
    // todo: add safe check

    lcm_data_ = *data;
}

void SriLcmClient::Publish() {
    lcm_.publish(lcm_command_channel_, &lcm_cmd_);
}
