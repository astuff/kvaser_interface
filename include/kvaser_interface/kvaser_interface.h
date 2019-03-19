/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef KVASER_INTERFACE_KVASER_INTERFACE_H
#define KVASER_INTERFACE_KVASER_INTERFACE_H

extern "C"
{
#include <canlib.h>
}

// C++ Includes
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace AS
{
namespace CAN
{

enum class ReturnStatuses
{
  OK = 0,
  INIT_FAILED = -1,
  BAD_PARAM = -2,
  NO_CHANNELS_FOUND = -3,
  CHANNEL_CLOSED = -4,
  NO_MESSAGES_RECEIVED = -5,
  READ_FAILED = -6,
  WRITE_FAILED = -7,
  CLOSE_FAILED = -8
};

enum class HardwareType
{
  NONE = 0,
  VIRTUAL = 1,
  LAPCAN = 2,
  CANPARI = 3,
  PCCAN = 8,
  PCICAN = 9,
  USBCAN = 11,
  PCICAN_II = 40,
  HWTYPE_USBCAN_II = 42,
  HWTYPE_SIMULATED = 44,
  HWTYPE_ACQUISITOR = 46,
  HWTYPE_LEAF = 48,
  HWTYPE_PC104_PLUS = 50,
  HWTYPE_PCICANX_II = 52,
  HWTYPE_MEMORATOR_II = 54,
  HWTYPE_MEMORATOR_PRO = 54,
  HWTYPE_USBCAN_PRO = 56,
  HWTYPE_IRIS = 58,
  HWTYPE_BLACKBIRD = 58,
  HWTYPE_MEMORATOR_LIGHT = 60,
  HWTYPE_MINIHYDRA = 62,
  HWTYPE_EAGLE = 62,
  HWTYPE_BAGEL = 64,
  HWTYPE_BLACKBIRD_V2 = 64,
  HWTYPE_MINIPCIE = 66,
  HWTYPE_USBCAN_KLINE = 68,
  HWTYPE_ETHERCAN = 70,
  HWTYPE_USBCAN_LIGHT = 72,
  HWTYPE_USBCAN_PRO2 = 74,
  HWTYPE_PCIE_V2 = 76,
  HWTYPE_MEMORATOR_PRO2 = 78,
  HWTYPE_LEAF2 = 80,
  HWTYPE_MEMORATOR_V2 = 82,
  HWTYPE_CANLINHYBRID = 84
};

struct MsgFlags
{
  bool rtr;
  bool std_id;
  bool ext_id;
  bool wakeup_mode;
  bool error_frame;
  bool tx_ack;
  bool tx_rq;
  bool msg_delayed;
  bool single_shot;
  bool tx_nack;
  bool arb_lost;
  bool fd_msg;
  bool fd_bitrate_switch;
  bool fd_sndr_err_pass_md;
};

struct MsgErrFlags
{
  bool has_err;
  bool hw_overrun_err;
  bool sw_overrun_err;
  bool stuff_err;
  bool form_err;
  bool crc_err;
  bool bit0_err;
  bool bit1_err;
  bool any_overrun_err;
  bool any_bit_err;
  bool any_rx_err;
};

struct CanMsg
{
  uint32_t id;
  uint32_t dlc;
  MsgFlags flags;
  MsgErrFlags error_flags;
  std::vector<uint8_t> data;
  uint64_t timestamp;
};

class KvaserCard
{
  public:
    uint64_t serial_no = 0;
    uint16_t firmware_rev_maj = 0;
    uint16_t firmware_rev_min = 0;
    uint16_t firmware_rev_rel = 0;
    uint16_t firmware_rev_bld = 0;
    HardwareType hw_type = HardwareType::NONE;
    std::string dev_name;
    std::string upc_no;
    std::string driver_name;
    uint16_t driver_ver_maj = 0;
    uint16_t driver_ver_min = 0;
    uint16_t driver_ver_bld = 0;
    bool all_data_valid = true;
};

class KvaserChannel
: public KvaserCard
{
  public:
    KvaserChannel()
    : KvaserCard()
    {}

    uint32_t channel_idx = 0;
    uint32_t channel_no_on_card = 0;
    uint32_t max_bitrate = 0;
};

class KvaserCan
{
  public:
    KvaserCan();

    // Moving and copying not allowed because of
    // state associated with handle.
    KvaserCan(KvaserCan && p) = delete;
    KvaserCan &operator=(KvaserCan && p) = delete;
    KvaserCan(const KvaserCan &) = delete;
    KvaserCan &operator=(const KvaserCan &) = delete;

    ~KvaserCan();

    // Called to pass in parameters and open can link
    ReturnStatuses open(const uint64_t &hardware_id,
                        const uint32_t &circuit_id,
                        const uint32_t &bitrate,
                        const bool &echo_on = true);

    ReturnStatuses open(const uint32_t &channel_index,
                        const uint32_t &bitrate,
                        const bool &echo_on = true);

    // Close the can link
    ReturnStatuses close();

    // Check to see if the CAN link is open
    bool isOpen();

    // Read a message
    ReturnStatuses read(CanMsg *msg);

    // Send a message
    ReturnStatuses write(const uint32_t &id,
                         uint8_t *msg,
                         const uint32_t &size,
                         const bool &extended);

  private:
    std::unique_ptr<CanHandle> handle;
    bool on_bus;
};

class KvaserCanUtils
{
  public:
    static ReturnStatuses canlibStatToReturnStatus(const int32_t &canlibStat);
    static size_t dlcToSize(const uint8_t &dlc);
    static uint8_t sizeToDlc(const size_t &size);
    static void getChannelCount(int32_t *numChannels);
    static std::vector<std::shared_ptr<KvaserCard>> getCards();
    static std::vector<std::shared_ptr<KvaserChannel>> getChannels();
    static std::vector<std::shared_ptr<KvaserChannel>> getChannelsOnCard(const uint64_t &serialNo);
    static std::string returnStatusDesc(const ReturnStatuses &ret);
    static void setMsgFlags(CanMsg *msg, const uint32_t &flags);
};

}  // namespace CAN
}  // namespace AS

#endif  // KVASER_INTERFACE_KVASER_INTERFACE_H
