/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <kvaser_interface/kvaser_interface.h>
#include <canstat.h>
#include <gtest/gtest.h>

using AS::CAN::CanMsg;
using AS::CAN::KvaserCan;
using AS::CAN::KvaserCanUtils;
using AS::CAN::ReturnStatuses;

TEST(KvaserCanUtils, getChannels)
{
  int32_t chan_count = -1;
  KvaserCanUtils::getChannelCount(&chan_count);
  ASSERT_EQ(chan_count, 6);

  auto chans = KvaserCanUtils::getChannels();
  ASSERT_EQ(chans.size(), 6);

  auto cards = KvaserCanUtils::getCards();
  ASSERT_EQ(cards.size(), 3);

  chans = KvaserCanUtils::getChannelsOnCard(1);
  ASSERT_EQ(chans.size(), 2);
}

TEST(KvaserCanUtils, setFlags)
{
  // All flags start false
  CanMsg msg;

  // Set all flags in msg to true
  KvaserCanUtils::setMsgFromFlags(&msg, 0xFFFFFFFF);
  ASSERT_EQ(msg.flags.rtr, true);
  ASSERT_EQ(msg.flags.std_id, true);
  ASSERT_EQ(msg.flags.ext_id, true);
  ASSERT_EQ(msg.flags.wakeup_mode, true);
  ASSERT_EQ(msg.flags.error_frame, true);
  ASSERT_EQ(msg.flags.tx_ack, true);
  ASSERT_EQ(msg.flags.tx_rq, true);
  ASSERT_EQ(msg.flags.msg_delayed, true);
  ASSERT_EQ(msg.flags.single_shot, true);
  ASSERT_EQ(msg.flags.tx_nack, true);
  ASSERT_EQ(msg.flags.arb_lost, true);
  ASSERT_EQ(msg.flags.fd_msg, true);
  ASSERT_EQ(msg.flags.fd_bitrate_switch, true);
  ASSERT_EQ(msg.flags.fd_sndr_err_pass_md, true);
  ASSERT_EQ(msg.error_flags.has_err, true);
  ASSERT_EQ(msg.error_flags.hw_overrun_err, true);
  ASSERT_EQ(msg.error_flags.sw_overrun_err, true);
  ASSERT_EQ(msg.error_flags.stuff_err, true);
  ASSERT_EQ(msg.error_flags.form_err, true);
  ASSERT_EQ(msg.error_flags.crc_err, true);
  ASSERT_EQ(msg.error_flags.bit0_err, true);
  ASSERT_EQ(msg.error_flags.bit1_err, true);
  ASSERT_EQ(msg.error_flags.any_overrun_err, true);
  ASSERT_EQ(msg.error_flags.any_bit_err, true);
  ASSERT_EQ(msg.error_flags.any_rx_err, true);

  // Set all flags in msg to false
  KvaserCanUtils::setMsgFromFlags(&msg, 0x00000000);
  ASSERT_EQ(msg.flags.rtr, false);
  ASSERT_EQ(msg.flags.std_id, false);
  ASSERT_EQ(msg.flags.ext_id, false);
  ASSERT_EQ(msg.flags.wakeup_mode, false);
  ASSERT_EQ(msg.flags.error_frame, false);
  ASSERT_EQ(msg.flags.tx_ack, false);
  ASSERT_EQ(msg.flags.tx_rq, false);
  ASSERT_EQ(msg.flags.msg_delayed, false);
  ASSERT_EQ(msg.flags.single_shot, false);
  ASSERT_EQ(msg.flags.tx_nack, false);
  ASSERT_EQ(msg.flags.arb_lost, false);
  ASSERT_EQ(msg.flags.fd_msg, false);
  ASSERT_EQ(msg.flags.fd_bitrate_switch, false);
  ASSERT_EQ(msg.flags.fd_sndr_err_pass_md, false);
  ASSERT_EQ(msg.error_flags.has_err, false);
  ASSERT_EQ(msg.error_flags.hw_overrun_err, false);
  ASSERT_EQ(msg.error_flags.sw_overrun_err, false);
  ASSERT_EQ(msg.error_flags.stuff_err, false);
  ASSERT_EQ(msg.error_flags.form_err, false);
  ASSERT_EQ(msg.error_flags.crc_err, false);
  ASSERT_EQ(msg.error_flags.bit0_err, false);
  ASSERT_EQ(msg.error_flags.bit1_err, false);
  ASSERT_EQ(msg.error_flags.any_overrun_err, false);
  ASSERT_EQ(msg.error_flags.any_bit_err, false);
  ASSERT_EQ(msg.error_flags.any_rx_err, false);

  uint32_t flags = 0;
  // Set all flags in msg to true
  KvaserCanUtils::setMsgFromFlags(&msg, 0xFFFFFFFF);

  // Set all flags in int to true
  KvaserCanUtils::setFlagsFromMsg(msg, &flags);

  // Check information flags
  ASSERT_GT((flags & canMSG_RTR), 0);
  ASSERT_GT((flags & canMSG_STD), 0);
  ASSERT_GT((flags & canMSG_EXT), 0);
  ASSERT_GT((flags & canMSG_WAKEUP), 0);
  ASSERT_GT((flags & canMSG_NERR), 0);
  ASSERT_GT((flags & canMSG_ERROR_FRAME), 0);
  ASSERT_GT((flags & canMSG_TXACK), 0);
  ASSERT_GT((flags & canMSG_TXRQ), 0);
  ASSERT_GT((flags & canMSG_DELAY_MSG), 0);
  ASSERT_GT((flags & canMSG_SINGLE_SHOT), 0);
  ASSERT_GT((flags & canMSG_TXNACK), 0);
  ASSERT_GT((flags & canMSG_ABL), 0);

  // Check CAN FD flags
  ASSERT_GT((flags & canFDMSG_FDF), 0);
  ASSERT_GT((flags & canFDMSG_BRS), 0);
  ASSERT_GT((flags & canFDMSG_ESI), 0);

  // Check error flags
  ASSERT_GT((flags & canMSGERR_HW_OVERRUN), 0);
  ASSERT_GT((flags & canMSGERR_SW_OVERRUN), 0);
  ASSERT_GT((flags & canMSGERR_STUFF), 0);
  ASSERT_GT((flags & canMSGERR_FORM), 0);
  ASSERT_GT((flags & canMSGERR_CRC), 0);
  ASSERT_GT((flags & canMSGERR_BIT0), 0);
  ASSERT_GT((flags & canMSGERR_BIT1), 0);

  // Set all flags in msg to false
  KvaserCanUtils::setMsgFromFlags(&msg, 0x00000000);

  // Set all flags in int to false
  KvaserCanUtils::setFlagsFromMsg(msg, &flags);
  ASSERT_EQ(flags, 0x00000000);
}

void dummyCb()
{
}

TEST(KvaserCan, channelOpenClose)
{
  KvaserCan kv_can;

  // Channel closed, call KvaserCan::open()
  auto stat = kv_can.open(1, 0, 500000, false);
  ASSERT_EQ(stat, ReturnStatuses::OK);
  ASSERT_EQ(kv_can.isOpen(), true);

  // Channel open, call KvaserCan::open()
  stat = kv_can.open(1, 0, 250000, false);
  ASSERT_EQ(stat, ReturnStatuses::OK);

  // Channel open, call KvaserCan::isOpen()
  ASSERT_EQ(kv_can.isOpen(), true);

  // Channel open, call KvaserCan::close()
  stat = kv_can.close();
  ASSERT_EQ(stat, ReturnStatuses::OK);

  // Channel closed, call KvaserCan::close()
  stat = kv_can.close();
  ASSERT_EQ(stat, ReturnStatuses::CHANNEL_CLOSED);

  // Channel closed, call KvaserCan::isOpen()
  ASSERT_EQ(kv_can.isOpen(), false);
}

TEST(KvaserCan, singleChannelTests)
{
  KvaserCan kv_can;
  CanMsg msg;

  auto stat = kv_can.open(0, 500000);
  ASSERT_EQ(stat, ReturnStatuses::OK);

  // Channel open, no messages transmitted, call KvaserCan::read()
  stat = kv_can.read(&msg);
  ASSERT_EQ(stat, ReturnStatuses::NO_MESSAGES_RECEIVED);

  // Channel open, call KvaserCan::registerReadCallback()
  stat = kv_can.registerReadCallback(std::function<void(void)>(dummyCb));
  ASSERT_EQ(stat, ReturnStatuses::OK);

  // Channel open, call KvaserCan::write()
  stat = kv_can.write(std::move(msg));
  ASSERT_EQ(stat, ReturnStatuses::OK);

  stat = kv_can.close();
  ASSERT_EQ(stat, ReturnStatuses::OK);

  // Channel closed, call KvaserCan::read()
  stat = kv_can.read(&msg);
  ASSERT_EQ(stat, ReturnStatuses::CHANNEL_CLOSED);

  // Channel closed, call KvaserCan::registerReadCallback()
  stat = kv_can.registerReadCallback(std::function<void(void)>(dummyCb));
  ASSERT_EQ(stat, ReturnStatuses::CHANNEL_CLOSED);

  // Channel closed, call KvaserCan::write()
  stat = kv_can.write(std::move(msg));
  ASSERT_EQ(stat, ReturnStatuses::CHANNEL_CLOSED);
}

TEST(KvaserCan, readWriteTests)
{
  KvaserCan kv_can_writer;
  KvaserCan kv_can_reader;
  CanMsg sent_msg;
  CanMsg rcvd_msg;

  auto writer_stat = kv_can_writer.open(1, 500000);
  auto reader_stat = kv_can_reader.open(0, 500000);

  ASSERT_EQ(writer_stat, ReturnStatuses::OK);
  ASSERT_EQ(reader_stat, ReturnStatuses::OK);

  // Populate message data
  for (auto i = 0; i < 4; ++i)
  {
    sent_msg.data.push_back(0);
    sent_msg.data.push_back(1);
  }

  // Send standard CAN ID with std flag true and extended flag false
  sent_msg.id = 0x555;
  sent_msg.dlc = 8;
  sent_msg.flags.std_id = true;
  sent_msg.flags.ext_id = false;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);

  // Receive standard CAN ID with std flag true and extended flag false
  kv_can_reader.read(&rcvd_msg);
  ASSERT_EQ(reader_stat, ReturnStatuses::OK);

  // Check that sent and received messages are identical
  ASSERT_EQ(sent_msg, rcvd_msg);

  // Send extended CAN ID with std flag false and extended flag true
  sent_msg.id = 0x15555555;
  sent_msg.dlc = 8;
  sent_msg.flags.std_id = false;
  sent_msg.flags.ext_id = true;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);

  // Receive extended CAN ID with std flag false and extended flag true
  kv_can_reader.read(&rcvd_msg);
  ASSERT_EQ(reader_stat, ReturnStatuses::OK);

  // Check that sent and received messages are identical
  ASSERT_EQ(sent_msg, rcvd_msg);

  writer_stat = kv_can_writer.close();
  reader_stat = kv_can_reader.close();

  ASSERT_EQ(writer_stat, ReturnStatuses::OK);
  ASSERT_EQ(reader_stat, ReturnStatuses::OK);
}

TEST(KvaserCan, writeTests)
{
  KvaserCan kv_can_writer;
  CanMsg sent_msg;

  auto writer_stat = kv_can_writer.open(0, 500000);
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);

  // Send standard CAN ID with 0 DLC and 0 payload bytes
  sent_msg.id = 0x555;
  sent_msg.dlc = 0;
  sent_msg.data.clear();
  sent_msg.flags.std_id = true;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);

  // Send standard CAN ID with 8 DLC and 0 payload bytes
  sent_msg.id = 0x555;
  sent_msg.dlc = 8;
  sent_msg.data.clear();
  sent_msg.flags.std_id = true;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::DLC_PAYLOAD_MISMATCH);

  // Populate message data
  for (auto i = 0; i < 4; ++i)
  {
    sent_msg.data.push_back(0);
    sent_msg.data.push_back(1);
  }

  // Send standard CAN ID with std flag true and extended flag true
  sent_msg.id = 0x555;
  sent_msg.dlc = 8;
  sent_msg.flags.std_id = true;
  sent_msg.flags.ext_id = true;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);

  // Send extended CAN ID with 0 DLC and 0 payload bytes
  sent_msg.id = 0x15555555;
  sent_msg.dlc = 0;
  sent_msg.data.clear();
  sent_msg.flags.std_id = false;
  sent_msg.flags.ext_id = true;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);

  // Populate message data
  for (auto i = 0; i < 4; ++i)
  {
    sent_msg.data.push_back(0);
    sent_msg.data.push_back(1);
  }

  // Send extended CAN ID with extended flag false and std flag false
  sent_msg.id = 0x15555555;
  sent_msg.dlc = 8;
  sent_msg.flags.ext_id = false;
  sent_msg.flags.std_id = false;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::WRITE_FAILED);

  // Send extended CAN ID with extended flag false and std flag true
  sent_msg.id = 0x15555555;
  sent_msg.dlc = 8;
  sent_msg.flags.std_id = true;
  sent_msg.flags.ext_id = false;
  writer_stat = kv_can_writer.write(CanMsg(sent_msg));
  ASSERT_EQ(writer_stat, ReturnStatuses::WRITE_FAILED);

  writer_stat = kv_can_writer.close();
  ASSERT_EQ(writer_stat, ReturnStatuses::OK);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
