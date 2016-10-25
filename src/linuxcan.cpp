/*
* AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
* Unpublished Copyright (c) 2009-2016 AutonomouStuff, LLC, All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical concepts contained
* herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from COMPANY.  Access to the source code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
*/

#include <as_can_interface.hpp>
#include <canlib.h>

using namespace std;
using namespace AS::CAN;

//Default constructor.
CanInterface::CanInterface() :
        handle(NULL)
{
    handle = malloc(sizeof(canHandle));
}

//Default destructor.
CanInterface::~CanInterface()
{
    free(handle);
}

return_statuses CanInterface::open(int hardware_id, int circuit_id, int bitrate)
{
    if (handle == NULL)
    {
        return init_failed;
    }

    canHandle *h = (canHandle *) handle;

    int numChan;
    if (canGetNumberOfChannels(&numChan) != canOK)
    {
        return init_failed;
    }

    unsigned int serial[2];
    unsigned int channel_number;
    int channel = -1;
    for (int idx = 0; idx < numChan; idx++)
    {
        if (canGetChannelData(idx, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial)) == canOK)
        {
            if (serial[0] == (unsigned int) hardware_id)
            {
                if (canGetChannelData(idx, canCHANNELDATA_CHAN_NO_ON_CARD, &channel_number, sizeof(channel_number)) == canOK)
                {
                    if (channel_number == (unsigned int) circuit_id)
                    {
                        channel = idx;
                    }
                }
            }
        }
    }

    if (channel == -1)
    {
        return bad_params;
    }

    // Open channel
    *h = canOpenChannel(channel, canOPEN_REQUIRE_EXTENDED);
    if (*h < 0)
    {
        return init_failed;
    }

    // Set bit rate and other parameters
    long freq;
    switch (bitrate)
    {
        case 125000: freq = canBITRATE_125K; break;
        case 250000: freq = canBITRATE_250K; break;
        case 500000: freq = canBITRATE_500K; break;
        case 1000000: freq = canBITRATE_1M; break;
        default:
        {
            return  bad_params;
        }
    }

    if (canSetBusParams(*h, freq, 4, 3, 1, 1, 0) < 0)
    {
        return bad_params;
    }

    // Set output control
    canSetBusOutputControl(*h, canDRIVER_NORMAL);

    // Go bus on
    canBusOn(*h);

    return ok;
}

return_statuses CanInterface::close()
{
    if (handle == NULL)
    {
        return init_failed;
    }

    canHandle *h = (canHandle *) handle;

    // Close the channel
    canClose(*h);

    return ok;
}

return_statuses CanInterface::read(long *id, unsigned char *msg, unsigned int *size, bool *extended, unsigned long *time)
{
    if (handle == NULL)
    {
        return init_failed;
    }

    canHandle *h = (canHandle *) handle;

    bool done = false;
    return_statuses ret_val = init_failed;
    unsigned int flag = 0;

    while (!done)
    {
        canStatus ret = canRead(*h, id, msg, size, &flag, time);

        if (ret == canERR_NOMSG)
        {
            ret_val = no_messages_received;
            done = true;
        }
        else if (ret != canOK)
        {
            ret_val = read_failed;
            done = true;
        }
        else if (!(flag & 0xFFF9))
        {
            // Was a received message with actual data
            ret_val = ok;
            done = true;
        }
    }

    if (ret_val == ok)
    {
        *extended = ((flag & canMSG_EXT) > 0);
    }

    return ret_val;
}

return_statuses CanInterface::send(long id, unsigned char *msg, unsigned int size, bool extended)
{
    if (handle == NULL)
    {
        return init_failed;
    }

    canHandle *h = (canHandle *) handle;

    unsigned int flag;

    if (extended)
    {
        flag = canMSG_EXT;
    }
    else
    {
        flag = canMSG_STD;
    }

    canStatus ret = canWrite(*h, id, msg, size, flag);

    return (ret == canOK) ? ok : send_failed;
}
