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

// Define a class that supports a basic CAN interface that's independent of the hardware and driver library used
// Different libraries can be created to define all these functions for a specific driver library

#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

//C++ Includes
#include <iostream>

//OS Includes
#include <unistd.h>

namespace AS
{
    namespace CAN
    {
        enum return_statuses
        {
            ok = 0,
            init_failed,
            bad_params,
            no_messages_received,
            read_failed,
            send_failed
        };

        class CanInterface
        {
        public:
            CanInterface();

            ~CanInterface();

            // Called to pass in parameters and open can link
            return_statuses open(int hardware_id, int circuit_id, int bitrate);

            // Close the can link
            return_statuses close();

            // Read a message
            return_statuses read(long *id, unsigned char *msg, unsigned int *size, bool *extended, unsigned long *time);

            // Send a message
            return_statuses send(long id, unsigned char *msg, unsigned int size, bool extended);

        private:
            void *handle;
            bool onBus;
        };
    }
}
#endif
