/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */


/* modify the path to reflect the new module names */
#include "torqueScheduler.h"
#include "string.h"
#include <math.h>
#include <stdio.h>


/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_torqueScheduler(torqueSchedulerConfig *configData, int64_t moduleID)
{
    ArrayMotorTorqueMsg_C_init(&configData->motorTorqueOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_torqueScheduler(torqueSchedulerConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input message is included
    if (!ArrayMotorTorqueMsg_C_isLinked(&configData->motorTorque1InMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.motorTorque1InMsg wasn't connected.");
    }
    // check if the required input message is included
    if (!ArrayMotorTorqueMsg_C_isLinked(&configData->motorTorque2InMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.motorTorque2InMsg wasn't connected.");
    }
    // set t0 to current time
    configData->t0 = callTime;
}

/*! This method computes the control torque to the solar array drive based on a PD control law
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_torqueScheduler(torqueSchedulerConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Create buffer messages */
    ArrayMotorTorqueMsgPayload  motorTorque1In;
    ArrayMotorTorqueMsgPayload  motorTorque2In;
    ArrayMotorTorqueMsgPayload  motorTorqueOut;

    /*! - zero the output message */
    motorTorqueOut = ArrayMotorTorqueMsg_C_zeroMsgPayload();

    /*! read the first motor torque message */
    motorTorque1In = ArrayMotorTorqueMsg_C_read(&configData->motorTorque1InMsg);

    /*! read the second motor torque message */
    motorTorque2In = ArrayMotorTorqueMsg_C_read(&configData->motorTorque2InMsg);

    /*! compute current time from Reset call */
    double t;
    t = (double) ((callTime - configData->t0) / 1e9);

    /*! populate output msg */
    motorTorqueOut.motorTorque[0] = motorTorque1In.motorTorque[0];
    motorTorqueOut.motorTorque[1] = motorTorque2In.motorTorque[0];
    
    switch (configData->lockFlag) {

        case 0:
            motorTorqueOut.motorLockFlag[0] = 0;
            motorTorqueOut.motorLockFlag[1] = 0;
            break;

        case 1:
            if (t > configData->tSwitch) {
                motorTorqueOut.motorLockFlag[0] = 1;
                motorTorqueOut.motorLockFlag[1] = 0;
            }
            else {
                motorTorqueOut.motorLockFlag[0] = 0;
                motorTorqueOut.motorLockFlag[1] = 1;
            }
            break;

        case 2:
            if (t > configData->tSwitch) {
                motorTorqueOut.motorLockFlag[0] = 0;
                motorTorqueOut.motorLockFlag[1] = 1;
            }
            else {
                motorTorqueOut.motorLockFlag[0] = 1;
                motorTorqueOut.motorLockFlag[1] = 0;
            }
            break;

        case 3:
            motorTorqueOut.motorLockFlag[0] = 1;
            motorTorqueOut.motorLockFlag[1] = 1;
            break;

        default:
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: torqueScheduler.lockFlag has to be an integer between 0 and 3.");

    }

    /* write output message */
    ArrayMotorTorqueMsg_C_write(&motorTorqueOut, &configData->motorTorqueOutMsg, moduleID, callTime);

    return;
}
