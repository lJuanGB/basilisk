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

/* Modify the path to reflect the new module names */
#include "prescribed1DOF.h"
#include "string.h"
#include <math.h>
#include <stdlib.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_prescribed1DOF(Prescribed1DOFConfig *configData, int64_t moduleID)
{
    PrescribedMotionMsg_C_init(&configData->prescribedMotionOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_prescribed1DOF(Prescribed1DOFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! Check if the required input messages are included */
    if (!RefAngleMsg_C_isLinked(&configData->refAngleInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribed1DOF.refAngleInMsg wasn't connected.");
    }

    if (!CurrAngleMsg_C_isLinked(&configData->currAngleInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribed1DOF.currAngleInMsg wasn't connected.");
    }

    /*! Store initial time */
    configData->t0 = callTime*1e-9; // [s]
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_prescribed1DOF(Prescribed1DOFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! Create buffer messages */
    RefAngleMsgPayload refAngleIn;
    CurrAngleMsgPayload currAngleIn;
    PrescribedMotionMsgPayload prescribedMotionOut;

    /*! Zero the output message */
    prescribedMotionOut = PrescribedMotionMsg_C_zeroMsgPayload();

    /*! Read the input messages */
    refAngleIn = RefAngleMsg_C_read(&configData->refAngleInMsg); // [rad]
    currAngleIn = CurrAngleMsg_C_read(&configData->currAngleInMsg); // [rad]

    /*! Define initial variables */
    double theta0 = currAngleIn.thetaCurr; // [rad]
    double thetaDot0 = currAngleIn.thetaDotCurr; // [rad/s]

    /*! Grab reference variables */
    double thetaRef = refAngleIn.thetaRef; // [rad]
    double thetaDotRef = refAngleIn.thetaDotRef; // [rad/s]

    /*! Grab other module variables */
    double thetaDDotMax = configData->thetaDDotMax;
    int spinAxis = configData->spinAxis;

    /*! Define temporal information */
    double t0 = configData->t0;
    double tf = sqrt(((0.5 * abs(thetaRef - theta0)) * 8) / thetaDDotMax); // [s] ADD BACK abs(thetaRef-theta0) AFTER CODE COMPILES
    double ts = tf / 2; // switch time [s]
    double t = callTime*1e-9; // current time [s]

    /*! Define scalar module states */
    double thetaDDot;
    double thetaDot;
    double theta;

    // Define constants for analytic parabolas
    double a = (0.5 * thetaRef - theta0) / ((ts - t0) * (ts - t0)); // Constant for first parabola
    double b = -0.5 * thetaRef / ((ts - tf) * (ts - tf)); // Constant for second parabola

    /*! Compute analytic scalar states: thetaDDot, thetaDot, and theta */
    if (t < ts || t == ts)
    {
        thetaDDot = thetaDDotMax;
        thetaDot = thetaDDot * (t - t0) + thetaDot0;
        theta = a * (t - t0) * (t - t0) + theta0;
    }
    else if ( t > ts && (t < tf || t == tf) )
    {
        thetaDDot = -thetaDDotMax;
        thetaDot = thetaDDot * (t - t0) + thetaDot0 - thetaDDot * (tf - t0);
        theta = b * (t - tf) * (t - tf) + thetaRef;
    }
    else
    {
        thetaDDot = 0.0;
        thetaDot = thetaDotRef;
        theta = thetaRef;
    }

    /*! Initialize prescribed parameters */
    double omega_FB_F[3] = {0.0, 0.0, 0.0}; // [rad/s]
    double omegaPrime_FB_F[3] = {0.0, 0.0, 0.0}; // [rad/s^2]
    double sigma_FB[3] = {0.0, 0.0, 0.0};

    /*! Determine omega_FB_F and omegaPrime_FB_F parameters */
    omega_FB_F[spinAxis] = thetaDot;
    omegaPrime_FB_F[spinAxis] = thetaDDot;

    /*! Determine sigma_FB, mrp from F frame to B frame */
    double rotAxis_B[3] = {0.0, 0.0, 0.0};
    rotAxis_B[spinAxis] = 1;
    double dcm_FF0[3][3];

    /*! Determine dcm_FF0 */
    double prv_FF0_array[3];
    v3Scale(theta, rotAxis_B, prv_FF0_array);
    PRV2C(prv_FF0_array, dcm_FF0);

    /*! Determine dcm_F0B */
    double dcm_F0B[3][3];
    double prv_F0B_array[3];
    v3Scale(theta0, rotAxis_B, prv_F0B_array);
    PRV2C(prv_F0B_array, dcm_F0B);

    /*! Determine dcm_FB */
    double dcm_FB[3][3];
    m33MultM33(dcm_FF0, dcm_F0B, dcm_FB);
    C2MRP(dcm_FB, sigma_FB);

    /*! Copy local variables to output message */
    v3Copy(omega_FB_F, prescribedMotionOut.omega_FB_F);
    v3Copy(omegaPrime_FB_F, prescribedMotionOut.omegaPrime_FB_F);
    v3Copy(sigma_FB, prescribedMotionOut.sigma_FB);

    /*! write output message */
    PrescribedMotionMsg_C_write(&prescribedMotionOut, &configData->prescribedMotionOutMsg, moduleID, callTime);

    return;
}
