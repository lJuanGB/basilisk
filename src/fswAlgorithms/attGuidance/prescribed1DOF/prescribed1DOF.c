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
    SpinningBodyMsg_C_init(&configData->spinningBodyOutMsg);
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
    if (!SpinningBodyMsg_C_isLinked(&configData->spinningBodyInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribed1DOF.spinningBodyInMsg wasn't connected.");
    }

    if (!PrescribedMotionMsg_C_isLinked(&configData->prescribedMotionInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribed1DOF.prescribedMotionInMsg wasn't connected.");
    }

    /*! Store initial time */
    configData->tInit = callTime*1e-9; // [s]
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
    SpinningBodyMsgPayload spinningBodyIn;
    SpinningBodyMsgPayload spinningBodyOut;
    PrescribedMotionMsgPayload prescribedMotionIn;
    PrescribedMotionMsgPayload prescribedMotionOut;

    /*! Zero the output message */
    spinningBodyOut = SpinningBodyMsg_C_zeroMsgPayload();
    prescribedMotionOut = PrescribedMotionMsg_C_zeroMsgPayload();

    /*! Read the input messages */
    spinningBodyIn = SpinningBodyMsg_C_zeroMsgPayload();
    if (SpinningBodyMsg_C_isWritten(&configData->spinningBodyInMsg))
    {
        spinningBodyIn = SpinningBodyMsg_C_read(&configData->spinningBodyInMsg);
    }
    prescribedMotionIn = PrescribedMotionMsg_C_zeroMsgPayload();
    if (PrescribedMotionMsg_C_isWritten(&configData->prescribedMotionInMsg))
    {
        prescribedMotionIn = PrescribedMotionMsg_C_read(&configData->prescribedMotionInMsg);
    }
    v3Copy(prescribedMotionIn.r_FM_M, configData->r_FM_M);
    v3Copy(prescribedMotionIn.rPrime_FM_M, configData->rPrime_FM_M);
    v3Copy(prescribedMotionIn.rPrimePrime_FM_M, configData->rPrimePrime_FM_M);
    v3Copy(prescribedMotionIn.omega_FB_F, configData->omega_FB_F);
    v3Copy(prescribedMotionIn.omegaPrime_FB_F, configData->omegaPrime_FB_F);
    v3Copy(prescribedMotionIn.sigma_FM, configData->sigma_FM);
    
    /*! Define initial variables */
    if (SpinningBodyMsg_C_timeWritten(&configData->spinningBodyInMsg) == callTime)
    {
        double prv_FM_array[3];
        MRP2PRV(configData->sigma_FM, prv_FM_array);
        configData->thetaInit = v3Norm(prv_FM_array); // [rad]
        configData->thetaDotInit = v3Norm(configData->omega_FB_F); // [rad/s]
    }

    /*! Grab reference variables */
    double thetaRef = spinningBodyIn.theta; // [rad]
    double thetaDotRef = spinningBodyIn.thetaDot; // [rad/s]

    /*! Define temporal information */
    double tf = sqrt(((0.5 * fabs(thetaRef - configData->thetaInit)) * 8) / configData->thetaDDotMax); // [s]
    double ts = tf / 2; // switch time [s]
    double t = callTime*1e-9; // current time [s]

    /*! Define scalar module states */
    double thetaDDot;
    double thetaDot;
    double theta;

    // Define constants for analytic parabolas
    double a = (0.5 * thetaRef - configData->thetaInit) / ((ts - configData->tInit) * (ts - configData->tInit)); // Constant for first parabola
    double b = -0.5 * thetaRef / ((ts - tf) * (ts - tf)); // Constant for second parabola

    /*! Compute analytic scalar states: thetaDDot, thetaDot, and theta */
    if ((t < ts || t == ts) && tf != 0)
    {
        thetaDDot = configData->thetaDDotMax;
        thetaDot = thetaDDot * (t - configData->tInit) + configData->thetaDotInit;
        theta = a * (t - configData->tInit) * (t - configData->tInit) + configData->thetaInit;
    }
    else if ( t > ts && (t < tf || t == tf) && tf != 0)
    {
        thetaDDot = -1 * configData->thetaDDotMax;
        thetaDot = thetaDDot * (t - configData->tInit) + configData->thetaDotInit - thetaDDot * (tf - configData->tInit);
        theta = b * (t - tf) * (t - tf) + thetaRef;
    }
    else
    {
        thetaDDot = 0.0;
        thetaDot = thetaDotRef;
        theta = thetaRef;
    }

    /*! Determine omega_FB_F and omegaPrime_FB_F parameters */
    v3Normalize(configData->rotAxis_M, configData->rotAxis_M);
    v3Scale(thetaDot, configData->rotAxis_M, configData->omega_FB_F);
    v3Scale(thetaDDot, configData->rotAxis_M, configData->omegaPrime_FB_F);

    /*! Determine sigma_FM, mrp from F frame to M frame */
    double dcm_FF0[3][3];

    /*! Determine dcm_FF0 */
    double prv_FF0_array[3];
    double theta_FF0 = theta - configData->thetaInit;
    v3Scale(theta_FF0, configData->rotAxis_M, prv_FF0_array);
    PRV2C(prv_FF0_array, dcm_FF0);

    /*! Determine dcm_F0M */
    double dcm_F0M[3][3];
    double prv_F0M_array[3];
    v3Scale(configData->thetaInit, configData->rotAxis_M, prv_F0M_array);
    PRV2C(prv_F0M_array, dcm_F0M);

    /*! Determine dcm_FM */
    double dcm_FM[3][3];
    m33MultM33(dcm_FF0, dcm_F0M, dcm_FM);
    C2MRP(dcm_FM, configData->sigma_FM);

    /*! Copy local variables to output message */
    v3Copy(configData->r_FM_M, prescribedMotionOut.r_FM_M);
    v3Copy(configData->rPrime_FM_M, prescribedMotionOut.rPrime_FM_M);
    v3Copy(configData->rPrimePrime_FM_M, prescribedMotionOut.rPrimePrime_FM_M);
    v3Copy(configData->omega_FB_F, prescribedMotionOut.omega_FB_F);
    v3Copy(configData->omegaPrime_FB_F, prescribedMotionOut.omegaPrime_FB_F);
    v3Copy(configData->sigma_FM, prescribedMotionOut.sigma_FM);

    spinningBodyOut.theta = theta;
    spinningBodyOut.thetaDot = thetaDot;

    /*! write output message */
    SpinningBodyMsg_C_write(&spinningBodyOut, &configData->spinningBodyOutMsg, moduleID, callTime);
    PrescribedMotionMsg_C_write(&prescribedMotionOut, &configData->prescribedMotionOutMsg, moduleID, callTime);

    return;
}
