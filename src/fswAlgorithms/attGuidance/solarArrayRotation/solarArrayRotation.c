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
#include "solarArrayRotation.h"
#include "string.h"
#include <math.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"


/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_solarArrayRotation(solarArrayRotationConfig *configData, int64_t moduleID)
{
    SpinningBodyMsg_C_init(&configData->spinningBodyRefOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_solarArrayRotation(solarArrayRotationConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input message is included
    if (!NavAttMsg_C_isLinked(&configData->attNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.attNavInMsg wasn't connected.");
    }
    // check if the required input message is included
    if (!AttRefMsg_C_isLinked(&configData->attRefInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.attRefInMsg wasn't connected.");
    }
    // check if the required input message is included
    if (!SpinningBodyMsg_C_isLinked(&configData->spinningBodyInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.spinningBodyInMsg wasn't connected.");
    }
    // zero counter
    configData->count = 0;
}

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current rotation angle
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_solarArrayRotation(solarArrayRotationConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Create buffer messages */
    NavAttMsgPayload         attNavIn;
    AttRefMsgPayload         attRefIn;
    SpinningBodyMsgPayload   spinningBodyIn;
    SpinningBodyMsgPayload   spinningBodyRefOut;

    /*! - zero the output message */
    spinningBodyRefOut = SpinningBodyMsg_C_zeroMsgPayload();

    /*! read the attitude navigation message */
    attNavIn = NavAttMsg_C_read(&configData->attNavInMsg);

    /*! read the attitude reference message */
    attRefIn = AttRefMsg_C_read(&configData->attRefInMsg);

    /*! read the solar array angle message */
    spinningBodyIn = SpinningBodyMsg_C_read(&configData->spinningBodyInMsg);

    /* read Sun direction in B frame from the attNav message and map it to R frame */
    double rS_B[3], rS_R[3], BN[3][3], RN[3][3], RB[3][3];
    v3Normalize(attNavIn.vehSunPntBdy, rS_B);
    switch (configData->bodyFrame) {

        case 0:
        MRP2C(attNavIn.sigma_BN, BN);
        MRP2C(attRefIn.sigma_RN, RN);
        m33MultM33t(RN, BN, RB);
        m33MultV3(RB, rS_B, rS_R);
        break;

        case 1:
        v3Copy(rS_B, rS_R);
        break;

        default:
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.bodyFrame input can be either 0 or 1.");
    }

    /* normalize a1_B and a2_B from module input */
    double a1_R_nominal[3], a2_R_nominal[3], a2_R_target[3];
    v3Normalize(configData->a1_B, a1_R_nominal);                    // a1 and a2 are body fixed, therefore their coordinates in B and R remain the same
    v3Normalize(configData->a2_B, a2_R_nominal);

    /* compute a2_R target */
    double dotP = v3Dot(a1_R_nominal, rS_R);
    for (int n = 0; n < 3; n++) {
        a2_R_target[n] = rS_R[n] - dotP * a1_R_nominal[n];
    }
    v3Normalize(a2_R_target, a2_R_target);

    double a1_R_target[3];
    v3Cross(a2_R_nominal, a2_R_target, a1_R_target);
    v3Normalize(a1_R_target, a1_R_target);

    /* compute reference rotation angle theta */
    double thetaR;
    double thetaC, sinThetaC, cosThetaC;
    // clip theta current between 0 and 2*pi
    sinThetaC = sin(spinningBodyIn.theta);
    cosThetaC = cos(spinningBodyIn.theta);
    thetaC = atan2(sinThetaC, cosThetaC);
    // compute shortest angle difference
    if (v3Norm(a2_R_target) < EPS) {
        spinningBodyRefOut.theta = spinningBodyIn.theta;
    }
    else {
        thetaR = acos( fmin(fmax(v3Dot(a2_R_nominal, a2_R_target),-1),1) );
        if (v3Dot(a1_R_nominal, a1_R_target) < 0) {
            thetaR = -thetaR;
        }
        if (thetaR - thetaC > MPI) {
            spinningBodyRefOut.theta = spinningBodyIn.theta + thetaR - thetaC - 2*MPI;
        }
        else if (thetaR - thetaC < - MPI) {
            spinningBodyRefOut.theta = spinningBodyIn.theta + thetaR - thetaC + 2*MPI;
        }
        else {
            spinningBodyRefOut.theta = spinningBodyIn.theta + thetaR - thetaC;
        }
    }

    /* implement finite differences to compute thetaDotR */
    double dt;
    if (configData->count == 0) {
        spinningBodyRefOut.thetaDot = 0;
    }
    else {
        dt = (double) (callTime - configData->priorT) / 1e9;
        spinningBodyRefOut.thetaDot = (thetaR - configData->priorThetaR) / dt;
    }
    // update stored variables
    configData->priorThetaR = thetaR;
    configData->priorT = callTime;
    configData->count += 1;
    

    /* write output message */
    SpinningBodyMsg_C_write(&spinningBodyRefOut, &configData->spinningBodyRefOutMsg, moduleID, callTime);

    return;
}
