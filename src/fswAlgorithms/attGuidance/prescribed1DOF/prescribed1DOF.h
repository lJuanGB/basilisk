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

#ifndef _PRESCRIBED1DOF_
#define _PRESCRIBED1DOF_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/SpinningBodyMsg_C.h"
#include "cMsgCInterface/PrescribedMotionMsg_C.h"

#define PI 3.1415926536

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* User configurable variables */
    double thetaDDotMax;                                        //!< Maximum angular acceleration [rad/s^2]
    double rotAxis_B[3];                                               //!< body-frame spin axis of prescribed body *GENERALIZE
    double r_FM_M[3];
    double rPrime_FM_M[3];
    double rPrimePrime_FM_M[3];
    double omega_FB_F[3];
    double omegaPrime_FB_F[3];
    double sigma_FB[3];

    /* Private variables */
    double tInit;                                               //!< Initial time when module reset is called [s]
    double thetaInit;
    double thetaDotInit;

    /* Declare module input-output interfaces */
    SpinningBodyMsg_C    spinningBodyInMsg;                     //!< Input msg for reference angle and angle rate
    SpinningBodyMsg_C    spinningBodyOutMsg;                     //!< Input msg for reference angle and angle rate
    PrescribedMotionMsg_C prescribedMotionInMsg;                //!< Prescribed input attitude reference message
    PrescribedMotionMsg_C prescribedMotionOutMsg;               //!< Prescribed output attitude reference message

    BSKLogger *bskLogger;                                       //!< BSK Logging

}Prescribed1DOFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_prescribed1DOF(Prescribed1DOFConfig *configData, int64_t moduleID);
    void Reset_prescribed1DOF(Prescribed1DOFConfig *configData, uint64_t callTime, int64_t moduleID);
    void Update_prescribed1DOF(Prescribed1DOFConfig *configData, uint64_t callTime, int64_t moduleID);
#ifdef __cplusplus
}
#endif

#endif
