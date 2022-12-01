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

#ifndef _PRESCRIBED2DOF_
#define _PRESCRIBED2DOF_

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/SpinningBodyMsg_C.h"
#include "cMsgCInterface/PrescribedMotionMsg_C.h"

#define PI 3.1415926536

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* User configurable variables */
    double phiDDotMax;                                         //!< Maximum angular acceleration [rad/s^2]
    double rotAxis1_M[3];                                        //!< F-frame spin axis of prescribed body
    double rotAxis2_F1[3];                                        //!< F-frame spin axis of prescribed body *GENERALIZE

    /* Private variables */
    double r_FM_M[3];
    double rPrime_FM_M[3];
    double rPrimePrime_FM_M[3];
    double omega_FM_F[3];
    double omegaPrime_FM_F[3];
    double sigma_FM[3];
    bool convergence;
    double tInit;                                               //!< Initial time when module reset is called [s]
    double rotAxisInit_M[3];
    double rotAxis_M[3];
    double phi;
    double phiRef;
    double phiRefPrev;
    double phiDotRef;
    double ts;
    double tf;
    double a;
    double b;
    double dcm_F0M[3][3];
    double phiAccum;

    /* Declare module input-output interfaces */
    SpinningBodyMsg_C    spinningBodyRef1InMsg;                     //!< Input msg for reference angles and angle rates
    SpinningBodyMsg_C    spinningBodyRef2InMsg;                     //!< Input msg for reference angles and angle rates
    SpinningBodyMsg_C    spinningBodyOutMsg;                                //!< Input msg for reference angle and angle rate
    PrescribedMotionMsg_C prescribedMotionInMsg;                            //!< Prescribed input attitude reference message
    PrescribedMotionMsg_C prescribedMotionOutMsg;                           //!< Prescribed output attitude reference message

    BSKLogger *bskLogger;                                                   //!< BSK Logging

}Prescribed2DOFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_prescribed2DOF(Prescribed2DOFConfig *configData, int64_t moduleID);
    void Reset_prescribed2DOF(Prescribed2DOFConfig *configData, uint64_t callTime, int64_t moduleID);
    void Update_prescribed2DOF(Prescribed2DOFConfig *configData, uint64_t callTime, int64_t moduleID);
#ifdef __cplusplus
}
#endif

#endif
