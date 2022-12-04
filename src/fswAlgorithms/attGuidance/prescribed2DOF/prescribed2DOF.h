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
    double rotAxis1_M[3];                                      //!< M-frame spin axis of prescribed body
    double rotAxis2_F1[3];                                     //!< Intermediate F1-frame spin axis of prescribed body

    /* Private variables */
    double r_FM_M[3];                                          //!< [m] position vector of point F relative to point M in M frame components
    double rPrime_FM_M[3];                                     //!< [m/s] B frame time derivative of position vector r_FM_M in M frame components
    double rPrimePrime_FM_M[3];                                //!< [m/s^2] B frame time derivative of vector rPrime_FM_M in M frame components
    double omega_FM_F[3];                                      //!< [rad/s] angular velocity vector of F frame relative to the B frame in F frame components
    double omegaPrime_FM_F[3];                                 //!< [rad/s^2] B frame time derivative of omega_FB_F in F frame components
    double sigma_FM[3];                                        //!< Initial MRP attitude of frame F relative to the M frame
    bool convergence;                                          //!< Boolean variable for convergence to the desired reference attitude
    double tInit;                                              //!< [s] Initial time when module reset is called
    double rotAxis_M[3];                                       //!< Unit Euler axis for the reference PRV
    double phi;                                                //!< Current PRV angle
    double phiRef;                                             //!< Reference PRV angle (positive short rotation chosen is always)
    double phiDotRef;                                          //!< Reference PRV angle rate
    double phiRefAccum;                                        //!< This variable logs the accumulation of the reference PRV angles (used to determine phiAccum for plotting)
    double phiAccum;                                           //!< This variable holds the accumulation of the current PRV angles (used to determine phiAccum for plotting)
    double ts;                                                 //!< Switch time (halfway through the attitude maneuver)
    double tf;                                                 //!< Final time of the reference maneuver
    double a;                                                  //!< Upward parabolic constant for the first half of the attitude maneuver
    double b;                                                  //!< Downward parabolic constant for the second half of the attitude maneuver
    double dcm_F0M[3][3];                                      //!< DCM from the mount M frame to initial F0 frame of the prescribed body

    /* Declare module input-output interfaces */
    SpinningBodyMsg_C    spinningBodyRef1InMsg;                //!< Input msg for reference angles and angle rates
    SpinningBodyMsg_C    spinningBodyRef2InMsg;                //!< Input msg for reference angles and angle rates
    PrescribedMotionMsg_C prescribedMotionInMsg;               //!< Prescribed input attitude reference message
    PrescribedMotionMsg_C prescribedMotionOutMsg;              //!< Prescribed output attitude reference message

    BSKLogger *bskLogger;                                      //!< BSK Logging

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
