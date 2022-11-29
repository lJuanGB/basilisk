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

#ifndef _THREE_AXIS_POINT_
#define _THREE_AXIS_POINT_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/AttRefMsg_C.h"
#include "cMsgCInterface/BodyHeadingMsg_C.h"
#include "cMsgCInterface/InertialHeadingMsg_C.h"
#include "cMsgCInterface/NavTransMsg_C.h"
#include "cMsgCInterface/EphemerisMsg_C.h"
#include "cMsgCInterface/NavAttMsg_C.h"

#define EPS 1e-6


/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* declare these quantities that always must be specified as flight software parameters */
    double a1_B[3];                                //!< arrays axis direction in B frame
    int priorityFlag;                              //!< flag to indicate which constraint must be prioritized

    /* declare these optional quantities */
    double h_B[3];                                //!< main heading in B frame coordinates
    double h_N[3];                                //!< main heading in N frame coordinates
    double a2_B[3];                               //!< body frame heading that should remain as close as possible to Sun heading

    /* declare these internal variables that are used by the module and should not be declared by the user */
    int      flagB;
    int      flagN;
    int      count;                              //!< count variable used in the finite difference logic
    uint64_t T1;                                 //!< callTime one update step prior
    uint64_t T2;                                 //!< callTime two update steps prior
    double   sigma_RN_1[3];                      //!< reference attitude one update step prior
    double   sigma_RN_2[3];                      //!< reference attitude two update steps prior

    /* declare module IO interfaces */
    NavAttMsg_C          attNavInMsg;             //!< input msg measured attitude
    BodyHeadingMsg_C     bodyHeadingInMsg;        //!< input body heading msg
    InertialHeadingMsg_C inertialHeadingInMsg;    //!< input inertial heading msg
    NavTransMsg_C        transNavInMsg;           //!< input msg measured position
    EphemerisMsg_C       ephemerisInMsg;          //!< input ephemeris msg
    AttRefMsg_C          attRefOutMsg;            //!< output attitude reference message

    BSKLogger *bskLogger;                         //!< BSK Logging

}threeAxesPointConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_threeAxesPoint(threeAxesPointConfig *configData, int64_t moduleID);
    void Reset_threeAxesPoint(threeAxesPointConfig *configData, uint64_t callTime, int64_t moduleID);
    void Update_threeAxesPoint(threeAxesPointConfig *configData, uint64_t callTime, int64_t moduleID);

    void computeFirstRotation(double hRef_B[3], double hReq_B[3], double R1B[3][3]);
    void computeSecondRotation(double hRef_B[3], double rSun_R1[3], double a1_B[3], double a2_B[3], double R2R1[3][3]);
    void computeThirdRotation(int priorityFlag, double hRef_B[3], double rSun_R2[3], double a1_B[3], double R3R2[3][3]);
    void computeFinalRotation(int priorityFlag, double BN[3][3], double rSun_B[3], double hRef_B[3], double hReq_B[3], double a1_B[3], double a2_B[3], double RN[3][3]);

    void v3Perpendicular(double x[3], double y[3]);


#ifdef __cplusplus
}
#endif


#endif
