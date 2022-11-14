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

    /* declare these quantities that will eventually become input modules */
    double h_B[3];                                //!< main heading in B frame coordinates
    double h_N[3];                                //!< main heading in N frame coordinates
    double a_B[3];                                //!< arrays axis direction in B frame

    int flagB;
    int flagN;
    int priorityFlag;

    /* declare module IO interfaces */
    NavAttMsg_C          attNavInMsg;             //!< input msg measured attitude
    BodyHeadingMsg_C     bodyHeadingInMsg;        //!< input body heading msg
    InertialHeadingMsg_C inertialHeadingInMsg;    //!< input inertial heading msg
    NavTransMsg_C        attTransInMsg;           //!< input msg measured position
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

    void v3Perpendicular(double x[3], double y[3]);

#ifdef __cplusplus
}
#endif


#endif
