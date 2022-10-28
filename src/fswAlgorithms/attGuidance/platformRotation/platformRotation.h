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

#ifndef _PLATFORM_ROTATION_
#define _PLATFORM_ROTATION_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/VehicleConfigMsg_C.h"

#define EPS 1e-6

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* declare these quantities that will eventually become input modules */
    double sigma_MB[3];                      //!< orientation of the M frame w.r.t. the B frame
    double r_BM_M[3];                        //!< position of B frame origin w.r.t. M frame origin, in M frame coordinates
    double r_FM_F[3];                        //!< position of F frame origin w.r.t. M frame origin, in F frame coordinates
    double r_TF_F[3];                        //!< position of the thrust application point w.r.t. F frame origin, in F frame coordinates
    double T_F[3];                           //!< thrust vector in F frame coordinates

    /* declare module IO interfaces */
    VehicleConfigMsg_C vehConfigInMsg;            //!< input msg vehicle configuration msg (needed for CM location)
    
    BSKLogger *bskLogger;                         //!< BSK Logging

}platformRotationConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_platformRotation(platformRotationConfig *configData, int64_t moduleID);
    void Reset_platformRotation(platformRotationConfig *configData, uint64_t callTime, int64_t moduleID);
    void Update_platformRotation(platformRotationConfig *configData, uint64_t callTime, int64_t moduleID);

    double computeSecondRotation(double r_CM_F[3], double r_FM_F[3], double r_TF_F[3], double r_CT_F[3], double T_F_hat[3]);
    double computeThirdRotation(double e_theta[3], double F2M[3][3]);

#ifdef __cplusplus
}
#endif


#endif
