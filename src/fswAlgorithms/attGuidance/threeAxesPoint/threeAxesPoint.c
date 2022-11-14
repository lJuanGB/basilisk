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
#include "threeAxesPoint.h"
#include "string.h"
#include <math.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"


/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_threeAxesPoint(threeAxesPointConfig *configData, int64_t moduleID)
{
    AttRefMsg_C_init(&configData->attRefOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_threeAxesPoint(threeAxesPointConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input message is included
    if (!NavAttMsg_C_isLinked(&configData->attNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: threeAxesPoint.attNavInMsg wasn't connected.");
    }

    // check how the input body heading is provided
    if (BodyHeadingMsg_C_isLinked(&configData->bodyHeadingInMsg)) {
        configData->flagB = 1;
    }
    else {
        if (v3Norm(configData->h_B) > EPS) {
            configData->flagB = 0;
        }
        else {
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: threeAxesPoint.bodyHeadingInMsg wasn't connected and no body heading h_B was specified.");
        }
    }

    // check how the input inertial heading is provided
    if (InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg) || EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
        if (InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg) && !EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
            configData->flagN = 1;
        }
        else if (!InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg) && EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
            if (!NavTransMsg_C_isLinked(&configData->attTransInMsg)) {
                _bskLog(configData->bskLogger, BSK_ERROR, "Error: threeAxesPoint.ephemerisInMsg was specified but threeAxesPoint.attTransInMsg was not.");
            }
            else {
                configData->flagN = 2;
            }
        }
        else {
            configData->flagN = 1;
            _bskLog(configData->bskLogger, BSK_WARNING, "Warning: both threeAxesPoint.inertialHeadingInMsg and threeAxesPoint.ephemerisInMsg were linked. Inertial heading is computed based on threeAxesPoint.inertialHeadingInMsg");
        }
    }
    else {
        if (v3Norm(configData->h_N) > EPS) {
            configData->flagN = 0;
        }
        else {
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: threeAxesPoint.inertialHeadingInMsg and threeAxesPoint.ephemerisInMsg weren't connected and no inertial heading h_N was specified.");
        }
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_threeAxesPoint(threeAxesPointConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Create buffer messages */
    NavAttMsgPayload           attNavIn;
    BodyHeadingMsgPayload      bodyHeadingIn;
    InertialHeadingMsgPayload  inertialHeadingIn;
    NavTransMsgPayload         transNavIn;
    EphemerisMsgPayload        ephemerisIn;
    AttRefMsgPayload           attRefOut;

    /*! - zero the output message */
    attRefOut = AttRefMsg_C_zeroMsgPayload();

    /* read the attitude navigation message */
    attNavIn = NavAttMsg_C_read(&configData->attNavInMsg);

    /*! get body frame reference heading */
    double hRef_B[3];
    if (configData->flagB == 0) {
        v3Normalize(configData->h_B, hRef_B);
    }
    else if (configData->flagB == 1) {
        bodyHeadingIn = BodyHeadingMsg_C_read(&configData->bodyHeadingInMsg);
        v3Normalize(bodyHeadingIn.rHat_XB_B, hRef_B);
    }

    /*! get inertial frame requested heading */
    double hReq_N[3];
    if (configData->flagN == 0) {
        v3Normalize(configData->h_N, hReq_N);
    }
    else if (configData->flagN == 1) {
        inertialHeadingIn = InertialHeadingMsg_C_read(&configData->inertialHeadingInMsg);
        v3Normalize(inertialHeadingIn.rHat_XN_N, hReq_N);
    }
    else if (configData->flagN == 2) {
        ephemerisIn = EphemerisMsg_C_read(&configData->ephemerisInMsg);
        transNavIn = NavTransMsg_C_read(&configData->attTransInMsg);
        v3Subtract(ephemerisIn.r_BdyZero_N, transNavIn.r_BN_N, hReq_N);
        v3Normalize(hReq_N, hReq_N);
    }
    
    /*! define the body frame orientation DCM BN */
    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    /*! get the solar array drive direction in body frame coordinates */
    double a_B[3];
    v3Normalize(configData->a_B, a_B);

    /*! read Sun direction in B frame from the attNav message */
    double rSun_B[3];
    v3Copy(attNavIn.vehSunPntBdy, rSun_B);

    /*! map requested heading into B frame */
    double hReq_B[3];
    m33MultV3(BN, hReq_N, hReq_B);

    /*! compute principal rotation angle (phi) and vector (e_phi) for the first rotation */
    double phi, e_phi[3];
    phi = acos( fmin( fmax( v3Dot(hRef_B, hReq_B), -1 ), 1 ) );
    v3Cross(hRef_B, hReq_B, e_phi);
    // If phi = PI, e_phi can be any vector perpendicular to F_current_B
    if (fabs(phi-M_PI) < EPS) {
        phi = M_PI;
        v3Perpendicular(hRef_B, e_phi);
    }
    else if (fabs(phi) < EPS) {
        phi = 0;
    }
    // normalize e_phi
    v3Normalize(e_phi, e_phi);

    /*! define intermediate rotation R1B */
    double R1B[3][3], PRV_phi[3];
    v3Scale(phi, e_phi, PRV_phi);
    PRV2C(PRV_phi, R1B);

    /*! compute Sun direction vector in D frame coordinates */
    double rSun_R1[3];
    m33MultV3(R1B, rSun_B, rSun_R1);

    /*! define second rotation vector to coincide with the thrust direction in B coordinates */
    double e_psi[3];
    v3Copy(hRef_B, e_psi);

    /*! define the coefficients of the quadratic equation */
    double A, B, C, Delta, b[3];
    v3Cross(rSun_R1, e_psi, b);
    A = 2 * v3Dot(rSun_R1, e_psi) * v3Dot(e_psi, a_B) - v3Dot(a_B, rSun_R1);
    B = 2 * v3Dot(a_B, b);
    C = v3Dot(a_B, rSun_R1);
    Delta = B * B - 4 * A * C;

    /*! compute exact solution or best solution depending on Delta */
    double t, t1, t2, y, y1, y2, psi;
    if (fabs(A) < EPS) {
        if (fabs(B) < EPS) {
            if (C > 0) {
                psi = M_PI;
            }
            else {
                psi = 0.0;
            }
        }
        else {
            t = - C / B;
            psi = 2*atan(t);
        }
    }
    else {
        if (Delta >= 0) {
            t1 = (-B + sqrt(Delta)) / (2*A);
            t2 = (-B - sqrt(Delta)) / (2*A);
            t = fmin(t1, t2);
            psi = 2*atan(t);
        }
        else {
            if (fabs(B) < EPS) {
                t = 0.0;
            }
            else {
                t1 = (A-C + sqrt((A-C)*(A-C) + B*B)) / B;
                t2 = (A-C - sqrt((A-C)*(A-C) + B*B)) / B;
                y1 = (A*t1*t1 + B*t1 + C) / (1 + t1*t1);
                y2 = (A*t2*t2 + B*t2 + C) / (1 + t2*t2);

                t = t1;
                if (fabs(y2) < fabs(y1)) {
                    t = t2;
                }
            }
            psi = 2*atan(t);
            y = (A*t*t + B*t + C) / (1 + t*t);
            if (fabs(A) < fabs(y)) {
                psi = M_PI;
            }
        }
    }

    /*! compute second rotation R2R1 */
    double R2R1[3][3], PRV_psi[3];
    v3Scale(psi, e_psi, PRV_psi);
    PRV2C(PRV_psi, R2R1);

    /*! compute second reference frame w.r.t inertial frame */
    double R1N[3][3], R2N[3][3];
    m33MultM33(R1B, BN, R1N);
    m33MultM33(R2R1, R1N, R2N);

    /*! define third rotation R3R2 */
    double R3R2[3][3];
    double theta, sTheta, e_theta[3], aP_B[3]; 
    double PRV_theta[3];

    /*! if priorityFlag == 0, the third rotation is null; otherwise, the third rotation is computed */
    double rSun_R2[3];
    m33MultV3(R2R1, rSun_R1, rSun_R2);
    if (configData->priorityFlag == 0) {
        for (int i = 0; i < 3; i++) {
            PRV_theta[i] = 0;
        }
    }
    else {
        sTheta = v3Dot(rSun_R2, a_B);
        theta = asin( fmin( fmax( fabs(sTheta), -1 ), 1 ) );
        if (fabs(theta) < EPS) {
            // if Sun direction and solar array drive are already perpendicular, third rotation is null
            for (int i = 0; i < 3; i++) {
            PRV_theta[i] = 0;
            }
        }
        else {
            // if Sun direction and solar array drive are not perpendicular, project solar array drive a_B onto perpendicular plane (aP_B) and compute third rotation
            if (fabs(theta-M_PI/2) > EPS) {
                for (int i = 0; i < 3; i++) {
                    aP_B[i] = (a_B[i] - sTheta * rSun_R2[i]) / (1 - sTheta * sTheta);
                }
                v3Cross(a_B, aP_B, e_theta);
            }
            else {
                v3Cross(rSun_R2, hRef_B, aP_B);
                if (v3Norm(aP_B) < EPS) {
                    v3Perpendicular(rSun_R2, aP_B);
                }
                v3Cross(a_B, aP_B, e_theta);
            }
            v3Normalize(e_theta, e_theta);
            v3Scale(theta, e_theta, PRV_theta);
        }
    }

    /*! compute third rotation R3R2 */
    PRV2C(PRV_theta, R3R2);

    /*! compute third reference frame w.r.t inertial frame */
    double R3N[3][3];
    m33MultM33(R3R2, R2N, R3N);

    /*! compute reference MRP */
    double sigma_RN[3];
    C2MRP(R3N, sigma_RN);

    v3Copy(sigma_RN, attRefOut.sigma_RN);

    /*! write output message */
    AttRefMsg_C_write(&attRefOut, &configData->attRefOutMsg, moduleID, callTime);

    return;
}


void v3Perpendicular(double x[3], double y[3])
{
    if (fabs(x[0]) > EPS) {
        y[0] = -(x[1]+x[2]) / x[0];
        y[1] = 1;
        y[2] = 1;
    }
    else if (fabs(x[1]) > EPS) {
        y[0] = 1;
        y[1] = -(x[0]+x[2]) / x[1];
        y[2] = 1;
    }
    else {
        y[0] = 1;
        y[1] = 1;
        y[2] = -(x[0]+x[1]) / x[2];
    }
}
