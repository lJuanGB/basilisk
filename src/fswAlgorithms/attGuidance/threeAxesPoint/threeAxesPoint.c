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
#include "architecture/utilities/astroConstants.h"


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
        if (v3Norm(configData->h1_B) > EPS) {
            configData->flagB = 0;
        }
        else {
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: threeAxesPoint.bodyHeadingInMsg wasn't connected and no body heading h1_B was specified.");
        }
    }

    // check how the input inertial heading is provided
    if (InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg) || EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
        if (InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg) && !EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
            configData->flagN = 1;
        }
        else if (!InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg) && EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
            if (!NavTransMsg_C_isLinked(&configData->transNavInMsg)) {
                _bskLog(configData->bskLogger, BSK_ERROR, "Error: threeAxesPoint.ephemerisInMsg was specified but threeAxesPoint.transNavInMsg was not.");
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

    // set count to zerp
    configData->count = 0;
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
        transNavIn = NavTransMsg_C_read(&configData->transNavInMsg);
        v3Subtract(ephemerisIn.r_BdyZero_N, transNavIn.r_BN_N, hReq_N);
        v3Normalize(hReq_N, hReq_N);
    }
    
    /*! define the body frame orientation DCM BN */
    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    /*! get the solar array drive direction in body frame coordinates */
    double a1_B[3];
    v3Normalize(configData->a1_B, a1_B);

    /*! get the second body frame direction */
    double a2_B[3];
    if (v3Norm(configData->a2_B) > EPS) {
        v3Normalize(configData->a2_B, a2_B);
    }
    else {
        a2_B[0] = 0; a2_B[1] = 0; a2_B[2] = 0;
    }

    /*! read Sun direction in B frame from the attNav message */
    double rSun_B[3];
    v3Copy(attNavIn.vehSunPntBdy, rSun_B);

    /*! map requested heading into B frame */
    double hReq_B[3];
    m33MultV3(BN, hReq_N, hReq_B);

    /*! get body frame reference heading */
    double hRef_B[3];
    if (configData->flagB == 0) {
        v3Normalize(configData->h1_B, hRef_B);
    }
    else if (configData->flagB == 1) {
        bodyHeadingIn = BodyHeadingMsg_C_read(&configData->bodyHeadingInMsg);
        v3Normalize(bodyHeadingIn.rHat_XB_B, hRef_B);
    }

    /*! compute the total rotation DCM */
    double RN[3][3];
    computeFinalRotation(configData->priorityFlag, BN, rSun_B, hRef_B, hReq_B, a1_B, a2_B, RN);

    /*! compute the relative rotation DCM */
    double RB[3][3], rSun_1_R[3], rSun_2_R[3];
    m33MultM33t(RN, BN, RB);
    m33MultV3(RB, rSun_B, rSun_1_R);

    /*! compute reference MRP */
    double sigma_RN[3], omega_RN_R[3], omegaDot_RN_R[3];
    C2MRP(RN, sigma_RN);
    int flag = 0;

    if (v3Norm(configData->h2_B) > EPS && v3Norm(a2_B) > EPS) {
        // compute second reference frame 
        computeFinalRotation(configData->priorityFlag, BN, rSun_B, configData->h2_B, hReq_B, a1_B, a2_B, RN);
        m33MultM33t(RN, BN, RB);
        m33MultV3(RB, rSun_B, rSun_2_R);

        if (v3Dot(rSun_2_R, a2_B) > v3Dot(rSun_1_R, a2_B) && fabs(v3Dot(rSun_2_R, a2_B) - v3Dot(rSun_1_R, a2_B)) > EPS) {
            C2MRP(RN, sigma_RN);
            flag = 1;
        }
    }

    v3Copy(sigma_RN, attRefOut.sigma_RN);

    /*! compute reference MRP derivatives via finite differences */
    double T1, T2, delSigma[3], sigma_RN_1[3], sigma_RN_2[3], sigmaDot_RN[3], sigmaDDot_RN[3];
    // read sigma at t-1 and switch it if needed
    v3Copy(configData->sigma_RN_1, sigma_RN_1);
    v3Subtract(sigma_RN, sigma_RN_1, delSigma);
    if (v3Norm(delSigma) > 1) {
        MRPshadow(sigma_RN_1, sigma_RN_1);
    }
    // read sigma at t-2 and switch it if needed
    v3Copy(configData->sigma_RN_2, sigma_RN_2);
    v3Subtract(sigma_RN_1, sigma_RN_2, delSigma);
    if (v3Norm(delSigma) > 1) {
        MRPshadow(sigma_RN_2, sigma_RN_2);
    }
    // if first update call, derivatives are set to zero
    if (configData->count == 0) {
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = 0.0;
            sigmaDDot_RN[j] = 0.0;
        }
        // store information for next time step
        configData->T1 = callTime;
        v3Copy(sigma_RN, configData->sigma_RN_1);
    }
    // if second update call, derivatives are computed with first order finite differences
    else if (configData->count == 1) {
        T1 = - (double) (callTime - configData->T1) / 1e9;
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = (sigma_RN_1[j] - sigma_RN[j]) / T1;
            sigmaDDot_RN[j] = 0.0;
        }
        // store information for next time step
        configData->T2 = configData->T1;
        configData->T1 = callTime;
        v3Copy(configData->sigma_RN_1, configData->sigma_RN_2);
        v3Copy(sigma_RN, configData->sigma_RN_1);
    }
    // if third update call or higher, derivatives are computed with second order finite differences
    else {
        T1 = - (double) (callTime - configData->T1) / 1e9;
        T2 = - (double) (callTime - configData->T2) / 1e9;
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = ((sigma_RN_1[j]*T2*T2 - sigma_RN_2[j]*T1*T1) / (T2 - T1) - sigma_RN[j] * (T2 + T1)) / T1 / T2;
            sigmaDDot_RN[j] = 2 * ((sigma_RN_1[j]*T2 - sigma_RN_2[j]*T1) / (T1 - T2) + sigma_RN[j]) / T1 / T2;
        }
        // store information for next time step
        configData->T2 = configData->T1;
        configData->T1 = callTime;
        v3Copy(configData->sigma_RN_1, configData->sigma_RN_2);
        v3Copy(sigma_RN, configData->sigma_RN_1);
    }
    configData->count += 1;

    /*! compute angular rates and accelerations in R frame */
    dMRP2Omega(sigma_RN, sigmaDot_RN, omega_RN_R);
    ddMRP2dOmega(sigma_RN, sigmaDot_RN, sigmaDDot_RN, omegaDot_RN_R);

    /*! compute angular rates and accelerations in N frame and store in buffer msg */
    m33tMultV3(RN, omega_RN_R, attRefOut.omega_RN_N);
    m33tMultV3(RN, omegaDot_RN_R, attRefOut.domega_RN_N);

    /*! write output message */
    AttRefMsg_C_write(&attRefOut, &configData->attRefOutMsg, moduleID, callTime);

    return;
}

void computeFirstRotation(double hRef_B[3], double hReq_B[3], double R1B[3][3])
{
    /*! compute principal rotation angle (phi) and vector (e_phi) for the first rotation */
    double phi, e_phi[3];
    phi = acos( fmin( fmax( v3Dot(hRef_B, hReq_B), -1 ), 1 ) );
    v3Cross(hRef_B, hReq_B, e_phi);
    // If phi = PI, e_phi can be any vector perpendicular to F_current_B
    if (fabs(phi-MPI) < EPS) {
        phi = MPI;
        v3Perpendicular(hRef_B, e_phi);
    }
    else if (fabs(phi) < EPS) {
        phi = 0;
    }
    // normalize e_phi
    v3Normalize(e_phi, e_phi);

    /*! define first rotation R1B */
    double PRV_phi[3];
    v3Scale(phi, e_phi, PRV_phi);
    PRV2C(PRV_phi, R1B);
}

void computeSecondRotation(double hRef_B[3], double rSun_R1[3], double a1_B[3], double a2_B[3], double R2R1[3][3])
{
    /*! define second rotation vector to coincide with the thrust direction in B coordinates */
    double e_psi[3];
    v3Copy(hRef_B, e_psi);

    /*! define the coefficients of the quadratic equation A, B and C */
    double A, B, C, Delta, b[3];
    v3Cross(rSun_R1, e_psi, b);
    A = 2 * v3Dot(rSun_R1, e_psi) * v3Dot(e_psi, a1_B) - v3Dot(a1_B, rSun_R1);
    B = 2 * v3Dot(a1_B, b);
    C = v3Dot(a1_B, rSun_R1);
    Delta = B * B - 4 * A * C;

    /*! get the body direction that must be kept close to Sun and compute the coefficients of the quadratic equation E, F and G */
    double E, F, G;
    E = 2 * v3Dot(rSun_R1, e_psi) * v3Dot(e_psi, a2_B) - v3Dot(a2_B, rSun_R1);
    F = 2 * v3Dot(a2_B, b);
    G = v3Dot(a2_B, rSun_R1);

    /*! compute exact solution or best solution depending on Delta */
    double t, t1, t2, y, y1, y2, psi;
    if (fabs(A) < EPS) {
        if (fabs(B) < EPS) {
            if (C > 0) {
                psi = MPI;
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
        if (Delta < 0) {
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
                psi = MPI;
            }
        }
        else {
            t1 = (-B + sqrt(Delta)) / (2*A);
            t2 = (-B - sqrt(Delta)) / (2*A);

            t = t1;            
            if (fabs(v3Dot(hRef_B, a2_B)-1) > EPS) {
                y1 = (E*t1*t1 + F*t1 + G) / (1 + t1*t1);
                y2 = (E*t2*t2 + F*t2 + G) / (1 + t2*t2);
                if (y2 - y1 > EPS) {
                    t = t2;
                }
            }

            psi = 2*atan(t);
        }
    }

    /*! compute second rotation R2R1 */
    double PRV_psi[3];
    v3Scale(psi, e_psi, PRV_psi);
    PRV2C(PRV_psi, R2R1);
}

void computeThirdRotation(int priorityFlag, double hRef_B[3], double rSun_R2[3], double a1_B[3], double R3R2[3][3])
{
    double theta, sTheta, e_theta[3], aP_B[3]; 
    double PRV_theta[3];

    if (priorityFlag == 0) {
        for (int i = 0; i < 3; i++) {
            PRV_theta[i] = 0;
        }
    }
    else {
        sTheta = v3Dot(rSun_R2, a1_B);
        theta = asin( fmin( fmax( fabs(sTheta), -1 ), 1 ) );
        if (fabs(theta) < EPS) {
            // if Sun direction and solar array drive are already perpendicular, third rotation is null
            for (int i = 0; i < 3; i++) {
            PRV_theta[i] = 0;
            }
        }
        else {
            // if Sun direction and solar array drive are not perpendicular, project solar array drive a1_B onto perpendicular plane (aP_B) and compute third rotation
            if (fabs(fabs(theta)-MPI/2) > EPS) {
                for (int i = 0; i < 3; i++) {
                    aP_B[i] = (a1_B[i] - sTheta * rSun_R2[i]) / (1 - sTheta * sTheta);
                }
                v3Cross(a1_B, aP_B, e_theta);
            }
            else {
                // rotate about the axis that minimizes variation in hRef_B direction
                v3Cross(rSun_R2, hRef_B, aP_B);
                if (v3Norm(aP_B) < EPS) {
                    v3Perpendicular(rSun_R2, aP_B);
                }
                v3Cross(a1_B, aP_B, e_theta);
            }
            v3Normalize(e_theta, e_theta);
            v3Scale(theta, e_theta, PRV_theta);
        }
    }

    /*! compute third rotation R3R2 */
    PRV2C(PRV_theta, R3R2);
}

void computeFinalRotation(int priorityFlag, double BN[3][3], double rSun_B[3], double hRef_B[3], double hReq_B[3], double a1_B[3], double a2_B[3], double RN[3][3])
{
    /*! compute the first rotation DCM */
    double R1B[3][3];
    computeFirstRotation(hRef_B, hReq_B, R1B);

    /*! compute Sun direction vector in D frame coordinates */
    double rSun_R1[3];
    m33MultV3(R1B, rSun_B, rSun_R1);

    /*! compute the second rotation DCM */
    double R2R1[3][3];
    computeSecondRotation(hRef_B, rSun_R1, a1_B, a2_B, R2R1);

    /* compute Sun direction in R2 frame components */
    double rSun_R2[3];
    m33MultV3(R2R1, rSun_R1, rSun_R2);

    /*! compute the third rotation DCM */
    double R3R2[3][3];
    computeThirdRotation(priorityFlag, hRef_B, rSun_R2, a1_B, R3R2);

    /*! compute reference frames w.r.t inertial frame */
    double R1N[3][3], R2N[3][3];
    m33MultM33(R1B, BN, R1N);
    m33MultM33(R2R1, R1N, R2N);
    m33MultM33(R3R2, R2N, RN);
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
