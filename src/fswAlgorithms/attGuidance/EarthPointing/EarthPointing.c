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
#include "EarthPointing.h"
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
void SelfInit_EarthPointing(EarthPointingConfig *configData, int64_t moduleID)
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
void Reset_EarthPointing(EarthPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input messages are included
    if (!NavAttMsg_C_isLinked(&configData->attNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: EarthPointing.attNavInMsg wasn't connected.");
    }
    if (!NavTransMsg_C_isLinked(&configData->transNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: EarthPointing.transNavInMsg wasn't connected.");
    }
    if (!EphemerisMsg_C_isLinked(&configData->ephEarthInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: EarthPointing.ephEarthInMsg wasn't connected.");
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_EarthPointing(EarthPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Create buffer messages */
    NavAttMsgPayload        attNavIn;
    NavTransMsgPayload      transNavIn;
    EphemerisMsgPayload     ephEarthIn;
    AttRefMsgPayload        attRefOut;

    /*! - zero the output message */
    attRefOut = AttRefMsg_C_zeroMsgPayload();

    /* read the attitude navigation message */
    attNavIn = NavAttMsg_C_read(&configData->attNavInMsg);

    /* read the attitude navigation message */
    transNavIn = NavTransMsg_C_read(&configData->transNavInMsg);

    /* read the attitude navigation message */
    ephEarthIn = EphemerisMsg_C_read(&configData->ephEarthInMsg);

    /* get current state attitude */
    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    /* normalize inputs and store them in local variables */
    double HGA_B[3], TSP_B[3], a_B[3];
    v3Normalize(configData->HGA_B, HGA_B);
    v3Normalize(configData->TSP_B, TSP_B);
    v3Normalize(configData->a_B, a_B);

    /* read Sun direction in B frame from the attNav message */
    double r_S_B[3];
    v3Normalize(attNavIn.vehSunPntBdy, r_S_B);

    /* compute Earth direction w.r.t. spacecraft */
    double r_E_N[3], r_E_B[3];
    v3Subtract(ephEarthIn.r_BdyZero_N, transNavIn.r_BN_N, r_E_N);
    v3Normalize(r_E_N, r_E_N);
    m33MultV3(BN, r_E_N, r_E_B);

    /* compute principal rotation angle (phi) and vector (e_phi) for the first rotation */
    double phi, e_phi[3];
    phi = acos( fmin( fmax( v3Dot(HGA_B, r_E_B), -1 ), 1 ) );
    v3Cross(HGA_B, r_E_B, e_phi);
    // If phi = PI, e_phi can be any vector perpendicular to HGA_B
    if (fabs(phi-M_PI) < EPS) {
        phi = M_PI;
        if (fabs(HGA_B[0]) > EPS) {
            e_phi[0] = -(HGA_B[1]+HGA_B[2]) / HGA_B[0];
            e_phi[1] = 1;
            e_phi[2] = 1;
        }
        else if (fabs(HGA_B[1]) > EPS) {
            e_phi[0] = 1;
            e_phi[1] = -(HGA_B[0]+HGA_B[2]) / HGA_B[1];
            e_phi[2] = 1;
        }
        else {
            e_phi[0] = 1;
            e_phi[1] = 1;
            e_phi[2] = -(HGA_B[0]+HGA_B[1]) / HGA_B[2];
        }
    }
    else if (fabs(phi) < EPS) {
        phi = 0;
    }
    // normalize e_phi
    v3Normalize(e_phi, e_phi);

    /* define intermediate rotation DB */
    double DB[3][3], PRV_phi[3];
    v3Scale(phi, e_phi, PRV_phi);
    PRV2C(PRV_phi, DB);

    /* compute Sun direction vector in D frame coordinates */
    double r_S_D[3];
    m33MultV3(DB, r_S_B, r_S_D);

    /* define second rotation vector to coincide with the thrust direction in B coordinates */
    double e_psi[3];
    v3Copy(HGA_B, e_psi);

    /* define the coefficients of the quadratic equation */
    double A, B, C, Delta, b[3];
    v3Cross(r_S_D, e_psi, b);
    A = 2 * v3Dot(r_S_D, e_psi) * v3Dot(e_psi, a_B) - v3Dot(a_B, r_S_D);
    B = 2 * v3Dot(a_B, b);
    C = v3Dot(a_B, r_S_D);
    Delta = B * B - 4 * A * C;

    /* compute exact solution or best solution depending on Delta */
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

            t = discriminate_t(t1, t2, e_psi, r_S_D, HGA_B);
            
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

    /* compute second rotation RD */
    double RD[3][3], PRV_psi[3];
    v3Scale(psi, e_psi, PRV_psi);
    PRV2C(PRV_psi, RD);

    /* compute final reference frame w.r.t inertial frame */
    double DN[3][3], RN[3][3];
    m33MultM33(DB, BN, DN);
    m33MultM33(RD, DN, RN);

    /* compute reference MRP */
    double sigma_RN[3];
    C2MRP(RN, sigma_RN);

    v3Copy(sigma_RN, attRefOut.sigma_RN);

    /* write output message */
    AttRefMsg_C_write(&attRefOut, &configData->attRefOutMsg, moduleID, callTime);

    return;
}

double discriminate_t(double t1, double t2, double e_psi[3], double r_S_D[3], double HGA_R[3])
{
    // case 1
    double q1[3], RD1[3][3], r_S_R1[3], c1;
    v3Scale(t1, e_psi, q1);
    Gibbs2C(q1, RD1);
    m33MultV3(RD1, r_S_D, r_S_R1);
    c1 = v3Dot(r_S_R1, HGA_R);

    // case 2
    double q2[3], RD2[3][3], r_S_R2[3], c2;
    v3Scale(t2, e_psi, q2);
    Gibbs2C(q2, RD2);
    m33MultV3(RD2, r_S_D, r_S_R2);
    c2 = v3Dot(r_S_R2, HGA_R);

    // choose t that gives the largest angle between Sun and HGA
    double t;
    if (c2 > c1) {
        t = t1;
    }
    else {
        t = t2;
    }

    return t;
}