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
#include "SEPPointing.h"
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
void SelfInit_SEPPointing(SEPPointingConfig *configData, int64_t moduleID)
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
void Reset_SEPPointing(SEPPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input message is included
    if (!NavAttMsg_C_isLinked(&configData->attNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: SEPPointing.attNavInMsg wasn't connected.");
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_SEPPointing(SEPPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Read input message */
    NavAttMsgPayload  attNavIn;
    AttRefMsgPayload  attRefOut;

    /*! - zero the output message */
    attRefOut = AttRefMsg_C_zeroMsgPayload();

    /* read the attitude navigation message */
    attNavIn = NavAttMsg_C_read(&configData->attNavInMsg);

    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    // define the current thrust direction in B frame and the target thrust direction in the N frame
    // these are currently hardcoded but will have to be read from an input message
    double F_current_B[3], F_requested_N[3], a_B[3];
    v3Copy(configData->F_current_B, F_current_B);
    v3Copy(configData->F_requested_N, F_requested_N);
    v3Copy(configData->a_B, a_B);
    /*
    F_current_B[0] = 0;  F_current_B[1] = 0;  F_current_B[2] = 1;
    F_requested_N[0]  = 0;  F_requested_N[1]  = 0;  F_requested_N[2]  = 1;
    a_B[0] = 1;          a_B[0] = 0;          a_B[0] = 0;
    */

    /* read Sun direction in B frame from the attNav message */
    double r_S_B[3];
    v3Copy(attNavIn.vehSunPntBdy, r_S_B);

    /* map requested thrust direction into B frame */
    double F_requested_B[3];
    m33MultV3(BN, F_requested_N, F_requested_B);

    double phi, e_phi[3];
    phi = acos( v3Dot(F_current_B, F_requested_B) );
    v3Cross(F_current_B, F_requested_B, e_phi);
    v3Normalize(e_phi, e_phi);
    if (fabs(phi-M_PI) < 1e-6) {
        // e_phi can be any vector perpendicular to F_current_B
        if (fabs(F_current_B[0]) > 1e-5) {
            e_phi[0] = -(F_current_B[1]+F_current_B[2]) / F_current_B[0];
            e_phi[1] = 1;
            e_phi[2] = 1;
        }
        else if (fabs(F_current_B[1]) > 1e-5) {
            e_phi[0] = 1;
            e_phi[1] = -(F_current_B[0]+F_current_B[2]) / F_current_B[1];
            e_phi[2] = 1;
        }
        else {
            e_phi[0] = 1;
            e_phi[1] = 1;
            e_phi[2] = -(F_current_B[0]+F_current_B[1]) / F_current_B[2];
        }
    }

    /* define intermediate rotation DB */
    double DB[3][3], PRV_phi[3];
    v3Scale(phi, e_phi, PRV_phi);
    PRV2C(PRV_phi, DB);

    /* compute Sun direction vector in D frame coordinates */
    double r_S_D[3];
    m33MultV3(DB, r_S_B, r_S_D);

    /* define second rotation vector to coincide with the thrust direction in B coordinates */
    double e_psi[3];
    v3Copy(F_current_B, e_psi);

    /* define the coefficients of the quadratic equation */
    double A, B, C, Delta, b[3];
    v3Cross(r_S_D, e_psi, b);
    A = 2 * v3Dot(r_S_D, e_psi) * v3Dot(e_psi, a_B) - v3Dot(a_B, r_S_D);
    B = 2 * v3Dot(a_B, b);
    C = v3Dot(a_B, r_S_D);
    Delta = B * B - 4 * A * C;

    /* compute exact solution or best solution depending on Delta */
    double t, t1, t2, y1, y2;
    if (Delta >= 0) {
        t1 = (-B + sqrt(Delta)) / (2*A);
        t2 = (-B - sqrt(Delta)) / (2*A);
        t = t1;
    }
    else {
        if (fabs(B) == 0.0) {
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
    }

    /* compute second rotation using Gibs' vector */
    double Q_psi[3], RD[3][3];
    v3Scale(t, e_psi, Q_psi);
    Gibbs2C(Q_psi, RD);

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
