/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef LAMBERTSOLVER_H
#define LAMBERTSOLVER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertSolutionMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertPerformanceMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include <vector>

/*! @brief This module solves Lambert's problem using either the Gooding or the Izzo algorithm.
 */
class LambertSolver: public SysModel {
public:
    LambertSolver();
    ~LambertSolver();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    ReadFunctor<LambertProblemMsgPayload> lambertProblemInMsg;          //!<  lambert problem input message
    Message<LambertSolutionMsgPayload> lambertSolutionOutMsg;           //!< lambert solution output message
    Message<LambertPerformanceMsgPayload> lambertPerformanceOutMsg;     //!< lambert performance output message

    BSKLogger bskLogger;                                                //!< -- BSK Logging
    
private:
    void readMessages();
    void writeMessages(uint64_t CurrentSimNanos);
    void problemGeometry();
    void findx();
    std::vector<Eigen::Vector3d> computeVelocities(double x);
    std::vector<double> getInitialGuess(double lambda, double T);
    double x2tof(double x, int N, double lam);
    std::vector<double> dTdx(double x, double T, double lam);
    std::vector<double> householder(double T, double x0, int N);
    std::vector<double> halley(double T, double x0, int N);
    double getTmin(double T0M, int N);
    double hypergeometricF(double z);

    std::string solverName;         //!< name of lambert algorithm
    Eigen::Vector3d r1vec;          //!< position vector at t0
    Eigen::Vector3d r2vec;          //!< position vector at t1
    double transferTime{};          //!< time of flight between r1vec and r2vec (t1-t0)
    double mu{};                    //!< gravitational parameter
    int M{};                        //!< number of revolutions
    double TOF{};                   //!< non-dimensional time-of-flight constraint
    double lambda{};                //!< parameter of Lambert"s problem that defines problem geometry
    double X{};
    double X_sol2{};
    int numIter{};
    int numIter_sol2{};
    double err_x{};
    double err_x_sol2{};
    bool multiRevSolution{};
    bool noSolution{};
    std::vector<Eigen::Vector3d> vvecs;
    std::vector<Eigen::Vector3d> vvecs_sol2;
    std::vector<Eigen::Vector3d> O_frame1;
    std::vector<Eigen::Vector3d> O_frame2;
};


#endif
