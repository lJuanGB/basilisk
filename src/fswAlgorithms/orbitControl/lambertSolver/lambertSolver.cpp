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

#include "fswAlgorithms/orbitControl/lambertSolver/lambertSolver.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSolver::LambertSolver() = default;

/*! Module Destructor */
LambertSolver::~LambertSolver() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSolver.lambertProblemInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated. It computes the solution of Lambert's problem.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::UpdateState(uint64_t CurrentSimNanos)
{
    // read messages
    this->readMessages();
    // compute geometry of given Lambert problem
    this->problemGeometry();
    // find free variable x that satisfies the given time of flight
    this->findx();

    // compute velocity vector at initial and final position of Lambert's problem
    this->vvecs = this->computeVelocities(this->X);
    if (M > 0){
        // if one or more orbits are to be completed, two solutions exist. Compute velocities for second solution
        this->vvecs_sol2 = this->computeVelocities(this->X_sol2);
    }

    // write messages
    this->writeMessages(CurrentSimNanos);
}

/*! This method reads the input messages each call of updateState. It also checks if the message contents are valid for this module.
    @return void
*/
void LambertSolver::readMessages(){
    // Make local copies of messages
    LambertProblemMsgPayload lambertProblemInMsgBuffer;
    // read in the input messages
    lambertProblemInMsgBuffer = this->lambertProblemInMsg();

    // save message payload
    this->solverName.assign(lambertProblemInMsgBuffer.solverName);
    this->r1vec = cArray2EigenVector3d(lambertProblemInMsgBuffer.r1vec);
    this->r2vec = cArray2EigenVector3d(lambertProblemInMsgBuffer.r2vec);
    this->transferTime = lambertProblemInMsgBuffer.transferTime;
    this->mu = lambertProblemInMsgBuffer.mu;
    this->M = lambertProblemInMsgBuffer.numRevolutions;

    // check if input parameters are valid
    if (this->mu <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: mu must be positive.");
    }
    if (this->transferTime <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: transferTime must be positive.");
    }
    if (this->M < 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: M must be zero or positive.");
    }
    if (!(this->solverName == "Gooding" || this->solverName == "Izzo")){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: solverName must be either 'Gooding' or 'Izzo'.");
    }
}

/*! This method writes the output messages each call of updateState
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::writeMessages(uint64_t CurrentSimNanos){
    // Make local copies of messages
    LambertSolutionMsgPayload lambertSolutionOutMsgBuffer;
    LambertPerformanceMsgPayload lambertPerformanceOutMsgBuffer;
    // Always zero the output message buffers before assigning values
    lambertSolutionOutMsgBuffer = this->lambertSolutionOutMsg.zeroMsgPayload;
    lambertPerformanceOutMsgBuffer = this->lambertPerformanceOutMsg.zeroMsgPayload;

    if (this->M == 0){
        // if zero orbits are completed, only one solution exists. Only populate information for 1st solution, 2nd solution remains zero message payload

        eigenVector3d2CArray(this->vvecs.at(0), lambertSolutionOutMsgBuffer.v1);
        eigenVector3d2CArray(this->vvecs.at(1), lambertSolutionOutMsgBuffer.v2);
        lambertSolutionOutMsgBuffer.valid = 1; // solution 1 is valid

        lambertPerformanceOutMsgBuffer.x = this->X;
        lambertPerformanceOutMsgBuffer.numIter = this->numIter;
        lambertPerformanceOutMsgBuffer.err_x = this->err_x;
    }
    else if(this->multiRevSolution){
        // of one or more orbits are completed, two solutions exist (or 0 solutions if the transfer time is too short)

        eigenVector3d2CArray(this->vvecs.at(0), lambertSolutionOutMsgBuffer.v1);
        eigenVector3d2CArray(this->vvecs.at(1), lambertSolutionOutMsgBuffer.v2);
        lambertSolutionOutMsgBuffer.valid = 1; // solution 1 is valid
        eigenVector3d2CArray(this->vvecs_sol2.at(0), lambertSolutionOutMsgBuffer.v1_sol2);
        eigenVector3d2CArray(this->vvecs_sol2.at(1), lambertSolutionOutMsgBuffer.v2_sol2);
        lambertSolutionOutMsgBuffer.valid_sol2 = 1; // solution 2 is valid

        lambertPerformanceOutMsgBuffer.x = this->X;
        lambertPerformanceOutMsgBuffer.numIter = this->numIter;
        lambertPerformanceOutMsgBuffer.err_x = this->err_x;
        lambertPerformanceOutMsgBuffer.x_sol2 = this->X_sol2;
        lambertPerformanceOutMsgBuffer.numIter_sol2 = this->numIter_sol2;
        lambertPerformanceOutMsgBuffer.err_x_sol2 = this->err_x_sol2;
    }
    else{
        // transfer time is too short for multi-revolution solution, zero solutions exist. Neither solution is valid
        lambertSolutionOutMsgBuffer.valid = 0;
        lambertSolutionOutMsgBuffer.valid_sol2 = 0;
    }

    // Write to the output messages
    this->lambertSolutionOutMsg.write(&lambertSolutionOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->lambertPerformanceOutMsg.write(&lambertPerformanceOutMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! This method computes the problem geometry for the given parameters of Lambert's problem. The orbit frame is also determined.
    @return void
*/
void LambertSolver::problemGeometry()
{
    double c = (this->r2vec - this->r1vec).norm(); // chord length
    double r1 = this->r1vec.norm();
    double r2 = this->r2vec.norm();
    double s = 1.0/2.0*(r1+r2+c); // semiperimeter

    // lambda parameter
    this->lambda = sqrt(1.0-c/s);
    // non-dimensional time-of-flight
    this->TOF = sqrt(2.0*this->mu/pow(s,3))*this->transferTime;

    // compute orbit frame unit vectors

    // radial direction at for initial and final position vector
    Eigen::Vector3d i_r1 = this->r1vec/r1;
    Eigen::Vector3d i_r2 = this->r2vec/r2;
    // check if the two position vectors define a plane
    if (i_r1.cross(i_r2).norm() < 1e-8){
        bskLogger.bskLog(BSK_WARNING, "lambertSolver: position vectors r1 and r2 don't define a plane.");
    }
    // orbit normal direction
    Eigen::Vector3d i_h = i_r1.cross(i_r2)/(i_r1.cross(i_r2).norm());

    double sign = 1.0;
    if (i_h(2) < 0.0){
        // Transfer angle is greater than 180 degrees
        this->lambda = -this->lambda;
        sign = -sign;
    }
    // second unit vectors
    Eigen::Vector3d i_t1 = i_h.cross(i_r1)*sign;
    Eigen::Vector3d i_t2 = i_h.cross(i_r2)*sign;

    this->O_frame1 = {i_r1, i_t1, i_h};
    this->O_frame2 = {i_r2, i_t2, i_h};
}

/*! This method finds the free variable x that satisfies the requested time of flight TOF.
    @return void
*/
void LambertSolver::findx()
{
    // get initial guess for iteration variable x
    std::vector<double> x0 = getInitialGuess(this->lambda, this->TOF);
    if (this->M > 0 && !this->multiRevSolution){
        // if getInitialGuess function determines that TOF is too short for multi-revolution solution to exist, return no solution
        return;
    }

    // find x that satisfies time-of-flight constraint using numerical root finder
    if (this->solverName == "Gooding"){
        // for Gooding algorithm use halley root finder
        std::vector<double> solution = halley(this->TOF, x0[0], this->M);
        this->X = solution[0];
        this->numIter = int(solution[1]);
        this->err_x = solution[2];

        if (this->M > 0){
            // if one or more orbits are to be completed, two solutions exist. Find second solution
            std::vector<double> solution_sol2 = halley(this->TOF, x0[1], this->M);
            this->X_sol2 = solution_sol2[0];
            this->numIter_sol2 = int(solution_sol2[1]);
            this->err_x_sol2 = solution_sol2[2];
        }
    }
    else if (this->solverName == "Izzo"){
        // for Izzo algorithm use 3rd order householder root finder
        std::vector<double> solution = householder(this->TOF, x0[0], this->M);
        this->X = solution[0];
        this->numIter = int(solution[1]);
        this->err_x = solution[2];

        if (this->M > 0){
            // if one or more orbits are to be completed, two solutions exist. Find second solution
            std::vector<double> solution_sol2 = householder(this->TOF, x0[1], this->M);
            this->X_sol2 = solution_sol2[0];
            this->numIter_sol2 = int(solution_sol2[1]);
            this->err_x_sol2 = solution_sol2[2];
        }
    }
}

/*! This method computes the velocities at the initial and final position for a given free variable x
    @param x free variable of Lambert's problem that satisfies the given time of flight
    @return std::vector<Eigen::Vector3d>
*/
std::vector<Eigen::Vector3d> LambertSolver::computeVelocities(double x)
{
    double c = (this->r2vec - this->r1vec).norm(); // chord length
    double r1 = this->r1vec.norm();
    double r2 = this->r2vec.norm();
    double s = 1.0/2.0*(r1+r2+c); // semiperimeter

    double y = sqrt(1.0-pow(this->lambda,2)*(1.0-pow(x,2)));
    double gamma = sqrt(this->mu * s/2);
    double rho = (r1-r2)/c;
    double sigma = sqrt(1.0-pow(rho,2));
    
    // compute velocity components for v1vec and v2vec
    double Vr1 = gamma*((this->lambda*y - x) - rho*(this->lambda*y+x))/r1;
    double Vr2 = -gamma*((this->lambda*y - x) + rho*(this->lambda*y+x))/r2;
    double Vt1 = gamma*sigma*(y+this->lambda*x)/r1;
    double Vt2 = gamma*sigma*(y+this->lambda*x)/r2;

    // get orbit frame unit vectors
    Eigen::Vector3d i_r1 = this->O_frame1.at(0);
    Eigen::Vector3d i_t1 = this->O_frame1.at(1);
    Eigen::Vector3d i_r2 = this->O_frame2.at(0);
    Eigen::Vector3d i_t2 = this->O_frame2.at(1);

    // initial and final velocity of Lambert solution orbit
    Eigen::Vector3d v1 = Vr1*i_r1 + Vt1*i_t1;
    Eigen::Vector3d v2 = Vr2*i_r2 + Vt2*i_t2;

    std::vector<Eigen::Vector3d> velocities = {v1, v2};
    return velocities;
}

/*! This method computes the initial guess for the free variable x
    @param lam lambda parameter that defines the problem geometry
    @param T non-dimensional time-of-flight
    @return std::vector<double>
*/
std::vector<double> LambertSolver::getInitialGuess(double lam, double T)
{
    // T for x=0 and zero revolution
    double T00 = acos(lam) + lam*sqrt(1.0-pow(lam, 2));
    // T forx=1 (parabolic case)
    double T1 = 2.0/3.0*(1.0 - pow(lam, 3));
    // T for x=0 and M revolutions
    double T0M = T00 + this->M*M_PI;
    
    double x0_sol1 = 0.0; // initial guess for solution 1
    double x0_sol2 = 0.0; // initial guess for solution 2 (multi-revolutions only)
    
    if (this->solverName == "Gooding"){
        // initial guess procedure for Gooding algorithm. Involves several patches of solutions
        double c0 = 1.7;
        double c1 = 0.5;
        double c2 = 0.03;
        double c3 = 0.15;
        double c41 = 1.0;
        double c42 = 0.24;

        double theta = 2.0*atan2(1.0-pow(lam,2), 2.0*lam);
        // theta between 0 and pi if lambda > 0 and between pi and 2pi if lambda < 0
        if (lam < 0){
            theta = 2.0*M_PI - theta;
        }

        if (this->M == 0){
            // zero revolution case
            
            double x01 = -(T - T00)/(T - T00 + 4.0);
            // solution patches
            double W = x01 + 1.7*sqrt(2.0-theta/M_PI);
            double x03;
            if (W >= 0.0) {
                x03 = x01;
            }
            else {
                double x02 = -sqrt((T - T00)/(T + 1.0/2.0*T00));
                double w = pow(-W,1.0/16.0);
                x03 = x01 + w*(x02 - x01);
            }
            double scale = 1.0 + c1*x03*(1.0 + x01) - c2*pow(x03,2)*sqrt(1.0 + x01);

            x0_sol1 = scale * x03;
        }
        else {
            // multi-revolution case

            double x_M_pi = 4.0/(3.0*M_PI*(2*this->M + 1));
            double x_M; // x that corresponds to Tmin (the minimum TOF for two solutions to exist, otherwise zero solutions exist)
            if (theta <= M_PI){
                x_M = x_M_pi*pow(theta/M_PI, 1.0/8.0);
            }
            else {
                x_M = x_M_pi*(2.0 - pow(2.0 - theta/M_PI, 1.0/8.0));
            }

            // find Tmin using root-finder
            double Tmin = this->getTmin(T0M, this->M);
                
            if (T < Tmin){
                // if T < Tmin, no multi-revolution solution exists for the given time of flight T

                this->multiRevSolution = false;
                bskLogger.bskLog(BSK_WARNING, "lambertSolver: no multi-revolution solution exists for the given time of flight.");
                std::vector<double> x0 = {0.0, 0.0};
                return x0;
            }

            // if T >= Tmin, two multi-revolution solutions exist for the given time of flight T (or one solution if T = Tmin)
            this->multiRevSolution = true;

            // get derivatives of T at x_M
            std::vector<double> DTs = dTdx(x_M, Tmin, this->lambda);
            double D2T = DTs[1];

            double TdiffM = T - Tmin;
            double Tdiff = T - T0M;

            // solution patches for solution 1
            if (T > T0M){
                double term1 = TdiffM/(0.5*D2T - TdiffM*(0.5*D2T/(T0M-Tmin)-1/pow(x_M,2)));
                x0_sol1 = x_M - sqrt(term1);
            }
            else{
                double x01_1 =  -Tdiff/(Tdiff + 4.0);
                double W = x01_1 + c0*sqrt(2.0*(1.0-theta/(2*M_PI)));
                if (W < 0.0){
                    x0_sol1 = x01_1 - pow(-W, 1.0 / 16.0) * (x01_1 + sqrt(Tdiff / (Tdiff + 1.5 * T0M)));
                }
                else{
                    double W2 = 4.0/(4.0+Tdiff);
                    x0_sol1 = x01_1 * (1.0 + (1.0 + this->M + c42 * (theta / (2 * M_PI) - 0.5)) / (1.0 - c3 * this->M) * x01_1 * (c1 * W2 - c2 * x01_1 * pow(W2, 1.0 / 2.0)));
                }
            }

            // solution 2
            double term2 = TdiffM/(0.5*D2T + TdiffM/pow(1-x_M,2));
            double x01_2 = x_M + sqrt(term2);
            double W = x_M + x01_2;
            W = W*4.0/(4.0 + TdiffM) + pow(1.0-W,2);
            x0_sol2 = x01_2 * (1.0 - (1.0 + this->M + c41 * (theta / (2 * M_PI) - 0.5)) / (1.0 - c3 * this->M) * x01_2 * (c1 * W + c2 * x01_2 * pow(W, 1.0 / 2.0))) - x_M;
        }
    }
    else if (this->solverName == "Izzo"){
        // initial guess procedure for Izzo algorithm
        
        if (this->M == 0){
            // zero revolution case
            
            if (T >= T00){
                x0_sol1 = pow(T00 / T, 2.0 / 3.0) - 1.0;
            }
            else if (T <= T1){
                x0_sol1 = 5.0 / 2.0 * T1 * (T1 - T) / (T * (1.0 - pow(lam, 5))) + 1.0;
            }
            else{
                x0_sol1 = pow(T00 / T, log(T1 / T00) / log(2.0)) - 1.0;
            }
        }
        else{
            // multi-revolution case
            
            double Tmin = this->getTmin(T0M, this->M);
            if (T < Tmin){
                // if T < Tmin, no multi-revolution solution exists for the given time of flight T

                this->multiRevSolution = false;
                bskLogger.bskLog(BSK_WARNING, "lambertSolver: no multi-revolution solution exists for the given time of flight.");
                std::vector<double> x0 = {0.0, 0.0};
                return x0;
            }
            // if T > Tmin, two multi-revolution solutions exist for the given time of flight T (or one solution if T = Tmin)

            this->multiRevSolution = true;

            double term1 = pow((this->M*M_PI + M_PI)/(8.0*T),(2.0/3.0));
            double term2 = pow((8.0*T)/(this->M*M_PI),(2.0/3.0));
            x0_sol1 = (term1 - 1.0) / (term1 + 1.0);
            x0_sol2 = (term2 - 1.0) / (term2 + 1.0);
        }
    }
    
    std::vector<double> x0 = {x0_sol1, x0_sol2};
    return x0;
}

/*! This method computes the non-dimensional time of flight (TOF) for a given x
    @param x free variable of Lambert's problem that satisfies the given time of flight
    @param N number of revolutions
    @param lam lambda parameter that defines the problem geometry
    @return double
*/
double LambertSolver::x2tof(double x, int N, double lam)
{
    // if x is close to 1.0 (parabolic case), time-of-flight is computed according to Battin
    double battin = 0.01;
    double dist = abs(x - 1.0);
    double u = 1.0 - pow(x, 2);
    double y = sqrt(1.0- pow(lam, 2) * u);
    
    double tof;
    if (dist < 1e-8){
        // exact time of flight for parabolic case

        tof = 2.0/3.0*(1.0 - pow(lam,3));
    }
    else if (dist < battin){
        // Use Battin formulation
        
        double eta = y - lam * x;
        double S1 = 0.5 * (1.0 - lam - x * eta);
        double Q = 4.0 / 3.0 * hypergeometricF(S1);
        tof = (pow(eta,3)*Q + 4.0 * lam * eta) / 2.0 + N * M_PI / (pow(abs(u), 1.5));
    }
    else{
        // Use Lancaster formulation
        
        double f = (y - lam * x) * sqrt(abs(u));
        double g = x*y + lam * u;
        double d;
        if (u > 0.0){
            // elliptic case
            double psi = atan2(f,g);
            d = psi + N*M_PI;
        }
        else{
            // hyperbolic case
            d = atanh(f/g);
        }

        // non-dimensional time-of-flight
        tof = (d/sqrt(abs(u)) - x + lam * y) / u;
    }
    
    return tof;
}

/*! This method computes the derivatives of the time of flight curve T(x)
    @param x free variable of Lambert's problem that satisfies the given time of flight
    @param T requested non-dimensional time-of-flight
    @param lam lambda parameter that defines the problem geometry
    @return std::vector<double>
*/
std::vector<double> LambertSolver::dTdx(double x, double T, double lam)
{
    double DT; // dT/dx
    double D2T; // d2T/dx2
    double D3T; // d3T/dx3
    if (abs(x - 1.0) < 1e-8){
        // exact derivative for parabolic case

        DT = 2.0/5.0*(pow(lam,5) - 1.0);
        D2T = 0.0;
        D3T = 0.0;
    }
    else{
        double u = 1.0 - pow(x, 2);
        double y = sqrt(1.0- pow(lam, 2) * u);
        DT = 1.0/u*(3.0*T*x - 2.0 + 2.0 * pow(lam, 3) * x / y);
        D2T = 1.0/u*(3.0*T + 5.0*x*DT + 2.0 * (1. - pow(lam, 2)) * pow(lam, 3) / pow(y, 3));
        D3T = 1.0/u*(7.0*x*D2T + 8.0*DT - 6.0 * (1. - pow(lam, 2)) * pow(lam, 5) * x / pow(y, 5));
    }

    std::vector<double> DTs = {DT, D2T, D3T};
    
    return DTs;
}

/*! This method includes a 3rd order householder root-finder to find the free variable x that satisfies the time-of-flight constraint
    @param T requested non-dimensional time-of-flight
    @param x0 initial guess for x free variable of Lambert's problem that satisfies the given time of flight
    @param N number of revolutions
    @return std::vector<double>
*/
std::vector<double> LambertSolver::householder(double T, double x0, int N)
{
    double tol = 1e-8;
    int iter_max = 8;
    double xnew = 0.0; // initialize
    int j;
    double err;

    for (j = 0; j <= iter_max; ++j){
        // compute non-dimensional time-of-flight T for given x
        double tof = x2tof(x0, N, this->lambda);
        // get derivatives of T
        std::vector<double> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        double D3T = DTs[2];
        
        double delta = tof - T;
        // compute new x using 3rd order householder algorithm
        xnew = x0 - delta*(pow(DT,2) - delta*D2T/2.0)/(DT*(pow(DT,2) - delta*D2T) + D3T*pow(delta,2)/6.0);

        err = abs(x0 - xnew);
        x0 = xnew;
        if (err < tol){
            break;
        }
    }
    std::vector<double> sol = {xnew, static_cast<double>(j), err};

    return sol;
}

/*! This method includes a halley root-finder (2nd order householder) to find the free variable x that satisfies the time-of-flight constraint
    @param T requested non-dimensional time-of-flight
    @param x0 initial guess for x free variable of Lambert's problem that satisfies the given time of flight
    @param N number of revolutions
    @return std::vector<double>
*/
std::vector<double> LambertSolver::halley(double T, double x0, int N)
{
    double tol = 1e-8;
    int iter_max = 8;
    double xnew = 0.0; // initialize
    int j;
    double err;
    
    for (j = 0; j <= iter_max; ++j){
        // compute non-dimensional time-of-flight T for given x
        double tof = x2tof(x0, N, this->lambda);
        // get derivatives of T
        std::vector<double> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        
        double delta = tof - T;
        // compute new x using halley algorithm
        xnew = x0 - delta*DT/(pow(DT,2) - delta*D2T/2.0);
        err = abs(x0 - xnew);
        x0 = xnew;
        if (err < tol){
            break;
        }
    }

    std::vector<double> sol = {xnew, static_cast<double>(j), err};

    return sol;
}

/*! This method computes the minimum non-dimensional time-of-flight Tmin such that solutions exist for the multi-revolution case using a halley root-finder
    @param T0M initial guess for Tmin
    @param N number of revolutions
    @return double
*/
double LambertSolver::getTmin(double T0M, int N)
{
    double tol = 1e-5;
    double iter_max = 12;
    double xnew;
    double tof = T0M;
    double x0 = 0.0;

    for (int j = 0; j <= iter_max; ++j){
        // get derivatives of T
        std::vector<double> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        double D3T = DTs[2];
        
        // compute new x using halley algorithm
        xnew = x0 - DT*D2T/(pow(D2T,2) - DT*D3T/2.0);
        double err = abs(x0 - xnew);
        x0 = xnew;
        if (err < tol){
            break;
        }
        // compute non-dimensional time-of-flight T for given x
        tof = x2tof(xnew, N, this->lambda);
    }
    double Tmin = tof;
   
    return Tmin;
}

/*! This method computes the hypergeometric function 2F1(a,b,c,z)
    @param z argument of hypergeometric function
    @return double
*/
double LambertSolver::hypergeometricF(double z)
{
    double tol = 1e-11;
    double Sj = 1.0;
    double Cj = 1.0;
    double a = 3.0;
    double b = 1.0;
    double c = 2.5;

    double Cj1;
    double Sj1;
    double err;
    for (int j = 0; j <= 12; ++j){
        Cj1 = Cj * (a + j) * (b + j) / (c + j) * z / (j + 1);
        Sj1 = Sj + Cj1;
        err = abs(Cj1);
        Sj = Sj1;
        Cj = Cj1;
        if (err < tol){
            break;
        }
    }
    return Sj;
}
