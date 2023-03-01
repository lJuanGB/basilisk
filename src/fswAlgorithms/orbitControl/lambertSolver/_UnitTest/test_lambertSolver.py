# 
#  ISC License
# 
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
# 
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
# 
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
# 
#
import copy

import numpy as np
import itertools
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSolver
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport

from IzzoLambert import *

# parameters
solver = ["Gooding", "Izzo"]
revs = [0, 1, 4]
times = [1e2, 1e6]
eccentricities = [0.0, 0.05, 1.0, 1.2]
transferAngle = [30., 90., 180., 210., -60., 500.]

paramArray = [solver, revs, times, eccentricities, transferAngle]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))
# transfer angles >= 180 or < 0 not applicable for parabolic and hyperbolic transfer. Delete from list.
paramList = [item for item in paramList if not (item[3] >= 1.0 and (item[4] >= 180. or item[4] < 0.))]

@pytest.mark.parametrize("accuracy", [1e-2])
@pytest.mark.parametrize("p1_solver, p2_revs, p3_times, p4_eccs, p5_angles", paramList)

def test_lambertSolver(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert solver module works correctly for different Lambert solver algorithms, number of revolutions, time of flight, transfer orbit eccentricities, and transfer angles.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown
        :param p1_solver: Lambert solver algorithm
        :param p2_revs: number of revolutions to be completed
        :param p3_times: time-of-flight (transfer time)
        :param p4_eccs: eccentricity of transfer orbit
        :param p5_angles: transfer angle

    **Description of Variables Being Tested**

    The computed velocity vectors at position 1 and position 2 are compared to the true velocity vectors. For the zero-revolution case, the eccentricity and transfer angle parameters are used to determine the time-of-flight input for the Lambert solver, and the corresponding orbit elements are used to determine the true velocity vectors for the given position vectors. For the multi-revolution case, the time-of-flight parameter is used as the input for the Lambert solver, and the true velocity vectors are computed using an external Python script that uses Izzo's algorithm to solve Lambert's problem.
    """
    [testResults, testMessages] = lambertSolverTestFunction(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, accuracy)
    assert testResults < 1, testMessages


def lambertSolverTestFunction(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, accuracy):
    """This test checks for a large force return when far away from the waypoint"""
    testFailCount = 0
    testMessages = []

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = lambertSolver.LambertSolver()
    module.ModelTag = "lambertSolver"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # set up the transfer orbit using classical orbit elements
    mu = 3.986004418e14
    oe1 = orbitalMotion.ClassicElements()
    r = 10000. * 1000  # meters
    if p4_eccs < 1.0:
        # elliptic case
        oe1.a = r
    else:
        # parabolic and hyperbolic case
        oe1.a = -r
    oe1.e = p4_eccs
    oe1.i = 5. * macros.D2R
    oe1.Omega = 25. * macros.D2R
    oe1.omega = 30. * macros.D2R
    oe1.f = -10. * macros.D2R
    r1_N, v1_N = orbitalMotion.elem2rv_parab(mu, oe1)

    oe2 = copy.deepcopy(oe1)
    oe2.f = oe1.f + p5_angles * macros.D2R
    # Izzo and Gooding Lambert algorithms only consider positive transfer angles. Convert to 0 < angle < 2pi
    oe2.f = (oe2.f*macros.R2D % 360) * macros.D2R
    r2_N, v2_N = orbitalMotion.elem2rv_parab(mu, oe2)

    # determine time-of-flight for given transfer orbit and position vectors (true anomalies)
    if p2_revs > 0:
        # for multi-revolution case use time-of-flight from parameterization
        t_transfer = p3_times
    elif p4_eccs < 1.0:
        # elliptic case
        M1 = orbitalMotion.E2M(orbitalMotion.f2E(oe1.f, p4_eccs), p4_eccs)
        M2 = orbitalMotion.E2M(orbitalMotion.f2E(oe2.f, p4_eccs), p4_eccs)
        n = np.sqrt(mu/(oe1.a)**3)
        t_transfer = np.abs(M2-M1)/n
    elif p4_eccs == 1.0:
        # parabolic case
        D1 = np.tan(oe1.f/2)
        D2 = np.tan(oe2.f/2)
        M1 = D1 + 1/3*D1**3
        M2 = D2 + 1/3*D2**3
        n = np.sqrt(mu/(2*(-oe1.a)**3))
        t_transfer = np.abs(M2-M1)/n
    else:
        # hyperbolic case
        N1 = orbitalMotion.H2N(orbitalMotion.f2H(oe1.f, p4_eccs), p4_eccs)
        N2 = orbitalMotion.H2N(orbitalMotion.f2H(oe2.f, p4_eccs), p4_eccs)
        n = np.sqrt(mu/(-oe1.a)**3)
        t_transfer = np.abs(N2-N1)/n

    solverName = p1_solver
    time = t_transfer
    r1vec = r1_N
    r2vec = r2_N
    revs = p2_revs

    # Configure input messages
    lambertProblemInMsgData = messaging.LambertProblemMsgPayload()
    lambertProblemInMsgData.solverName = solverName
    lambertProblemInMsgData.r1vec = r1vec
    lambertProblemInMsgData.r2vec = r2vec
    lambertProblemInMsgData.transferTime = time
    lambertProblemInMsgData.mu = mu
    lambertProblemInMsgData.numRevolutions = revs
    lambertProblemInMsg = messaging.LambertProblemMsg().write(lambertProblemInMsgData)

    # subscribe input messages to module
    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)

    # setup output message recorder objects
    lambertSolutionOutMsgRec = module.lambertSolutionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertSolutionOutMsgRec)
    lambertPerformanceOutMsgRec = module.lambertPerformanceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertPerformanceOutMsgRec)

    # for multi-revolution case, use external Lambert solver
    if revs > 0 and p5_angles != 180.:
        Izzo = IzzoSolve(np.array(r1vec), np.array(r2vec), time, mu, revs)
        Izzo.solve()
        numSolutions = len(Izzo.x)

    idx = 2 * revs - 1
    idx_sol2 = idx + 1

    if p5_angles == 180. or (revs > 0 and idx+1 > numSolutions):
        # 1. if the transfer angle is 180 degrees, the two position vectors do not define a plane, so an infinite number of solutions exist. In this case, the module should not return any solutions.
        # 2. external Lambert solver does not compute solution if requested transfer time is less than minimum time-of-flight for multi-revolution solution. In this case set all true outputs to zero (so does the lambert module as well, since no solution exists for the requested time-of-flight)
        v1True = np.array([0., 0., 0.])
        v2True = np.array([0., 0., 0.])
        validFlagTrue = 0
        v1True_sol2 = np.array([0., 0., 0.])
        v2True_sol2 = np.array([0., 0., 0.])
        validFlagTrue_sol2 = 0
    elif revs == 0:
        # for zero-revolution case, obtain true velocity vectors from computed transfer orbit
        v1True = v1_N
        v2True = v2_N
        validFlagTrue = 1
        v1True_sol2 = np.array([0., 0., 0.])
        v2True_sol2 = np.array([0., 0., 0.])
        validFlagTrue_sol2 = 0
    else:
        # for multi-revolution case, obtain true velocity vectors from external Lambert solver script if solution exist
        v1True = Izzo.v1[idx]
        v2True = Izzo.v2[idx]
        validFlagTrue = 1
        v1True_sol2 = Izzo.v1[idx_sol2]
        v2True_sol2 = Izzo.v2[idx_sol2]
        validFlagTrue_sol2 = 1

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    v1 = lambertSolutionOutMsgRec.v1[0]
    v2 = lambertSolutionOutMsgRec.v2[0]
    validFlag = lambertSolutionOutMsgRec.valid[0]
    v1_sol2 = lambertSolutionOutMsgRec.v1_sol2[0]
    v2_sol2 = lambertSolutionOutMsgRec.v2_sol2[0]
    validFlag_sol2 = lambertSolutionOutMsgRec.valid_sol2[0]

    # make sure module output data is correct
    ParamsString = ' for solver=' + p1_solver + ', rev=' + str(p2_revs) + ', time=' + str(p3_times) + ', eccentricity=' + str(p4_eccs) + ', angle=' + str(p5_angles) + ', accuracy=' + str(accuracy)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v1True, v1, accuracy, ('Variable: v1,' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v2True, v2, accuracy, ('Variable: v2,' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        np.array([validFlagTrue]), np.array([validFlag]), accuracy, ('Variable: validFlag,' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v1True_sol2, v1_sol2, accuracy, ('Variable: v1_sol2,' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v2True_sol2, v2_sol2, accuracy, ('Variable: v2_sol2,' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        np.array([validFlagTrue_sol2]), np.array([validFlag_sol2]), accuracy, ('Variable: validFlag_sol2,' + ParamsString),
        testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_lambertSolver(False, solver[0], revs[1], times[1], eccentricities[1], transferAngle[2], 1e-2)


