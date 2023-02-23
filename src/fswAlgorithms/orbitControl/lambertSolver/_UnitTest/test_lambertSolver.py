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

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSolver
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport

from IzzoLambert import *

solver = ["Gooding", "Izzo"]
revs = [0, 1, 4]
times = [10000, 3500e3]
eccentricities = [0.00001, 0.05, 1.0, 1.2]
transferAngle = [30., 90., 210.]


@pytest.mark.parametrize("accuracy", [1e-2])
@pytest.mark.parametrize("p1_solver, p2_revs, p3_times, p4_eccs, p5_angles", [
    (solver[0], revs[0], times[0], eccentricities[0], transferAngle[0]),
    (solver[1], revs[0], times[0], eccentricities[0], transferAngle[0]),
])

def test_lambertSolver(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert solver module works correctly for different Lambert solver algorithms, number of revolutions, times of flight,

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown.

    **Description of Variables Being Tested**

    The computed velocity vectors at position 1 and position 2 are compared to the solution of a Python script that uses Izzo's algorithm to solve Lambert's problem.
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

    # solverName = "Izzo"
    # r1vec = [38826.24143253, 52763.58685417, 83.17983272]
    # r2vec = [-26100., 0., 0.]
    # time = 7592.319902320611*10
    # mu = 4370000.
    # M = 2

    # set up the transfer orbit using classical orbit elements
    mu = 3.986004418e14
    oe1 = orbitalMotion.ClassicElements()
    radius = 10000. * 1000      # meters
    oe1.a = radius
    oe1.e = p4_eccs
    oe1.i = 0.0 * macros.D2R
    oe1.Omega = 30. * macros.D2R
    oe1.omega = 25. * macros.D2R
    oe1.f = 10. * macros.D2R
    r1_N, v1_N = orbitalMotion.elem2rv(mu, oe1)

    oe2 = oe1
    oe2.f = oe1.f + p5_angles * macros.D2R
    r2_N, v2_N = orbitalMotion.elem2rv(mu, oe2)

    solverName = p1_solver
    time = p3_times
    r1vec = r1_N
    r2vec = r2_N
    M = p2_revs

    # Configure input messages
    lambertProblemInMsgData = messaging.LambertProblemMsgPayload()
    lambertProblemInMsgData.solverName = solverName
    lambertProblemInMsgData.r1vec = r1vec
    lambertProblemInMsgData.r2vec = r2vec
    lambertProblemInMsgData.transferTime = time
    lambertProblemInMsgData.mu = mu
    lambertProblemInMsgData.numRevolutions = M
    lambertProblemInMsg = messaging.LambertProblemMsg().write(lambertProblemInMsgData)

    # subscribe input messages to module
    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)

    # setup output message recorder objects
    lambertSolutionOutMsgRec = module.lambertSolutionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertSolutionOutMsgRec)
    lambertPerformanceOutMsgRec = module.lambertPerformanceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertPerformanceOutMsgRec)

    Izzo = IzzoSolve(np.array(r1vec), np.array(r2vec), time, mu, M)
    Izzo.solve()

    idx = 2*M - 1
    idx_sol2 = idx + 1

    if M == 0:
        xTrue = Izzo.x[0]
        v1True = Izzo.v1[0]
        v2True = Izzo.v2[0]
        xTrue_sol2 = 0.
        v1True_sol2 = np.array([0., 0., 0.])
        v2True_sol2 = np.array([0., 0., 0.])
    elif idx > len(Izzo.x):
        xTrue = 0.
        v1True = np.array([0., 0., 0.])
        v2True = np.array([0., 0., 0.])
        xTrue_sol2 = 0.
        v1True_sol2 = np.array([0., 0., 0.])
        v2True_sol2 = np.array([0., 0., 0.])
    else:
        xTrue = Izzo.x[idx]
        v1True = Izzo.v1[idx]
        v2True = Izzo.v2[idx]
        xTrue_sol2 = Izzo.x[idx_sol2]
        v1True_sol2 = Izzo.v1[idx_sol2]
        v2True_sol2 = Izzo.v2[idx_sol2]

    print(xTrue)
    print(v1True)
    print(v2True)
    print(xTrue_sol2)
    print(v1True_sol2)
    print(v2True_sol2)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    v1 = lambertSolutionOutMsgRec.v1[0]
    v2 = lambertSolutionOutMsgRec.v2[0]
    valid = lambertSolutionOutMsgRec.valid[0]
    v1_sol2 = lambertSolutionOutMsgRec.v1_sol2[0]
    v2_sol2 = lambertSolutionOutMsgRec.v2_sol2[0]
    valid_sol2 = lambertSolutionOutMsgRec.valid_sol2[0]

    x = lambertPerformanceOutMsgRec.x[0]
    numIter = lambertPerformanceOutMsgRec.numIter[0]
    err_x = lambertPerformanceOutMsgRec.err_x[0]
    x_sol2 = lambertPerformanceOutMsgRec.x_sol2[0]
    numIter_sol2 = lambertPerformanceOutMsgRec.numIter_sol2[0]
    err_x_sol2 = lambertPerformanceOutMsgRec.err_x_sol2[0]

    print(x)
    print(numIter)
    print(err_x)
    print(v1)
    print(v2)
    print(valid)
    print(x_sol2)
    print(numIter_sol2)
    print(err_x_sol2)
    print(v1_sol2)
    print(v2_sol2)
    print(valid_sol2)

    # make sure module output data is correct
    ParamsString = ' for solver=' + p1_solver + ', rev=' + str(p2_revs) + ', time=' + str(p3_times) + ', eccentricity=' + str(p4_eccs) + ', angle=' + str(p5_angles) + ', accuracy=' + str(accuracy)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v1True, v1, accuracy, ('v1' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v2True, v2, accuracy, ('v2' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v1True_sol2, v1_sol2, accuracy, ('v1_sol2' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        v2True_sol2, v2_sol2, accuracy, ('v2_sol2' + ParamsString),
        testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_lambertSolver(False, solver[1], revs[0], times[0], eccentricities[0], transferAngle[0], 1e-2)


