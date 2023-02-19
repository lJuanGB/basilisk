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
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSolver
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

from IzzoLambert import *

# @pytest.mark.parametrize("accuracy", [1e-12])
# @pytest.mark.parametrize("param1, param2", [
#      (1, 1)
#     ,(1, 3)
# ])

def test_lambertSolver(show_plots):
    r"""
    **Validation Test Description**

    This test checks two things: a large force output when the spacecraft is far from the waypoint, and a small force
    output when the spacecraft is at the waypoint.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown.

    **Description of Variables Being Tested**

    In this test, the ``forceRequestBody`` variable in the :ref:`CmdForceBodyMsgPayload` output by the module is tested.
    When far away from the waypoint, the force request should be larger than 1 N. When close to the waypoint, the force
    request should only account for third body perturbations and SRP.
    """
    [testResults, testMessages] = lambertSolverTestFunction()
    assert testResults < 1, testMessages


def lambertSolverTestFunction():
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

    r1vec = [38826.24143253, 52763.58685417, 83.17983272]
    r2vec = [-26100., 0., 0.]
    time = 7592.319902320611*10
    # conv = time/0.86518
    mu = 4370000
    M = 1

    module.solverName = "Gooding"
    module.r1vec = r1vec
    module.r2vec = r2vec
    module.transferTime = time
    module.mu = mu  # Gravitational constant of the asteroid
    module.M = M

    # setup output message recorder objects
    # lambertOutMsgRec = module.lambertSolutionOutMsgs[0].recorder()
    # unitTestSim.AddModelToTask(unitTaskName, lambertOutMsgRec)

    # Izzo = IzzoSolve(np.array(r1vec), np.array(r2vec), time, mu, M)
    # Izzo.solve()
    # print(Izzo.x)
    # print(Izzo.v1)
    # print(Izzo.v2)

    # v1 = [-8.35812140e+00, -2.84849745e-01, -4.49055031e-04]
    # v2 = [-0.17159854, -6.56310227, -0.01034649]

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    # x = lambertOutMsgRec.x
    # v1 = lambertOutMsgRec.v1
    # v2 = lambertOutMsgRec.v2
    x = module.lambertSolutionOutMsgs[0].read().x
    v1 = module.lambertSolutionOutMsgs[0].read().v1
    v2 = module.lambertSolutionOutMsgs[0].read().v2
    x_2 = module.lambertSolutionOutMsgs[1].read().x
    v1_2 = module.lambertSolutionOutMsgs[1].read().v1
    v2_2 = module.lambertSolutionOutMsgs[1].read().v2

    print(x)
    print(x_2)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_lambertSolver(False)


