#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Unit Test Script
#   Module Name:        SEPPointing
#   Author:             Riccardo Calaon
#   Creation Date:      October 20, 2022
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.utilities import RigidBodyKinematics as rbk
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import threeAxesPoint                # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging


def computeGamma(alpha, delta):

    if alpha >= 0 and alpha <= np.pi/2:
        if delta < np.pi/2 - alpha:
            gamma = np.pi/2 - alpha - delta
        elif delta > alpha + np.pi/2:
            gamma = - np.pi/2 - alpha + delta
        else:
            gamma = 0
    else:
        if delta < alpha - np.pi/2:
            gamma = - np.pi/2 + alpha - delta 
        elif delta > 3/2*np.pi - alpha:
            gamma = alpha + delta - 3/2*np.pi
        else:
            gamma = 0

    return gamma

# The 'ang' array spans the interval from 0 to pi. 0 and pi are excluded because 
# the code is less accurate around those points; it still provides accurate results at 1e-6
ang = np.linspace(0, np.pi, 5, endpoint=True)
ang = list(ang)
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("alpha", ang)
@pytest.mark.parametrize("delta", ang)
@pytest.mark.parametrize("flagB", [0,1])
@pytest.mark.parametrize("flagN", [0,1,2])
@pytest.mark.parametrize("priorityFlag", [0,1])
@pytest.mark.parametrize("accuracy", [1e-6])

# update "module" in this function name to reflect the module name
def test_threeAxesPointTestFunction(show_plots, alpha, delta, flagB, flagN, priorityFlag, accuracy):
    r"""
    **Validation Test Description**

    Compose a general description of what is being tested in this unit test script.  Add enough information so the
    reader understands the purpose and limitations of the test.  As this test script is not parameterized, only one
    version of this script will run.  Note that the ``pytest`` HTML report will list each parameterized test case
    individually.  This way it is clear what set of parameters passed.  But, this also means that this doc-string
    content will be copied into each report so each test description is individually complete.  If there is a
    discussion you want to include that is specific to the a parameterized test case, then include this at the
    end of the file with a conditional print() statement that only executes for that particular parameterized test.

    **Test Parameters**

    As this is a parameterized unit test, note that the test case parameters values are shown automatically in the
    pytest HTML report.  This sample script has the parameters param1 and param 2.  Provide a description of what
    each parameter controls.  This is a convenient location to include the accuracy variable used in the
    validation test.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what parameters are being checked.  For example, in this file we are checking the values of the
    variables

    - ``dummy``
    - ``dataVector[3]``

    **Figure Discussion**

    If the test script produces figures you might include a brief discussion on what the simulation results show.
    Discuss why these results validate the operation of the BSK module.

    **General Documentation Comments**

    If the script generates figures, these figures will be automatically pulled from ``matplotlib`` and included below.
    Make sure that the figures have appropriate axes labels and a figure title if needed.  The figures content
    should be understood by just looking at the figure.

    At the end of the script where a print statement says that the script passes.

    Don't use any of the AutoTeX methods we used to use as the goal is to have all the validation reporting
    contained within this HTML ``pytest`` report.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = threeAxesPointTestFunction(show_plots, alpha, delta, flagB, flagN, priorityFlag, accuracy)

    assert testResults < 1, testMessage


def threeAxesPointTestFunction(show_plots, alpha, delta, flagB, flagN, priorityFlag, accuracy):

    gamma_true = computeGamma(alpha, delta)

    rS_N = np.array([0, 0, 1])                            # Sun direction in inertial coordinates
    rS_N = rS_N / np.linalg.norm(rS_N)
    sA_B = np.array([1, 0, 0])                            # array axis direction in body frame
    sA_B = sA_B / np.linalg.norm(sA_B)

    a = np.cross(rS_N, np.random.rand(3))
    a = a / np.linalg.norm(a)
    
    d = np.cross(sA_B, np.random.rand(3))
    d = d / np.linalg.norm(d)
    
    DCM1 = rbk.PRV2C(a * alpha)
    DCM2 = rbk.PRV2C(d * delta)

    h_N = np.matmul(DCM1, rS_N)             # required thrust direction in inertial frame, at an angle alpha from rS_N
    h_B = np.matmul(DCM2, sA_B)             # required thrust direction in inertial frame, at an angle alpha from rS_N

    testFailCount = 0                                        # zero unit test result counter
    testMessages = []                                        # create empty array to store test log messages
    unitTaskName = "unitTask"                                # arbitrary name (don't change)
    unitProcessName = "TestProcess"                          # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.1)                   # process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    threeAxConfig = threeAxesPoint.threeAxesPointConfig()   
    threeAxWrap = unitTestSim.setModelDataWrap(threeAxConfig)
    threeAxWrap.ModelTag = "threeAxesPoint"                   # python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, threeAxWrap, threeAxConfig)

    # Initialize the test module configuration data
    # These will eventually become input messages
    threeAxConfig.a1_B = sA_B
    threeAxConfig.priorityFlag = priorityFlag

    # Create input message
    sigma_BN = np.array([0, 0, 0])
    BN = rbk.MRP2C(sigma_BN)
    rS_B = np.matmul(BN, rS_N)
    NavAttMessageData = messaging.NavAttMsgPayload()     
    NavAttMessageData.sigma_BN = sigma_BN
    NavAttMessageData.vehSunPntBdy = rS_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    threeAxConfig.attNavInMsg.subscribeTo(NavAttMsg)

    if flagB == 0:
        threeAxConfig.h1_B = h_B
    else:
        # Create input bodyHeadingMsg
        bodyHeadingData = messaging.BodyHeadingMsgPayload()
        bodyHeadingData.rHat_XB_B = h_B
        bodyHeadingMsg = messaging.BodyHeadingMsg().write(bodyHeadingData)
        threeAxConfig.bodyHeadingInMsg.subscribeTo(bodyHeadingMsg)

    if flagN == 0:
        threeAxConfig.h_N = h_N
    elif flagN == 1:
        # Create input inertialHeadingMsg
        inertialHeadingData = messaging.InertialHeadingMsgPayload()
        inertialHeadingData.rHat_XN_N = h_N
        inertialHeadingMsg = messaging.InertialHeadingMsg().write(inertialHeadingData)
        threeAxConfig.inertialHeadingInMsg.subscribeTo(inertialHeadingMsg)
    else:
        transNavData = messaging.NavTransMsgPayload()
        transNavData.r_BN_N = [0, 0, 0]
        transNavMsg = messaging.NavTransMsg().write(transNavData)
        threeAxConfig.transNavInMsg.subscribeTo(transNavMsg)
        ephemerisData = messaging.EphemerisMsgPayload()
        ephemerisData.r_BdyZero_N = h_N
        ephemerisMsg = messaging.EphemerisMsg().write(ephemerisData)
        threeAxConfig.ephemerisInMsg.subscribeTo(ephemerisMsg)


    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = threeAxConfig.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    moduleOutput = dataLog.sigma_RN
    sigma_RN = moduleOutput[0]
    RN = rbk.MRP2C(sigma_RN)
    NR = RN.transpose()
    sA_N = np.matmul(NR, sA_B)
    gamma_sim = np.arcsin( abs( np.clip( np.dot(rS_N, sA_N), -1, 1 ) ) )

    # set the filtered output truth states
    if priorityFlag == 0:
        if not unitTestSupport.isDoubleEqual(gamma_sim, gamma_true, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + threeAxWrap.ModelTag + " Module failed incidence angle for flagB = {}, flagN = {} and priorityFlag = {}".format(flagB, flagN, priorityFlag))
    else:
        if not unitTestSupport.isDoubleEqual(gamma_sim, 0, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + threeAxWrap.ModelTag + " Module failed incidence angle for flagB = {}, flagN = {} and priorityFlag = {}".format(flagB, flagN, priorityFlag))
            breakpoint()

    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    threeAxesPointTestFunction(
                 False,
                 0,     # alpha
                 0,     # delta
                 0,           # flagB
                 0,           # flagN
                 0,           # priorityFlag
                 1e-6         # accuracy
               )
