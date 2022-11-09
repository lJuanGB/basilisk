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
#   Module Name:        platformRotation
#   Author:             Riccardo Calaon
#   Creation Date:      October 29, 2022
#

import pytest
import os, inspect, random
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)



# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.fswAlgorithms import platformRotation                # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("seed", list(np.linspace(1,10,10)))
@pytest.mark.parametrize("CM_offset", [0.1, 0.2, 0.3])
@pytest.mark.parametrize("accuracy", [1e-7])

# update "module" in this function name to reflect the module name
def test_platformRotation(show_plots, CM_offset, seed, accuracy):
    r"""
    **Validation Test Description**

    
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = platformRotationTestFunction(show_plots, CM_offset, seed, accuracy)
    assert testResults < 1, testMessage


def platformRotationTestFunction(show_plots, CM_offset, seed, accuracy):

    random.seed(seed)

    sigma_MB = np.array([0., 0., 0.])
    r_BM_M = np.array([0.0, 0.1, 0.4])
    r_FM_F = np.array([0.0, 0.0, -0.1])
    r_TF_F = np.array([-0.01, 0.03, 0.02])
    T_F    = np.array([1.0, 1.0, 10.0])

    T_F_hat = T_F / np.linalg.norm(T_F)

    r_CB_B = np.array([0,0,0]) + np.random.rand(3)
    r_CB_B = r_CB_B / np.linalg.norm(r_CB_B) * CM_offset

    testFailCount = 0                        # zero unit test result counter
    testMessages = []                        # create empty array to store test log messages
    unitTaskName = "unitTask"                # arbitrary name (don't change)
    unitProcessName = "TestProcess"          # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    platformConfig = platformRotation.platformRotationConfig()                     # update with current values
    platformWrap = unitTestSim.setModelDataWrap(platformConfig)
    platformWrap.ModelTag = "platformRotation"                                     # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, platformWrap, platformConfig)

    # Initialize the test module configuration data
    platformConfig.sigma_MB = sigma_MB
    platformConfig.r_BM_M = r_BM_M
    platformConfig.r_FM_F = r_FM_F

    # Create input VehicleConfigMsg
    vehConfigData = messaging.VehicleConfigMsgPayload()    # Create a structure for the input message
    vehConfigData.CoM_B = r_CB_B                 
    vehConfigMsg = messaging.VehicleConfigMsg().write(vehConfigData)
    platformConfig.vehConfigInMsg.subscribeTo(vehConfigMsg)

    # Create input THRConfigMsg
    THRConfigData = messaging.THRConfigMsgPayload()    # Create a structure for the input message
    THRConfigData.rThrust_B = r_TF_F
    THRConfigData.tHatThrust_B = T_F_hat
    THRConfigData.maxThrust = np.linalg.norm(T_F)
    THRConfigMsg = messaging.THRConfigMsg().write(THRConfigData)
    platformConfig.thrConfigFInMsg.subscribeTo(THRConfigMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    anglesLog = platformConfig.platformAnglesOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, anglesLog)
    thrDataLog = platformConfig.thrConfigBOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, thrDataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    alpha = anglesLog.alpha[0]
    beta = anglesLog.beta[0]

    T_B_hat_output = thrDataLog.tHatThrust_B[0]
    r_TB_B_output = thrDataLog.rThrust_B[0]

    FM = [[np.cos(beta),  np.sin(alpha)*np.sin(beta), -np.cos(alpha)*np.sin(beta)],
                   [      0     ,        np.cos(alpha)       ,        np.sin(alpha)       ],
                   [np.sin(beta), -np.sin(alpha)*np.cos(beta),  np.cos(alpha)*np.cos(beta)]]

    MB = rbk.MRP2C(sigma_MB)

    FB = np.matmul(FM, MB)

    r_CB_M = np.matmul(MB, r_CB_B)
    r_CM_M = r_CB_M + r_BM_M
    r_CM_F = np.matmul(FM, r_CM_M)
    r_CT_F = r_CM_F - r_FM_F - r_TF_F

    r_TB_B = np.matmul(FB.transpose(), r_TF_F + r_FM_F) - np.matmul(MB.transpose(), r_BM_M)

    offset = np.arccos( min( max( np.dot(r_CT_F, np.array(T_F)) / np.linalg.norm(np.array(r_CT_F)) / np.linalg.norm(np.array(T_F)), -1), 1) )

    T_B_hat = np.matmul(FB.transpose(), T_F_hat)

    # check that the offset between thrust direction and CM is zero
    if not unitTestSupport.isDoubleEqual(offset, 0.0, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + platformWrap.ModelTag + "platformRotation module failed zero offset unit test\n")

    # check that the thrust direction in body frame coordinates is correct
    if not unitTestSupport.isVectorEqual(T_B_hat_output, T_B_hat, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + platformWrap.ModelTag + "platformRotation module failed output thrust direction unit test\n")

    # check that the thrust application point in B frame coordinates is correct
    if not unitTestSupport.isVectorEqual(r_TB_B_output, r_TB_B, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + platformWrap.ModelTag + "platformRotation module failed output thrust application point unit test\n")

    # check that the output max trust is correct
    if not unitTestSupport.isDoubleEqual(THRConfigData.maxThrust, thrDataLog.maxThrust[0], accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + platformWrap.ModelTag + "platformRotation module failed maximum thrust unit test\n")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_platformRotation(              # update "module" in function name
                 False,
                 0.1,
                 np.random.rand(1)[0],
                 1e-7        # accuracy
               )
