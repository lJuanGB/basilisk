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
#   Module Name:        solarArrayRotation
#   Author:             Riccardo Calaon
#   Creation Date:      November 1, 2022
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
from Basilisk.utilities import unitTestSupport                   # general support file with common unit test functions
from Basilisk.fswAlgorithms import torqueScheduler               # import the module that is to be tested
from Basilisk.utilities import macros
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
@pytest.mark.parametrize("tSwitch", [5, 10])
@pytest.mark.parametrize("torque1", [1,2])
@pytest.mark.parametrize("torque2", [2,4])
@pytest.mark.parametrize("accuracy", [1e-12])


# update "module" in this function name to reflect the module name
def test_torqueScheduler(show_plots, tSwitch, torque1, torque2, accuracy):
    r"""
    **Validation Test Description**

    This unit test verifies the correctness of the output motor torque :ref:`solarArrayPDController`.
    The inputs provided are reference angle and angle rate, current angle and angle rate, proportional
    gain and derivative gain.

    **Test Parameters**

    Args:
        thetaR (double): reference angle;
        thetaDotR (double): reference angle rate;
        theta (double): current angle;
        thetaDot (double): current angle rate;
        accuracy (float): absolute accuracy value used in the validation tests.

    **Description of Variables Being Tested**

    This unit test checks the correctness of the output motor torque

    - ``motorTorqueOutMsg``

    where in this case the output is one dimensional, since the spinning body is one dimensional.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = torqueSchedulerTestFunction(show_plots, tSwitch, torque1, torque2, accuracy)
    assert testResults < 1, testMessage


def torqueSchedulerTestFunction(show_plots, tSwitch, torque1, torque2, accuracy):

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
    schedulerConfig = torqueScheduler.torqueSchedulerConfig()
    schedulerWrap = unitTestSim.setModelDataWrap(schedulerConfig)
    schedulerWrap.ModelTag = "torqueScheduler"
    schedulerConfig.tSwitch = tSwitch
    unitTestSim.AddModelToTask(unitTaskName, schedulerWrap, schedulerConfig)

    # Create input torque msg 1
    MotorTorque1Data = messaging.ArrayMotorTorqueMsgPayload()
    MotorTorque1Data.motorTorque = [torque1]
    MotorTorque1Msg = messaging.ArrayMotorTorqueMsg().write(MotorTorque1Data)
    schedulerConfig.motorTorque1InMsg.subscribeTo(MotorTorque1Msg)

    # Create input torque msg 2
    MotorTorque2Data = messaging.ArrayMotorTorqueMsgPayload()
    MotorTorque2Data.motorTorque = [torque2]
    MotorTorque2Msg = messaging.ArrayMotorTorqueMsg().write(MotorTorque2Data)
    schedulerConfig.motorTorque2InMsg.subscribeTo(MotorTorque2Msg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = schedulerConfig.motorTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(tSwitch * 2))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    timeData = dataLog.times() * macros.NANO2SEC
    motorTorqueData = dataLog.motorTorque
    motorLockFlagData = dataLog.motorLockFlag

    for i in range(len(timeData)):
        t = timeData[i]
        if not unitTestSupport.isDoubleEqual(motorTorqueData[i][0], torque1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + schedulerWrap.ModelTag + "torqueScheduler module failed unit test on torque #1 at simulation time t = {} \n".format(t))
        if not unitTestSupport.isDoubleEqual(motorTorqueData[i][1], torque2, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + schedulerWrap.ModelTag + "torqueScheduler module failed unit test on torque #2 at simulation time t = {} \n".format(t))
        if t > tSwitch:
            if not unitTestSupport.isDoubleEqual(motorLockFlagData[i][0], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + schedulerWrap.ModelTag + "torqueScheduler module failed unit test on torque #1 lock flag at simulation time t = {} \n".format(t))
            if not unitTestSupport.isDoubleEqual(motorLockFlagData[i][1], 0, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + schedulerWrap.ModelTag + "torqueScheduler module failed unit test on torque #2 lock flag at simulation time t = {} \n".format(t))
        else:
            if not unitTestSupport.isDoubleEqual(motorLockFlagData[i][0], 0, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + schedulerWrap.ModelTag + "torqueScheduler module failed unit test on torque #1 lock flag at simulation time t = {} \n".format(t))
            if not unitTestSupport.isDoubleEqual(motorLockFlagData[i][1], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + schedulerWrap.ModelTag + "torqueScheduler module failed unit test on torque #2 lock flag at simulation time t = {} \n".format(t))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_torqueScheduler( 
                 False,
                 10,
                 1.0,
                 2.0,
                 1e-12        # accuracy
               )
