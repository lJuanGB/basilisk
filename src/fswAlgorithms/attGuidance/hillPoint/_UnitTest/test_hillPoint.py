#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        hillPoint
#   Author:             Mar Cols
#   Creation Date:      January 22, 2016
#

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import hillPoint  # import the module that is to be tested
# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import astroFunctions as af
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("celMsgSet", [True, False])

def test_hillPoint(show_plots, celMsgSet):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = hillPointTestFunction(show_plots, celMsgSet)
    assert testResults < 1, testMessage


def hillPointTestFunction(show_plots, celMsgSet):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = hillPoint.hillPointConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "hillPoint"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data

    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
    r_BN_N = r
    v_BN_N = v
    planetPos = np.array([0.0, 0.0, 0.0])
    planetVel = np.array([0.0, 0.0, 0.0])

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    #
    #   Navigation Input Message
    #
    NavStateOutData = messaging.NavTransMsgPayload()  # Create a structure for the input message
    NavStateOutData.r_BN_N = r_BN_N
    NavStateOutData.v_BN_N = v_BN_N
    navMsg = messaging.NavTransMsg().write(NavStateOutData)
    moduleConfig.transNavInMsg.subscribeTo(navMsg)

    #
    #   Spice Input Message
    #
    if (celMsgSet):
        CelBodyData = messaging.EphemerisMsgPayload()
        CelBodyData.r_BdyZero_N = planetPos
        CelBodyData.v_BdyZero_N = planetVel
        celBodyMsg = messaging.EphemerisMsg().write(CelBodyData)
        moduleConfig.celBodyInMsg.subscribeTo(celBodyMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = moduleConfig.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    #
    # check sigma_RN
    #
    moduleOutput = dataLog.sigma_RN

    # set the filtered output truth states
    trueVector = [
               [0.,              0.,              0.267949192431],
               [0.,              0.,              0.267949192431],
               [0.,              0.,              0.267949192431]
               ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed sigma_RN unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
                                "sec\n")
    #
    # check omega_RN_N
    #
    moduleOutput = dataLog.omega_RN_N
    # set the filtered output truth states
    trueVector = [
               [0.,              0.,              0.000264539877],
               [0.,              0.,              0.000264539877],
               [0.,              0.,              0.000264539877]
               ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed omega_RN_N unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
                                "sec\n")
    #
    # check domega_RN_N
    #
    moduleOutput = dataLog.domega_RN_N
    # set the filtered output truth states
    trueVector = [
               [0.0, 0.0, 1.315647475046e-23],
               [0.0, 0.0, 1.315647475046e-23],
               [0.0, 0.0, 1.315647475046e-23]
               ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed domega_RN_N unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
                                "sec\n")

    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_hillPoint(False, True)
