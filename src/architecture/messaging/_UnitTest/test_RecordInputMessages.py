#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Test if a C-wrapped input message can be logged with a recorder module
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 3, 2023
#


from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport as uts


def test_RecordingInputMessages():
    """
    testing recording a C-wrapped input message with the recorder module
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cModuleTemplate.cModuleTemplateConfig()
    mod1Wrap = scSim.setModelDataWrap(mod1)
    mod1Wrap.ModelTag = "cModule1"
    scSim.AddModelToTask("dynamicsTask", mod1Wrap, mod1)

    # Write input data
    inputData = messaging.CModuleTemplateMsgPayload()
    inputData.dataVector = [1, 2, 3]
    inputDataMsg = messaging.CModuleTemplateMsg().write(inputData)

    # Subscribe input message to stand alone message
    mod1.dataInMsg.subscribeTo(inputDataMsg)

    # Create recorders tied to IO messages
    dataInRec = mod1.dataInMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataInRec)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    testFailCount, testMessages = uts.compareArray([inputData.dataVector]*2
                                                   , dataInRec.dataVector
                                                   , 0.01
                                                   , "recorded input message was not correct."
                                                   , testFailCount
                                                   , testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_RecordingInputMessages()
