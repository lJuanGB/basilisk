import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../')
from desktopFSW_sim import DesktopFSW

import EMM_methodsParser

def run_autocode():
    taskActivityDir = dict()
    taskActivityDir["initOnlyTask"] = str(0)
    taskActivityDir["inertial3DPointTask"] = str(0)
    taskActivityDir["feedbackControlTask"] = str(0)
    TheSim = DesktopFSW()
    outputFileName = 'FSW_autoset'
    str_ConfigData = 'config_data'
    EMM_methodsParser.parseSimAlgorithms(TheSim=TheSim, taskActivityDir=taskActivityDir,
                                         outputCFileName=outputFileName,
                                         str_ConfigData=str_ConfigData,
                                         simTag="TheSim.fswModels",
                                         localPath=os.path.dirname(os.path.abspath(filename)))


if __name__ == "__main__":
    run_autocode()