import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../')
from desktopFSW_sim import DesktopFSW

import methodsParser


def empty_sets_folder(outputPath):
    if os.path.exists(outputPath):
        print "Cleaning directory: %s" % outputPath
        for data_file in os.listdir(outputPath):
            os.remove(outputPath + '/' + data_file)
    else:
        print "Creating new directory: %s" % outputPath
        os.makedirs(outputPath)

def run_autocode():
    taskActivityDir = dict()
    taskActivityDir["initOnlyTask"] = str(0)
    taskActivityDir["inertial3DPointTask"] = str(0)
    taskActivityDir["feedbackControlTask"] = str(0)
    TheSim = DesktopFSW()
    outputFileName = 'FSW_autoset'
    str_ConfigData = 'config_data'
    output_path = os.path.dirname(os.path.abspath(filename)) + "/sets"
    empty_sets_folder(outputPath=output_path)
    methodsParser.parseSimAlgorithms(TheSim=TheSim, taskActivityDir=taskActivityDir,
                                     outputCFileName=outputFileName,
                                     str_ConfigData=str_ConfigData,
                                     simTag="TheSim.fswModels",
                                     localPath=output_path)


if __name__ == "__main__":
    run_autocode()