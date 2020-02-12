from mpy_cpp_wrapper import CppWrapperClass
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../')
from desktopFSW_sim import DesktopFSW



def empty_wraps_folder(outputPath):
    if os.path.exists(outputPath):
        print "Cleaning directory: %s" % outputPath
        for plot_file in os.listdir(outputPath):
            os.remove(outputPath + '/' + plot_file)
    else:
        print "Creating new directory: %s" % outputPath
        os.makedirs(outputPath)


def run_auto_wrapper():
    taskActivityDir = dict()
    taskActivityDir["initOnlyTask"] = str(0)
    taskActivityDir["inertial3DPointTask"] = str(0)
    taskActivityDir["feedbackControlTask"] = str(0)
    TheSim = DesktopFSW()
    outputPath = os.path.dirname(os.path.abspath(filename)) + "/wraps"
    empty_wraps_folder(outputPath=outputPath)
    wrapper = CppWrapperClass(TheSim=TheSim, taskActivityDir=taskActivityDir,
                              simTag="TheSim.fswModels", outputPath=outputPath)


if __name__ == "__main__":
    run_auto_wrapper()





