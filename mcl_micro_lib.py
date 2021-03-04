from ctypes import *
import atexit
from time import sleep
import numpy as np

class MadMicroDrive(): #good
    def __init__(self): #good
        # provide valid path to Madlib.dll. Madlib.h and Madlib.lib should also be in the same folder
        path_to_dll = 'C:/Program Files/Mad City Labs/MicroDrive/MicroDrive.dll' #good
        self.madlib = cdll.LoadLibrary(path_to_dll) #good
        self.handler = self.mcl_start() #good
        atexit.register(self.mcl_close) #good
        self.status = c_ushort
        self.status_pointer = pointer(self.status)
        self.isMoving = c_int
        self.isMoving_pointer = pointer(self.isMoving)
        self.xSteps = c_int
        self.xSteps_pointer = pointer(self.xSteps)
        self.ySteps = c_int
        self.ySteps_pointer = pointer(self.ySteps)

    def mcl_start(self):
        """
        Requests control of a single Mad City Labs Nano-Drive.
        Return Value:
            Returns a valid handle or returns 0 to indicate failure.
        """
        mcl_init_handle = self.madlib['MCL_InitHandle']  #good

        mcl_init_handle.restype = c_int #good
        handler = mcl_init_handle() #good
        if (handler == 0): #good
            print("MCL init error") #good
            return -1 #good
        return handler #good

    #int MCL_MDStatus(unsigned short* status, int handle)
    def get_status(self):
        mcl_get_status = self.madlib['MCL_MDStatus']
        mcl_get_status.restype = c_int
        mcl_get_status(self.status_pointer, c_int(self.handler))
        return self.status.value

    #int MCL_MDStop(unsigned short* status, int handle)
    def stop_move(self):
        mcl_get_status = self.madlib['MCL_MDStop']
        mcl_get_status.restype = c_int
        mcl_get_status(self.status_pointer, c_int(self.handler))
        return self.status.value

    #int MCL_MicroDriveMoveStatus(int * isMoving, int handle)
    def is_moving(self):
        mcl_is_moving = self.madlib['MCL_MicroDriveMoveStatus']
        mcl_is_moving.restype = c_int
        mcl_is_moving(self.isMoving_pointer, c_int(self.handler))
        return self.isMoving.value

    #int MCL_MicroDriveWait(int handle)
    def wait(self):
        print('start wait')
        mcl_wait = self.madlib['MCL_MicroDriveWait']
        mcl_wait.restype = c_int
        mcl_wait(c_int(self.handler))
        print('finished wait')

    #int MCL_MDMoveThreeAxesR(int axis1, double velocity1, double distance1, int rounding1, int axis2, double
    # velocity2, double distance2, int rounding2, int axis3, double velocity3, double distance3, int rounding3,
    # int handle)


    def move_R(self, rel_coordinates, velocity = 0.01, rounding = 1):
        mcl_move_three_axesR = self.madlib['MCL_MDMoveThreeAxesR']
        mcl_move_three_axesR.restype = c_int
        mcl_move_three_axesR(c_int(1), c_double(velocity), c_double(rel_coordinates[0]), c_int(rounding),
                             c_int(2), c_double(velocity), c_double(rel_coordinates[1]), c_int(rounding),
                             c_int(3), c_double(velocity), c_double(rel_coordinates[2]), c_int(rounding),
                             c_int(self.handler))
        self.wait()

    #int MCL_MDCurrentPositionM(unsigned int axis, int *microSteps, int handle)
    def read_position(self, axis):
        mcl_current_position = self.madlib['MCL_MDCurrentPositionM']
        mcl_current_position.restype = c_int
        if axis == 1:
            mcl_current_position(c_uint(axis), self.xSteps_pointer, c_int(self.handler))
            return self.xSteps.value
        if axis == 2:
            mcl_current_position(c_uint(axis), self.ySteps_pointer, c_int(self.handler))
            return self.ySteps.value
        else:
            print('axis needs to be 1 or 2')



    def mcl_read(self, axis_number): #this needs serious revision for the microcontroller
        """
        Read the current position of the specified axis.

        Parameters:
            axis [IN] Which axis to move. (X=1,Y=2,Z=3,AUX=4)
            handle [IN] Specifies which Nano-Drive to communicate with.
        Return Value:
            Returns a position value or the appropriate error code.
        """
        mcl_single_read_n = self.madlib['MCL_SingleReadN'] #int MCL_MDCurrentPositionM(unsigned int axis, int *microSteps, int handle)?
        mcl_single_read_n.restype = c_double
        return mcl_single_read_n(c_uint(axis_number), c_int(self.handler))

    def mcl_write(self, position, axis_number):
        """
        Commands the Nano-Drive to move the specified axis to a position.

        Parameters:
            position [IN] Commanded position in microns.
            axis [IN] Which axis to move. (X=1,Y=2,Z=3,AUX=4)
            handle [IN] Specifies which Nano-Drive to communicate with.
        Return Value:
            Returns MCL_SUCCESS or the appropriate error code.
        """
        mcl_single_write_n = self.madlib['MCL_SingleWriteN']
        mcl_single_write_n.restype = c_int
        error_code = mcl_single_write_n(c_double(position), c_uint(axis_number), c_int(self.handler))

        if (error_code != 0):
            print("MCL write error = ", error_code)
        return error_code

    def goxy(self, x_position, y_position):
        self.mcl_write(x_position, 1)
        self.mcl_write(y_position, 2)

    def goz(self, z_position):
        self.mcl_write(z_position, 3)

    def get_position(self):
        return self.mcl_read(1), self.mcl_read(2), self.mcl_read(3)

    def mcl_close(self):
        """
        Releases control of all Nano-Drives controlled by this instance of the DLL.
        """
        mcl_release_all = self.madlib['MCL_ReleaseAllHandles']
        mcl_release_all()


if __name__ == "__main__":

    Micro1 = MadMicroDrive()



    rel_move = [0,0,0]




    Micro1.mcl_close()

