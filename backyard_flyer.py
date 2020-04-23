
import time
from enum import Enum
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    TRANSITIONING = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.z = 3
        z = self.z
        self.target_position = np.array([0.0, 0.0, z])
        self.coordinates = [[0,0,z], [10,0,z], [10,10,z], [0,10,z]]
        self.in_mission = True
        self.state = States.MANUAL
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY,
                               self.velocity_callback)
        self.register_callback(MsgID.STATE,
                               self.state_callback)

    def local_position_callback(self):
        if self.state == States.TAKEOFF:   
            altitude = -1.0 * self.local_position[2]
            if altitude > 0.95 * self.target_position[2]:
                self.draw_square()
        if self.state == States.TRANSITIONING and np.linalg.norm(\
        self.target_position[0:2] - self.local_position[0:2]) < .5:
            if not self.coordinates:
                self.landing_transition()
            elif np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                self.draw_square()

    def velocity_callback(self):
        if self.state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.state == States.MANUAL:
            self.arming_transition()
        elif self.state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.state = States.ARMING
    def takeoff_transition(self):
        print("takeoff transition")
        self.takeoff(self.z)
        self.state = States.TAKEOFF

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.state = States.MANUAL
    
    def draw_square(self):
        self.target_position = self.coordinates.pop()
        print('Approaching Coordinate {}'.format(self.target_position))
        self.cmd_position(self.target_position[0], self.target_position[1], \
                          self.target_position[2], 0.0)
        self.state = States.TRANSITIONING

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        super().start()
        self.stop_log()

if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', 
                             threaded=False, 
                             PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()