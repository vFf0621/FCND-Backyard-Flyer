
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
        Drone.__init__(self, connection)
        self.z = 3
        z = self.z
        self.target_position = [0.0, 0.0, z]
        self.coordinates = [[0.,0.,z], [10.,0.,z], [10.,10.,z], [0.,10.,z]]
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
            if altitude >= self.target_position[2]*.97:
                self.waypoint_transition()
        if self.state == States.TRANSITIONING and np.linalg.norm(\
        self.target_position[0:2] - self.local_position[0:2]) < .5:
            if not self.coordinates:
                self.landing_transition()
            elif np.linalg.norm(self.local_velocity[0:2]) < .5:
                self.waypoint_transition()

    def velocity_callback(self):
        if self.state == States.LANDING and self.local_position[2] < 0.5:
            self.disarming_transition()

    def state_callback(self):
        
        if self.state == States.MANUAL: 
            self.arming_transition()
        elif self.state != States.MANUAL:
            return None
        if self.state == States.ARMING and self.armed:
            self.takeoff_transition()
        elif self.state == States.DISARMING and not self.armed:
            self.manual_transition()

    def arming_transition(self):
        print("Arming")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.state = States.ARMING
        
    def takeoff_transition(self):
        print("Takeoff")
        self.takeoff(self.z)
        self.state = States.TAKEOFF

    def landing_transition(self):
        print("landing")
        self.land()
        self.state = States.LANDING

    def disarming_transition(self):
        print("Disarmed")
        self.disarm()
        self.state = States.DISARMING

    def manual_transition(self):
        print("Manual")
        self.release_control()
        self.stop()
        self.state = States.MANUAL
    
    def waypoint_transition(self):
        self.target_position = self.coordinates.pop()
        print('Approaching Coordinate {}'.format(self.target_position))
        self.cmd_position(self.target_position[0], self.target_position[1], \
                          self.target_position[2], 0.0)
        self.state = States.TRANSITIONING

    def launch(self):
        self.start_log("Logs", "NavLog.txt")
        print("Launching")
        self.start()
        self.stop_log()

if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', 
                             threaded=False, 
                             PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.launch()