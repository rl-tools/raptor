from cflib.crazyflie import Crazyflie as CrazyflieCFLib
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.crazyflie.commander import SET_SETPOINT_CHANNEL, META_COMMAND_CHANNEL, TYPE_HOVER 
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.console import Console
import cflib.crtp
import time

uri='radio://0/80/2M/E7E7E7E7E7'
cflib.crtp.init_drivers()
with SyncCrazyflie(uri, cf=CrazyflieCFLib()) as scf:
    with MotionCommander(scf, default_height=0.3) as mc:
        time.sleep(3)
        mc.stop()