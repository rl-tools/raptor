import roslibpy

class Vicon:
    def __init__(self):
        self.ros = roslibpy.Ros(host='localhost', port=9090)
        self.ros.run(timeout=5)
    def add(self, vehicle, topic):
        listener = roslibpy.Topic(self.ros, topic, 'nav_msgs/Odometry', throttle_rate=10)
        listener.subscribe(vehicle._mocap_callback)