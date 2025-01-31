#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import threading

def handle_timer_service(req):
    """Handles incoming service requests."""
    try:
        # Extract input data
        data = req.message.strip()
        xyz, time_value = data.split(',')
        xyz = xyz.strip()
        time_value = int(time_value.strip())

        rospy.loginfo(f"Received request: Waiting for {time_value} seconds before stopping {xyz}.")

        # Run the timer in a separate thread
        timer_thread = threading.Thread(target=wait_and_publish, args=(xyz, time_value))
        timer_thread.start()

        return TriggerResponse(success=True, message=f"Timer started for {time_value} seconds")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

def wait_and_publish(xyz, time_value):
    """Waits for the specified time and then publishes a stop message."""
    rospy.sleep(time_value)
    rospy.loginfo(f"Time is up {xyz}!")

    # Publish stop message
    pub = rospy.Publisher('game_control', String, queue_size=10)
    rospy.sleep(1)  # Ensure the publisher gets registered
    pub.publish("stop")

def timer_service_node():
    """Initializes the ROS service node."""
    rospy.init_node('timer_service')
    service = rospy.Service('timer_service', Trigger, handle_timer_service)
    rospy.loginfo("Timer service is ready.")
    rospy.spin()

if __name__ == "__main__":
    timer_service_node()
