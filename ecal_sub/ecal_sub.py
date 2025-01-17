import sys
import time

import ecal.core.core as ecal_core
from ecal.core.subscriber import StringSubscriber

def callback(topic_name, msg, time):
    print("Received: {}".format(msg))

if __name__ == "__main__":
   # Init eCAL
   ecal_core.initialize(sys.argv, "Python Hello World Publisher")

   # Create a subscriber that listens on the "hello_world_python_topic"
   sub = StringSubscriber("hello_world_python_topic")

   # Set the Callback
   sub.set_callback(callback)

   while ecal_core.ok():
       time.sleep(0.5)
    
   ecal_core.finalize()