#!/usr/bin/python

"""Create a JACK client that outputs through ROS topic.
"""

import rospy
import sys
import signal
import os
import jack
import threading
import numpy as np

from rosjack.msg import JackAudio

node_name = 'rosjack_write'
rosjack_array = []
new_rosjack_array = False
tlock = threading.Lock()

# config ROS stuff
def roscallback(data):
    global rosjack_array
    global tlock
    global new_rosjack_array
    
    #tlock.acquire()
    rosjack_array[:] = data.data[:]
    new_rosjack_array = True
    #tlock.release()


rospy.init_node(node_name, anonymous=True)
rospy.Subscriber("jackaudio", JackAudio, roscallback)
print('ROS Node started')


if sys.version_info < (3, 0):
    # In Python 2.x, event.wait() cannot be interrupted with Ctrl+C.
    # Therefore, we disable the whole KeyboardInterrupt mechanism.
    # This will not close the JACK client properly, but at least we can
    # use Ctrl+C.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
else:
    # If you use Python 3.x, everything is fine.
    pass

# By default, use script name without extension as client name:
client = jack.Client(node_name, servername=None)
print('JACK Client started')

if client.status.server_started:
    print("JACK server started")
if client.status.name_not_unique:
    print("unique name {0!r} assigned".format(client.name))

event = threading.Event()

size=client.blocksize
samplerate=client.samplerate

#f = np.linspace(0,size,size+1)/size*(samplerate/2)
#hann = np.hanning(2*size)

@client.set_process_callback
def jackprocess(frames):
    global size
    global samplerate
    global rosjack_array
    global tlock
    global new_rosjack_array
    
    while not new_rosjack_array:
        pass
    #tlock.acquire()
    client.outports[0].get_array()[:] = rosjack_array[:]
    new_rosjack_array = False
    #tlock.release()


@client.set_shutdown_callback
def shutdown(status, reason):
    print("JACK shutdown!")
    print("status:", status)
    print("reason:", reason)
    event.set()


# create ports
client.outports.register("output_1")

with client:
    # When entering this with-statement, client.activate() is called.
    # This tells the JACK server that we are ready to roll.
    # Our jackprocess() callback will start running now.

    # Connect the ports.  You can't do this before the client is activated,
    # because we can't make connections to clients that aren't running.
    # Note the confusing (but necessary) orientation of the driver backend
    # ports: playback ports are "input" to the backend, and capture ports
    # are "output" from it.

    playback = client.get_ports(is_physical=True, is_input=True)
    if not playback:
        raise RuntimeError("No physical playback ports")

    for src, dest in zip(client.outports, playback):
        client.connect(src, dest)

    print("Press Ctrl+C to stop")
    try:
        event.wait()
    except KeyboardInterrupt:
        print("\nInterrupted by user")

# When the above with-statement is left (either because the end of the
# code block is reached, or because an exception was raised inside),
# client.deactivate() and client.close() are called automatically.
