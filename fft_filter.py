#!/usr/bin/python

"""Create a JACK client that filters output.

If you have a microphone and loudspeakers connected, this might cause an
acoustical feedback!

"""
import sys
import signal
import os
import jack
import threading
import numpy as np

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
defaultclientname = 'fft_filter'
clientname = defaultclientname
servername = None
print('client:',clientname)
print('server:',servername)
print('arguments:', sys.argv)

client = jack.Client(clientname, servername=servername)

if client.status.server_started:
    print("JACK server started")
if client.status.name_not_unique:
    print("unique name {0!r} assigned".format(client.name))

event = threading.Event()

size=client.blocksize
samplerate=client.samplerate

f = np.linspace(0,size,size+1)/size*(samplerate/2)
o_signal1=np.zeros(size)

i_signal1=np.zeros(2*size) #concatenated filtered past 2 windows
i_signal2=np.zeros(2*size) #last window in first half

hann = np.hanning(2*size)

def filter_signal(signal):
    global hann
    global f
    fsignal=np.fft.rfft(signal*hann)
    
    # do magic FFT stuff here
    #fsignal[(f>1000) & (f<4000)] *= 0.0 #band reject
    fsignal[(f<=1000) | (f>=4000)] *= 0.0 #band pass
    
    return np.fft.irfft(fsignal)


@client.set_process_callback
def process(frames):
    global size
    global samplerate
    global i_signal1
    global i_signal2
    global o_signal1
    
    current_window = client.inports[0].get_array()
    
    #filling the second half with current window
    i_signal2[size:] = current_window[:]
    
    #filtering 2nd buffer
    i_signal2[:] = filter_signal(i_signal2)

	#overlap-add
    o_signal1[:] = i_signal1[size:]+i_signal2[:size]
    
    #copying 2nd buffer to 1st buffer (slicing, to force cloning)
    i_signal1[:] = i_signal2[:]
    #copying current window to 2nd buffer
    i_signal2[:size] = current_window[:]

	#outputing overlap-add result
    client.outports[0].get_array()[:]=o_signal1
    client.outports[1].get_array()[:]=o_signal1

@client.set_shutdown_callback
def shutdown(status, reason):
    print("JACK shutdown!")
    print("status:", status)
    print("reason:", reason)
    event.set()


# create ports
client.inports.register("input_1")
client.outports.register("output_1")
client.outports.register("output_2")

with client:
    # When entering this with-statement, client.activate() is called.
    # This tells the JACK server that we are ready to roll.
    # Our process() callback will start running now.

    # Connect the ports.  You can't do this before the client is activated,
    # because we can't make connections to clients that aren't running.
    # Note the confusing (but necessary) orientation of the driver backend
    # ports: playback ports are "input" to the backend, and capture ports
    # are "output" from it.

    capture = client.get_ports(is_physical=True, is_output=True)
    if not capture:
        raise RuntimeError("No physical capture ports")

    for src, dest in zip(capture, client.inports):
        client.connect(src, dest)

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
