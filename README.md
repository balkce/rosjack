# rosjack
ROS node that connects to JACK Audio Connection Toolkit and provides audio-type topics.

## Description
A configured JACK server requires to be running. For more information on how to do this: https://github.com/jackaudio/jackaudio.github.com/wiki

There are two components

* jack_read: client that reads from a JACK server, and outputs windowed audio data through the "/jackaudio" topic.

* jack_write: client that subscribes to "/jackaudio" topic and writes the audio data to a JACK server.

There are two versions of these components: C/C++ version and the Python version. The Python version requires CFFI to connect to the C code to connect to JACK, however it is still too slow and may corrupt audio data.

At the moment, the C/C++ version of jack_read can handle multiple inputs however it writes into the jackaudio topic the audio data of the input with the highest energy.

In addition, the C/C++ version of jack_write writes the audio data received from the jackaudio topic directly to JACK from its sole output.

## Dependencies
The following libraries, installed by apt-get, are enough to compile the C/C++ version of the nodes:
* libjack-jackd2-dev: JACK development libraries

The Python version of the nodes require the following to be installed by apt-get:
* python-numpy:
* python-cffi


