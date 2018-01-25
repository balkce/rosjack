# rosjack
ROS node that connects to JACK Audio Connection Toolkit and provides audio-type topics.

## Description
A configured JACK server requires to be running. For more information on how to do this: https://github.com/jackaudio/jackaudio.github.com/wiki

There are two components

* talkerjack: client that reads from a JACK server, and outputs windowed audio data through the "/jackaudio" topic.

* listenerjack: client that subscribes to "/jackaudio" topic and writes the audio data to a JACK server.

There are two versions of these components: C/C++ version and the Python version. The Python version requires CFFI to connect to the C code to connect to JACK, however it is still too slow and may corrupt audio data.

At the moment, the C/C++ version of talkerjack and listenerjack only handle audio in mono, but the jackaudio topic could handle multiple channels, as long as both talker and listener handle the data in the same manner (interleaved or not).

From testing, it can be observed that the python version do produce some clicking, probably because of some data being lost. JACK is a real-time audio server, so this is to be expected.

## Dependencies
The following libraries, installed by apt-get, are enough to compile the C/C++ version of the nodes:
* libjack-jackd2-dev: JACK development libraries

The Python version of the nodes require the following to be installed by apt-get:
* python-numpy
* python-cffi


