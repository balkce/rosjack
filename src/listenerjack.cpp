/**
 * ROS agent that reads from ROS topic and outputs to speaker
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <mutex>

#include <jack/jack.h>

/*** ROS libraries ***/
#include "ros/ros.h"
#include "rosjack/Audio.h"
/*** End ROS libraries ***/

jack_port_t *output_port;
jack_client_t *client;

jack_default_audio_sample_t *ros2jack_buffer;
unsigned int buffer_size;
unsigned int buffer_r = 0;
unsigned int buffer_w = 0;

std::mutex mtx;

int jack_callback (jack_nframes_t nframes, void *arg){
	jack_default_audio_sample_t *out = (jack_default_audio_sample_t *)jack_port_get_buffer (output_port, nframes);
	
	mtx.lock();
	for (int i = 0; i < nframes; i++){
		out[i] = ros2jack_buffer[buffer_r];
		buffer_r++;
		if(buffer_r >= buffer_size)
			buffer_r = 0;
	}
	mtx.unlock();
	return 0;
}

void jack_shutdown (void *arg){
	exit (1);
}

void jackaudioCallback(const rosjack::Audio::ConstPtr& msg){
	int msg_size = msg->size;
	
	mtx.lock();
    //outputting only one channel
	for (int i = 0; i < msg_size; i++){
		ros2jack_buffer[buffer_w] = msg->data[i];
		buffer_w++;
		if(buffer_w >= buffer_size)
			buffer_w = 0;
	}
	mtx.unlock();
}

int main (int argc, char *argv[]) {
	/* ROS initialization*/
	
	const char *client_name = "listenerjack";
	
	printf ("Connecting to Jack Server...\n");
	jack_options_t options = JackNoStartServer;
	jack_status_t status;
	
	/* open a client connection to the JACK server */
	client = jack_client_open (client_name, options, &status);
	if (client == NULL){
		/* if connection failed, say why */
		printf ("jack_client_open() failed, status = 0x%2.0x\n", status);
		if (status & JackServerFailed) {
			printf ("Unable to connect to JACK server.\n");
		}
		exit (1);
	}
	
	/* if connection was successful, check if the name we proposed is not in use */
	if (status & JackNameNotUnique){
		client_name = jack_get_client_name(client);
		printf ("Warning: other agent with our name is running, `%s' has been assigned to us.\n", client_name);
	}
	
	/* tell the JACK server to call 'jack_callback()' whenever there is work to be done. */
	jack_set_process_callback (client, jack_callback, 0);
	
	
	/* tell the JACK server to call 'jack_shutdown()' if it ever shuts down,
	   either entirely, or if it just decides to stop calling us. */
	jack_on_shutdown (client, jack_shutdown, 0);
	
	
	/* display the current sample rate. */
	printf ("Engine sample rate: %d\n", jack_get_sample_rate (client));
	
	/* allocating ros-to-jack buffer */
	buffer_size = jack_get_buffer_size (client)*10;
	ros2jack_buffer = (jack_default_audio_sample_t *)malloc(sizeof(jack_default_audio_sample_t)*buffer_size);
	
	/* create the agent input port */
	output_port = jack_port_register (client, "output", JACK_DEFAULT_AUDIO_TYPE,JackPortIsOutput, 0);
	
	/* check that both ports were created succesfully */
	if ((output_port == NULL)) {
		printf("Could not create agent ports. Have we reached the maximum amount of JACK agent ports?\n");
		exit (1);
	}
	
	
	/* Tell the JACK server that we are ready to roll.
	   Our jack_callback() callback will start running now. */
	if (jack_activate (client)) {
		printf ("Cannot activate client.");
		exit (1);
	}
	
	printf ("Agent activated.\n");
	
	/* Connect the ports.  You can't do this before the client is
	 * activated, because we can't make connections to clients
	 * that aren't running.  Note the confusing (but necessary)
	 * orientation of the driver backend ports: playback ports are
	 * "input" to the backend, and capture ports are "output" from
	 * it.
	 */
	printf ("Connecting ports... ");
	 
	/* Assign our input port to a server output port*/
	// Find possible output server port names
	const char **serverports_names;
	serverports_names = jack_get_ports (client, NULL, NULL, JackPortIsPhysical|JackPortIsInput);
	if (serverports_names == NULL) {
		printf("No available physical output (server input) ports.\n");
		exit (1);
	}
	// Connect the first available to our output port
	if (jack_connect (client, jack_port_name (output_port), serverports_names[0])) {
		printf("Cannot connect output port.\n");
		exit (1);
	}
	// free serverports_names variable for reuse in next part of the code
	free (serverports_names);
	
	printf ("done.\n");
	
	/* connect to ROS*/
	printf ("Creating ROSJack_Write node...");
	ros::init(argc, argv, client_name);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("jackaudio", 1000, jackaudioCallback);
	printf (" done.\n");

	/* keep running until stopped by the user */
	ros::spin();
	
	
	/* this is never reached but if the program
	   had some other way to exit besides being killed,
	   they would be important to call.
	*/
	jack_client_close (client);
	exit (0);
}
