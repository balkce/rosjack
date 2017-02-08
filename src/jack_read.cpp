/**
 * ROS agent that reads from microphone and outputs to ROS topic
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <jack/jack.h>

/*** ROS libraries ***/
#include "ros/ros.h"
#include "rosjack/JackAudio.h"
#include "std_msgs/Float32.h"
/*** End ROS libraries ***/

jack_port_t **input_port;
jack_client_t *client;

ros::Publisher rosjack_out;

int number_of_microphones;
bool verbose;
int max_mic_past = -1;

int jack_callback (jack_nframes_t nframes, void *arg){
	int i,j;
	int max_mic_i = -1;
	double max_mic_energy = 0;
	double this_mic_energy = 0;
	
	jack_default_audio_sample_t **in = (jack_default_audio_sample_t **)malloc(sizeof(jack_default_audio_sample_t *)*number_of_microphones);
	for (i = 0; i < number_of_microphones; i++){
		in[i] = (jack_default_audio_sample_t *)jack_port_get_buffer (input_port[i], nframes);
		
		this_mic_energy = 0;
		for(j = 0;j < nframes; j++)
			this_mic_energy += abs((double)(in[i][j]*100));
		
		//printf("%d %f      ",i,this_mic_energy);fflush(stdout);
		if (this_mic_energy > max_mic_energy || max_mic_past == -1){
			max_mic_energy = this_mic_energy;
			max_mic_i = i;
			
			if(max_mic_past == -1)
				max_mic_past = max_mic_i;
		}
	}
	
	if (max_mic_i == -1)
		max_mic_i = max_mic_past;
	
	//printf(" active %d\n",max_mic_i);fflush(stdout);
	rosjack::JackAudio out;
	out.size = nframes;
	for (j = 0; j < nframes; j++){
		out.data.push_back(in[max_mic_i][j]);
	}
	rosjack_out.publish(out);
	
	if(verbose && max_mic_past != max_mic_i)
		printf("Microphone %d now active.\n",max_mic_i+1);
	
	max_mic_past = max_mic_i;
	return 0;
}

void jack_shutdown (void *arg){
	exit (1);
}

int main (int argc, char *argv[]) {
	/* ROS initialization*/
	
	const char *client_name = "rosjack_read";
	
	printf ("Creating ROSJack_Read node...\n");
	ros::init(argc, argv, client_name);
	ros::NodeHandle n;
	rosjack_out = n.advertise<rosjack::JackAudio>("jackaudio", 1000);

	// Obtaining parameters from ROS parameter server
	if (n.getParam("/rosjack_read/number_of_microphones",number_of_microphones)){
		ROS_INFO("Number of microphones: %d",number_of_microphones);
	}else{
		number_of_microphones = 1;
		ROS_WARN("Number of microphones argument not found in ROS param server, using default value (%d).",number_of_microphones);
	}
	
	if (n.getParam("/rosjack_read/verbose",verbose)){
		ROS_INFO("Verbose: %d",verbose);
	}else{
		verbose = false;
		ROS_WARN("Verbosity argument not found in ROS param server, using default value (%d).",verbose);
	}


	/* JACK initialization*/
	int i;
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
	
	
	/* create the agent input ports */
	input_port = (jack_port_t **)malloc(sizeof(jack_port_t *)*number_of_microphones);
	char input_port_name[100];
	for(i = 0; i < number_of_microphones; i++){
		sprintf(input_port_name,"input_%d",i);
		input_port[i] = jack_port_register (client, input_port_name, JACK_DEFAULT_AUDIO_TYPE,JackPortIsInput, 0);

		/* check that the port were created succesfully */
		if ((input_port[i] == NULL)) {
			printf("Could not create agent ports. Have we reached the maximum amount of JACK agent ports?\n");
			exit (1);
		}
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
	serverports_names = jack_get_ports (client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput);
	if (serverports_names == NULL) {
		printf("No available physical capture (server output) ports.\n");
		exit (1);
	}
	// Connect the first available to our input port
	for(i = 0; i < number_of_microphones; i++){
		if (jack_connect (client, serverports_names[i], jack_port_name (input_port[i]))) {
			printf("Cannot connect input port %s.\n",jack_port_name (input_port[i]));
			exit (1);
		}
	}
	// free serverports_names variable for reuse in next part of the code
	free (serverports_names);
	
	
	
	printf ("done.\n");
	if(verbose)
		printf("Microphone 1 now active.\n");
	/* keep running until stopped by the user */
	ros::spin();
	
	
	/* this is never reached but if the program
	   had some other way to exit besides being killed,
	   they would be important to call.
	*/
	jack_client_close (client);
	exit (0);
}
