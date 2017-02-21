/* RoombaCommander for Raspberry Pi 3   
 * Adapted from RPIwaveTrigger
 *
 * 2-21-17: Uses two serial ports: hardware port takes incoming Xbee data,
 * USB port transmits to Roomba.  Decodes incoming command packets, 
 * echos back to host, and sends commands to Roomba.
 *
 * Can QUIT or SHUT DOWN RPI on command.
 * Briefly tested SD card writes and timer routines.
 * Got SEGMENTATION FAULT 
 */
using namespace std;

#include <iostream>
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
// #include <wiringPi.h>


#define DISPLAY_STRING

#define ESCAPE 0x1B
#define MAXBUFFER 80

void set_mincount(int inPort, int mcount);
int set_interface_attribs(int inPort, int speed);

unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData);
unsigned char inPacket[MAXBUFFER], inData[MAXBUFFER] = { 137, 0, 200, 3, 232 };
int comError = 0;
unsigned int dataIndex = 0;
int error = 0;


//*****************************************************
//*****************************************************
//********** DELAY FOR # uS WITHOUT SLEEPING **********
//*****************************************************
//*****************************************************
//Using delayMicroseconds lets the linux scheduler decide to jump to another process.  Using this function avoids letting the
//scheduler know we are pausing and provides much faster operation if you are needing to use lots of delays.
void DelayMicrosecondsNoSleep(int delay_us)
{
	long int start_time;
	long int time_difference;
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (1)
	{
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0)
			time_difference += 1000000000;				//(Rolls over every 1 second)
		if (time_difference > (delay_us * 1000))		//Delay for # nS
			break;
	}
}

int main(int argc, char *argv[])
{
	char strReady[] = "COM PORTS OPEN: READY";
	char *outPortName = "/dev/ttyUSB0";
	char *inPortName = "/dev/ttyAMA0";
	int numInBytes = 0;
	int inPort, outPort;
	int wlen;
	char *ptrStartMSG = "\rTesting USB and hardware Serial ports";	
	unsigned char command = 0;

	FILE *fp;    // File pointer 
	double x = 3.14, y = 5.67, z = 99.1;
	char * s = "Hello world";


	DelayMicrosecondsNoSleep(1000);

	// Open for writing the file file.txt 
	if (NULL == (fp = fopen("file.txt", "w"))) {

		// if it doesn't succeed, exit out 
		printf("Couldn't open file.txt\n");
		return 0;
	}
	fprintf(fp, "%f, %f, %f, \"%s\"\n", x, y, z, s);    /* write the CSV data to the file */
	fclose(fp); /* close the file we opened earlier*/

	system("echo 'Testing RPI serial ports ttyUSB0 and ttyAMA0 at 9600 baud'");
	
	inPort = open(inPortName, O_RDWR | O_NOCTTY | O_SYNC);
	if (inPort < 0) {
		system("echo 'Error opening RPI hardware port' ");
		return -1;
	}
	else system("echo 'RPI hardware port open' ");

	outPort = open(outPortName, O_RDWR | O_NOCTTY | O_SYNC);
	if (outPort < 0) {
		system("echo 'Error opening RPI USB port' ");
		return -1;
	}	
	else system("echo 'RPI USB port open' ");
	 
	set_interface_attribs(inPort, B57600);		// XBEE baudrate 57600, 8 bits, no parity, 1 stop bit
	set_interface_attribs(outPort, B115200);	// baudrate 115200, 8 bits, no parity, 1 stop bit
   
	system("echo 'Running Roomba Commander' ");	

	int readyLength = strlen(strReady);
	write(inPort, strReady, readyLength);
	tcdrain(inPort);

	do {
		numInBytes = read(inPort, inPacket, sizeof(inPacket) - 1);
		if (numInBytes > 0)
		{
			int numDataBytes = decodePacket(inPacket, numInBytes, inData);			
			if (numDataBytes != 0)
			{					
				char strReply[MAXBUFFER] = " >Data: "; // TODO: change this to strcpy(), move definition to top of main()
				char strTemp[MAXBUFFER];
				if (numDataBytes < 16)
				{					
					for (int i = 0; i < numDataBytes; i++)
					{
						sprintf(strTemp, "%d, ", inData[i]);
						strcat(strReply, strTemp);
					}				
				}
				else error = 11; 					
				
				int replyLength = strlen(strReply);
				write(inPort, strReply, replyLength);
				tcdrain(inPort);

				command = inData[0];
				if (!command) {
					write(outPort, &inData[2], numDataBytes);
					tcdrain(outPort);
				}
				#define QUIT 0x80
				#define SHUTDOWN 0xA0
				else if (command == QUIT) {
					close(inPort);
					close(outPort);
					exit(0);
				}
				else if (command == SHUTDOWN) {
					close(inPort);
					close(outPort);
					system("sudo shutdown -h now");
					exit(0);
				}
			}
		}	
	} while (1);
}


int set_interface_attribs(int inPort, int speed) {
	struct termios tty;

	if (tcgetattr(inPort, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	// tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	    /* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	    /* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(inPort, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_mincount(int inPort, int mcount) {
	struct termios tty;

	if (tcgetattr(inPort, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(inPort, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}


// DECODER ROUTINE

#define STX 36
#define ETX 13
#define DLE 16

unsigned int decodePacket (unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData) {	
	static unsigned char startFlag = false, escapeFlag = false;
	unsigned char ch;
	unsigned int i = 0, dataLength;

	if (ptrInPacket == NULL || numInBytes == 0 || ptrData == NULL || numInBytes >= MAXBUFFER || dataIndex >= MAXBUFFER)
	{
		startFlag = false;
		escapeFlag = false;
		dataIndex = 0;
		return (0);
	}		
		
	do {		
		ch = ptrInPacket[i++];
	
		// Store next char if packet is valid and board number matches
		if (startFlag && dataIndex < MAXBUFFER) ptrData[dataIndex] = ch;   
			
		// If preceding character wasn't an escape char:
		// check whether it is STX, ETX or DLE,
		// otherwise if board number matches then store and advance for next char
		if (escapeFlag == false || startFlag == false) {
			if (ch == DLE) escapeFlag = true;
			else if (ch == STX) {
				dataIndex = 0;
				startFlag = true;			
			}
			else if (ch == ETX) {
				startFlag = false;				
				dataLength = dataIndex;
				// dataIndex = 0;
				return (dataLength);
			}
			else if (startFlag) dataIndex++;
		}		
		// Otherwise if preceding character was an escape char:	
		else { 
			escapeFlag = false;
			if (startFlag) dataIndex++;  	
		}
	
	} while (i < numInBytes && i < MAXBUFFER && dataIndex < MAXBUFFER);
	return (0);
}	