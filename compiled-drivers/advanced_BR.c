/* INCLUDE FILE DECLARATIONS */
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/types.h>
#include <pthread.h>
#include "ioctl.h"
#include "99xx.h"

/* LOCAL VARIABLES DECLARATIONS */

int base_clock_array[22] = {1843200, 2457600, 4915200, 3600000, 7200000, 
			3686400, 6451200, 7372800, 9216000, 12902400,
			25804800, 14745600, 18432000, 36864000, 29491200,
			58982400, 22118400, 44236800, 62500000, 125000000,
			24000000, 12000000};

static void usage()
{
	printf("\nUsage: advanced_BR [option], example: \"advanced_BR -d /dev/ttyF0 -b 0 -m 0 -l 1 -s 16\"\n\n");
	printf("The baud rate formula is: [base clock] / ([DLM]*256 + [DLL]) / [sampling]\n\n");
	printf("-d [dev]	specfiy the MCS 99xx serial port, for example '/dev/ttyF0'\n");
	printf("-b [base clock]	specfiy number of the base clock from table below\n");
	printf("		0:	1843200\n");
	printf("		1:	2457600\n");
	printf("		2:	4915200	(2457600 * 2)\n");
	printf("		3:	3600000\n");
	printf("		4:	7200000 (3600000 * 2)\n");
	printf("		5:	3686400\n");
	printf("		6:	6451200\n");
	printf("		7:	7372800\n");
	printf("		8:	9216000\n");
	printf("		9:	12902400\n");
	printf("		10:	25804800 (12902400 * 2)\n");
	printf("		11:	14745600\n");
	printf("		12:	18432000\n");
	printf("		13:	36864000 (18432000 * 2)\n");
	printf("		14:	29491200\n");
	printf("		15:	58982400 (29491200 * 2)\n");
	printf("		16:	22118400\n");
	printf("		17:	44236800 (22118400 * 2)\n");
	printf("		18:	62500000\n");
	printf("		19:	125000000\n");
	printf("		20:	24000000 (External)\n");
	printf("		21:	12000000 (External)\n");
	printf("-m [DLM]	input the number of Divisor Latch MSB, range 0-255\n");
	printf("-l [DLL]	input the number of Divisor Latch LSB, range 0-255\n");
	printf("-s [sampling]	input the number of sampling clock, range 4-16\n");
	printf("-r [recovery]	recovery the standard baud rate(ignore other parameters)\n\n");

}

int main(int argc, char* argv[])
{

	char	*program;
	int	c;
	char	buf[32];
	int	dev;
	int	base_clock;
	int	dlm;
	int	dll;
	int	sampling_clock;
	int	recovery;

	int	devfd;
	int	custom_baud;
	int	product;
	int	parameter;

	program = strrchr(argv[0], '/');
	program = program ? 1 + program : argv[0];
	
	dev = 0;
	base_clock = 0;
	dlm = 0;
	dll = 1;
	sampling_clock = 16;
	recovery = 0;

	while (EOF != (c = getopt(argc, argv, "d:b:m:l:s:hr"))) {
		switch (c) {
		case 'd':
			strcpy(buf, optarg);
			if (memcmp(buf, "/dev/ttyF", 9) != 0) {
				printf("Not MCS 99xx serial port node.\n");
				return -1;
			}
			dev = 1;
			break;
		case 'b':
			if (atoi(optarg) < 0 || atoi(optarg) > 21) {
				printf("Please select the base clock from 0-21.\n");
				return -1;
			} else {
				base_clock = atoi(optarg);
			}
			break;
		case 'm':
			if (atoi(optarg) < 0 || atoi(optarg) > 255) {
				printf("DLM value must be 0-255.\n");
				return -1;
			} else {
				dlm = atoi(optarg);
			}
			break;
		case 'l':
			if (atoi(optarg) < 0 || atoi(optarg) > 255) {
				printf("DLL value must be 0-255.\n");
				return -1;
			} else {
				dll = atoi(optarg);
			}
			break;
		case 's':
			if (atoi(optarg) < 4 || atoi(optarg) > 16) {
				printf("samlping clock must be 4-16.\n");
				return -1;
			} else {
				sampling_clock = atoi(optarg);
			}			
			break;
		case 'r':
			recovery = 1;			
			break;
		case 'h':
			usage();
			return 0;	
		default:
			usage();
			return 0;					
		}
	}

	if (dev == 0 && recovery == 0) {
		usage();
		return 0;
	}	

	if ((dlm == 0) && (dll == 0)) {
		dll = 1;
	}
	
	devfd = open(buf, O_RDWR);

	if (devfd == -1) {
		printf("Can't open MCS 99xx serial port\n");
		return 0;
	}

	if (ioctl(devfd, IOCTL_GET_PRODUCT, &product) < 0) {
		printf("IOCTL_GET_PRODUCT failed!!!\n");
		return 0;	
	}

	if (!(product == 0 || product == 1)) {
		printf("\nPort %s dose not support.\n", buf);
		return 0;
	}

	if (ioctl(devfd, IOCTL_GET_CUSTOM, &custom_baud) < 0) {
		printf("IOCTL_GET_CUSTOM failed!!!\n");
		return 0;	
	}

	if (recovery == 0) {

		custom_baud = (base_clock_array[base_clock] / (dlm * 256 + dll)) / sampling_clock;

		parameter = 0;
		parameter |= base_clock << 20;		//bit[27:20] = baud base clock
		parameter |= dlm << 12;			//bit[19:12] = DLM
		parameter |= dll << 4;			//bit[11:4] = DLL
		if (sampling_clock == 16)
			sampling_clock = 0;

		parameter |= sampling_clock << 0; 	//bit[3:0] = sampling rate
		
	} else {
	
		custom_baud = 0;

		parameter = 0;
		parameter |= 0 << 20;	//bit[27:20] = baud base clock
		parameter |= 0 << 12;	//bit[19:12] = DLM
		parameter |= 1 << 4;	//bit[11:4] = DLL
		parameter |= 16 << 0; 	//bit[3:0] = sampling rate		
	}

	if (ioctl(devfd, IOCTL_SET_CUSTOM, custom_baud) < 0) {
		printf("IOCTL_SET_CUSTOM failed!!!\n");
		return 0;	
	}

	if (custom_baud == 0)
		printf("\nPort %s support standard baud rate\n", buf);
	else
		printf("\nPort %s support custom baud rate: %d bps\n", buf, custom_baud);


	if (ioctl(devfd, IOCTL_SET_PARAMETER, parameter) < 0) {
		printf("IOCTL_SET_PARAMETER failed!!!\n");
		return 0;	
	}

	printf("The %s will operating in custom baud rate after open(re-open) it.\n\n", buf);

	return 0;
}
