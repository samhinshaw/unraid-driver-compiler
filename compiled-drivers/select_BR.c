/* INCLUDE FILE DECLARATIONS */
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <linux/types.h>
#include <pthread.h>
#include "ioctl.h"
#include "99xx.h"

/* LOCAL VARIABLES DECLARATIONS */

int main(int argc, char* argv[])
{

	char dev[16];
	int  select;
	int  custom_baud;
	int  product;
	int  parameter;

	int devfd;

	while (1) {

		printf("Please input the port of MCS99xx. (ex. /dev/ttyF0):");
		scanf("%s", dev);

		if (memcmp(dev, "/dev/ttyF", 9) != 0)
			printf("Wrong input!!\n");
		else
			break;
	}

	//memcpy(dev, "/dev/ttyF0", 10);

	devfd = open(dev, O_RDWR);
	
	if (devfd == -1) {
		printf("Can't open %s\n", dev);
		return 0;
	}

	if (ioctl(devfd, IOCTL_GET_PRODUCT, &product) < 0) {
		printf("IOCTL_GET_PRODUCT failed!!!\n");
		return 0;	
	}

	if (product == 2) {
		printf("\nPort %s is MCS9900 serial port.\n",dev);
	} else {
		printf("\nPort %s dose not support.\n", dev);
		return 0;
	}

	if (ioctl(devfd, IOCTL_GET_CUSTOM, &custom_baud) < 0) {
		printf("IOCTL_GET_CUSTOM failed!!!\n");
		return 0;	
	}

	if (custom_baud == 0)
		printf("\nPort %s support standard baud rate\n", dev);
	else
		printf("\nPort %s support custom baud rate:%d\n", dev, custom_baud);

	if (product == 2) { // MCS9900
		while (1) {

			printf("\nPlease specify the baud rate of %s.\n", dev);
			printf("0  : Standard baud rate\n");
			printf("1  : 230400\n");
			printf("2  : 403200\n");
			printf("3  : 460800\n");
			printf("4  : 806400\n");
			printf("5  : 921600\n");
			printf("6  : 1500000\n");
			printf("7  : 3000000\n");
			printf("8  : 6000000\n");
			printf("9  : 6400000\n");
			printf("10 : 8000000\n");
			printf("11 : 9600000\n");
			printf("12 : 12000000\n");
			printf("13 : 16000000\n");
			printf("99 : Exit\n");
			printf(":");
			scanf("%d", &select);


			if (select == 1) {
				custom_baud = 230400;
				break;
			} else if (select == 2) {	
				custom_baud = 403200;
				break;
			} else if (select == 3) {
				custom_baud = 460800;
				break;
			} else if (select == 4) {	
				custom_baud = 806400;
				break;
			} else if (select == 5) {	
				custom_baud = 921600;
				break;
			} else if (select == 6) {	
				custom_baud = 1500000;
				break;
			} else if (select == 7) {	
				custom_baud = 3000000;
				break;
			} else if (select == 8) {	
				custom_baud = 6000000;
				break;
			} else if (select == 9) {	
				custom_baud = 6400000;
				break;
			} else if (select == 10) {	
				custom_baud = 8000000;
				break;
			} else if (select == 11) {	
				custom_baud = 9600000;
				break;
			} else if (select == 12) {	
				custom_baud = 12000000;
				break;
			} else if (select == 13) {	
				custom_baud = 16000000;
				break;
			} else if (select == 0) {	
				custom_baud = 0;
				break;
			} else if (select == 99) {
				return 0;
			}
		}
	}
	
	if (ioctl(devfd, IOCTL_SET_CUSTOM, custom_baud) < 0) {
		printf("IOCTL_SET_CUSTOM failed!!!\n");
		return 0;	
	}

	if (ioctl(devfd, IOCTL_SET_PARAMETER, parameter) < 0) {
		printf("IOCTL_SET_PARAMETER failed!!!\n");
		return 0;	
	}

	printf("The %s will operating in custom baud rate after open(re-open) it.\n", dev);

exit:

	printf("\n");

	return 0;
}
