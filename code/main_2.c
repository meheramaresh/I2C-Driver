#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <time.h>

#define FAILURE			-1 	/* Macro for Failure */
#define SUCCESS			0	/* Macro for Success */

/* EPROM status */
#define EEPROM_READY		0x0A		/* Macro for EEPRO ready state */
#define EEPROM_BUSY		0x0B		/* Macro for EEPRO busy state */

/* IOCTL cases */
#define FLASHGETS		0xA0
#define FLASHGETP		0xB0
#define FLASHSETP		0xC0
#define FLASHERASE		0xD0


/*
 * Main function - This creates all the threads. 
 */
int main(int argc, char **argv){
	int fileHandler;
	int fd, res;
	int count;
	int i = 0;
	short pageNumber;
	char curAddress[2] = {0x00, 0x00};
	char *bufIn;
	char *bufOut;

	fd = open("/dev/i2c_flash", O_RDWR);
	if (fd < 0){
		printf("Can not open device file in writer sender.\n");
		fprintf(stderr, "open() failed: %s\n", strerror(errno));

		return;
	}
	else{
		/* Performs read, write, erase, gets, setp, getp operations */
		if(strcmp("write", argv[1]) == 0){
			count = atoi(argv[2]);
			//printf("No. of pages to write %d\n", count);
			bufIn = (char*)malloc(count * 64);	
			
			i = 0;
			while(i < (count*64)){
				*(bufIn + i) = 'a';
				i++;				
			}
			
			/* Write to the device file */
			res = write(fd, bufIn, count);

			/* Print the result */
			if(res < 0){
				if(errno == EINVAL)
					printf("pages count invalid\n");
				else if(errno == EBUSY)
					printf("Writing Failed\n");		
			}
		}
		else if(strcmp("read", argv[1]) == 0){
			count = atoi(argv[2]);
			printf("count in user %d\n", count);			

			bufOut = (char*)malloc(count * 64);
			res = read(fd, bufOut, count);
			if(res < 0){
				if(errno == EAGAIN)
					printf("Please try Again\n");		
				else if(errno == EBUSY)
					printf("EEPROM busy\n");
				else if(errno == EINVAL)
					printf("Pages count invalid\n");
			}
			else{
				printf("\nUser Read String - ");
				i = 0;
				while(i < (count*64)){	
					printf("%c", *(bufOut + i));
					i++;
				}
				printf("\n");
			}
		}
		/* Ioctl operations */
		else if(strcmp("flasherase", argv[1]) == 0){
			printf("Please wait.. \n");
			res = ioctl(fd, FLASHERASE);
		}
		else if(strcmp("flashgetp", argv[1]) == 0){
			res = ioctl(fd, FLASHGETP);
			printf("Current address 0x%x\n", res);
		}
		else if(strcmp("flashsetp", argv[1]) == 0){
			pageNumber = atoi(argv[2]);
			res = ioctl(fd, FLASHSETP, pageNumber);
			if(res == SUCCESS)
				printf("Moving to page %d\n", pageNumber);
			else
				printf("Please try again\n");

		}
		else if(strcmp("flashgets", argv[1]) == 0){
			res = ioctl(fd, FLASHGETS);
			if(res == EEPROM_READY)
				printf("EEPROM READY\n");
			else
				printf("EEPROM BUSY\n");
		}
	
		/* Close the device */		
		close(fd);
	}
	return 0;
}


