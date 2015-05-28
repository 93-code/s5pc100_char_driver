#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// open
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// perror
#include <errno.h>

// ./led_app /dev/led0
int main(int argc, const char *argv[])
{
	int fd;
	
	if (argc < 2){
		fprintf(stderr, "Usage: %s </dev/ledx>\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	
	fd = open(argv[1], O_RDWR);
	if (-1 == fd){
		perror("open fail");
		exit(EXIT_FAILURE);
	}
	
	sleep(2);
	
	close(fd);
	
	return 0;
}
