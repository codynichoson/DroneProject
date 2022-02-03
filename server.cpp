#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>


//#include <stdlib.h>
//#include <stdio.h>
#include <fcntl.h>
//#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"
int heartbeat=0;

int main(int argc, char *argv[])
{
  //joystick code
  int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];
	struct js_event js;

	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		printf( "Couldn't open joystick\n" );
		return -1;
	}

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	button = (char *) calloc( num_of_buttons, sizeof( char ) );

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
		, name_of_joystick
		, num_of_axis
		, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */
        read(joy_fd, &js, sizeof(struct js_event));
      	 printf("X: %6d  Y: %6d  X1: %6d Y1: %6d b1%d b2%d\n\r", axis[0], axis[1],axis[3],axis[4],button[0],button[1]);

	//end joystick code


    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr;

    char sendBuff[1025];
    time_t ticks;

  //  listenfd = socket(AF_INET, SOCK_STREAM|SOCK_NONBLOCK, 0);
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000);

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, 10);
	int i=0;
    while(1)
    {
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

	ticks = time(NULL);
	while(1)
	{
        //joystick code
        read(joy_fd, &js, sizeof(struct js_event));
	switch(js.type &~JS_EVENT_INIT)
	{
		case JS_EVENT_AXIS:
			axis[js.number]=js.value;
//			printf("event\n\r");
		break;
		case JS_EVENT_BUTTON:
			button [js.number]=js.value;
		break;

	}
	i++;
	if(i%50000==0)//keep from sending too much data 
	{
       // snprintf(sendBuff, sizeof(sendBuff), "%.24s\r\n", ctime(&ticks));

	int a=-axis[1]/290+128;	
	int b=axis[0]/290+128;
	int c=axis[3]/290+128;	
	int d=-axis[4]/290+128;
	int e=(int)button[0]+1;
	int f=(int)button[1]+1;	
	int g=(int)button[2]+1;	
	int h=(int)button[3]+1;	
        snprintf(sendBuff, sizeof(sendBuff), "%c%c%c%c%c%c%c%c%c\r\n ", a,b,c,d,e,f,g,h,heartbeat++);
		
		

      	 printf("X: %6d Y:%6d X1: %6d Y1: %6d %d %d %d %d heartbeat %d\n\r ", a,b,c,d,e,f,g,h,heartbeat);

		write(connfd, sendBuff, strlen(sendBuff));
	}
	}
        close(connfd);
//        sleep(1);
     }
}
