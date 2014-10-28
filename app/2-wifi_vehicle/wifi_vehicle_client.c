

#include <sys/types.h>          /* See NOTES */
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <curses.h>
#include <fcntl.h>
#include <string.h>

/* socket
 * connect
 * send/recv
 */
#define SERVER_PORT                         8888

#define ESC                                 27          // char 'esc'
#define ENTER                               13          // char 'enter'
#define ACCELERATE                          91          // char '['
#define DECELERATE                          93          // char ']'
#define SPACE                               32          // char 'space'
#define TEMPERATURE                         116         // char 't'

/* request type */
#define REQ_CMD_TYPE_DIRECTION              0
#define REQ_CMD_TYPE_BUZZER                 1
#define REQ_CMD_TYPE_SPEED                  2
#define REQ_CMD_TYPE_TEMPERATURE            3

#define WIFI_VEHICLE_MOVE_FORWARD           0
#define WIFI_VEHICLE_MOVE_BACKWARD          1
#define WIFI_VEHICLE_MOVE_LEFT              2
#define WIFI_VEHICLE_MOVE_RIGHT             3
#define WIFI_VEHICLE_STOP                   4

#define WIFI_VEHICLE_BUZZER_ON              5
#define WIFI_VEHICLE_BUZZER_OFF             6

struct reqMsg {
    unsigned char type;
    unsigned char dir;
    unsigned char speed;        /* 0 ~ 100 */
    float temp;
};

static int connectToServer(int *iSocket, char *argv[])
{
	struct sockaddr_in tSocketServerAddr;
	int iRet;

	*iSocket = socket(AF_INET, SOCK_STREAM, 0);

	tSocketServerAddr.sin_family      = AF_INET;
	tSocketServerAddr.sin_port        = htons(SERVER_PORT);  /* host to net, short */
 	//tSocketServerAddr.sin_addr.s_addr = INADDR_ANY;
 	if (0 == inet_aton(argv[1], &tSocketServerAddr.sin_addr)) {
		printf("invalid server_ip\n");
		return -1;
	}
    
	memset(tSocketServerAddr.sin_zero, 0, 8);


	iRet = connect(*iSocket, (const struct sockaddr *)&tSocketServerAddr, sizeof(struct sockaddr));	
	if (-1 == iRet) {
		printf("connect error!\n");
		return -1;
	}
    
    return 0;
}

static void libncurses_init(void)
{
    /* Initialize ncurses */
    initscr();

    /* Initialize keypad */
    nonl();                         /* Set the return char without using newline char */
    intrflush(stdscr, FALSE);       /* Whether to deal with interrupt signal */
    keypad(stdscr, TRUE);           /* Enable keypad functionality so that the system will receive the up,down,left,right keys */
}

int main(int argc, char *argv[])
{
	int iSocketClient;
	int iSendLen;
    int key;
    int isOn = 0;
    unsigned char dutyCycle = 30;
    struct reqMsg request;

	if (argc != 2) {
		printf("Usage:\n");
		printf("%s <server_ip>\n", argv[0]);
		return -1;
	}
    
    connectToServer(&iSocketClient, argv);

    /* Initialize libncurses */    
    libncurses_init();

    /* Control the wifi vehicle by using ioctl */    
    while ( (key = getch()) != ESC) {
        switch (key) {
        case KEY_UP:
            clear();                    // clear screen
            printw("KEY_UP\n");
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_DIRECTION;
            request.dir     = WIFI_VEHICLE_MOVE_FORWARD;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case KEY_DOWN:
            clear();                    // clear screen
            printw("KEY_DOWN\n");
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_DIRECTION;
            request.dir     = WIFI_VEHICLE_MOVE_BACKWARD;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case KEY_LEFT:
            clear();                    // clear screen
            printw("KEY_LEFT\n");
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_DIRECTION;
            request.dir     = WIFI_VEHICLE_MOVE_LEFT;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case KEY_RIGHT:
            clear();                    // clear screen
            printw("KEY_RIGHT\n");
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_DIRECTION;
            request.dir     = WIFI_VEHICLE_MOVE_RIGHT;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case ENTER: {
            clear();                    // clear screen
            printw("KEY_ENTER\n");

            if (!isOn) {
                memset(&request, 0, sizeof(struct reqMsg));
                request.type    = REQ_CMD_TYPE_BUZZER;
                request.dir     = WIFI_VEHICLE_BUZZER_ON;
                iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
                if (iSendLen <= 0) {
                    close(iSocketClient);
                    return -1;
                }
                
                isOn = 1;
                
            }else {
                memset(&request, 0, sizeof(struct reqMsg));
                request.type    = REQ_CMD_TYPE_BUZZER;
                request.dir     = WIFI_VEHICLE_BUZZER_OFF;
                iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
                if (iSendLen <= 0) {
                    close(iSocketClient);
                    return -1;
                }
                
                isOn = 0;
            }

            break;
        }

        case ACCELERATE:
            clear();                    // clear screen
            printw("KEY_[\n");

            /* Setup the duty cycle */
            dutyCycle++;
            if (dutyCycle > 100)
                dutyCycle = 100;
            
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_SPEED;
            request.speed   = dutyCycle;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case DECELERATE:
            clear();                    // clear screen
            printw("KEY_]\n");

            /* Setup the duty cycle */
            dutyCycle--;
            if (dutyCycle > 100)
                dutyCycle = 100;
            
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_SPEED;
            request.speed   = dutyCycle;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case SPACE:
            clear();                    // clear screen
            printw("KEY_SPACE\n");
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_DIRECTION;
            request.dir     = WIFI_VEHICLE_STOP;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            
            break;

        case TEMPERATURE:
            clear();                    // clear screen
            printw("KEY_t\n");
            memset(&request, 0, sizeof(struct reqMsg));
            request.type    = REQ_CMD_TYPE_TEMPERATURE;
			iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
			if (iSendLen <= 0) {
				close(iSocketClient);
				return -1;
			}            

            /* Recv the temperature from the server */
            
            
            break;
            
        default:
            clear();                    // clear screen
            printw("key = %d\n", key);
            break;
        }
    }
    
    /* End ncurses */
    endwin();
    
	return 0;
}



