

#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

/* socket
 * bind
 * listen
 * accept
 * send/recv
 */
#define SERVER_PORT                         8888
#define BACKLOG                             10

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

int main(int argc, char *argv[])
{
	int iSocketServer;
	int iSocketClient;
	struct sockaddr_in tSocketServerAddr;
	struct sockaddr_in tSocketClientAddr;
	int iRet;
	//int iAddrLen;
	socklen_t iAddrLen;
	int iRecvLen;
	int iClientNum = -1;
    struct reqMsg request;

	signal(SIGCHLD, SIG_IGN);
	
	iSocketServer = socket(AF_INET, SOCK_STREAM, 0);
	if (-1 == iSocketServer) {
		printf("socket error!\n");
		return -1;
	}

	tSocketServerAddr.sin_family      = AF_INET;
	tSocketServerAddr.sin_port        = htons(SERVER_PORT);  /* host to net, short */
 	tSocketServerAddr.sin_addr.s_addr = INADDR_ANY;
	memset(tSocketServerAddr.sin_zero, 0, 8);
	
	iRet = bind(iSocketServer, (const struct sockaddr *)&tSocketServerAddr, sizeof(struct sockaddr));
	if (-1 == iRet) {
		printf("bind error!\n");
		return -1;
	}

	iRet = listen(iSocketServer, BACKLOG);
	if (-1 == iRet) {
		printf("listen error!\n");
		return -1;
	}

	while (1) {
		iAddrLen = sizeof(struct sockaddr);
		iSocketClient = accept(iSocketServer, (struct sockaddr *)&tSocketClientAddr, &iAddrLen);
		if (-1 != iSocketClient) {
			iClientNum++;
			printf("Get connect from client %d : %s\n",  iClientNum, inet_ntoa(tSocketClientAddr.sin_addr));
			if (!fork()) {

                /* 子进程的源码 */
				while (1) {

                    /* 接收客户端发来的数据并显示出来 */
					iRecvLen = recv(iSocketClient, &request, sizeof(struct reqMsg), 0);
					if (iRecvLen <= 0) {
						close(iSocketClient);
						return -1;
					}else {
					    
					    switch (request.type) {
                        case REQ_CMD_TYPE_DIRECTION:
                            printf("Get Msg From Client %d: REQ_CMD_TYPE_DIRECTION = %d\n", iClientNum, request.dir);
                            break;

                        case REQ_CMD_TYPE_BUZZER:
                            printf("Get Msg From Client %d: REQ_CMD_TYPE_BUZZER = %d\n", iClientNum, request.dir);
                            break;

                        case REQ_CMD_TYPE_SPEED:
                            printf("Get Msg From Client %d: REQ_CMD_TYPE_SPEED = %d\n", iClientNum, request.speed);
                            break;

                        case REQ_CMD_TYPE_TEMPERATURE:
                            printf("Get Msg From Client %d: REQ_CMD_TYPE_TEMPERATURE\n", iClientNum);

                            /* Acquire the temperature from ds18b20, and send the temperature to the client */
                            
                            break;

                        default:
                            printf("No such request type!\n");
                            break;
					    }                        
					}
				}				
			}
		}
	}
	
	close(iSocketServer);
	return 0;
}



