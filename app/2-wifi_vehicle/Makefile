
all: wifi_vehicle_client.c wifi_vehicle_server.c
	gcc -Wall wifi_vehicle_client.c -o wifi_vehicle_client -lncurses
	gcc -Wall wifi_vehicle_server.c -o wifi_vehicle_server

client:
	gcc -Wall wifi_vehicle_client.c -o wifi_vehicle_client -lncurses

server:
	gcc -Wall wifi_vehicle_server.c -o wifi_vehicle_server

clean:
	rm -f wifi_vehicle_server wifi_vehicle_client

