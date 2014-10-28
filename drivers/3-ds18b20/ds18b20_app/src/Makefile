CC = gcc
CFLAGS = -Wall
OBJS = ds18b20_app.o

all: ds18b20_app

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $< $(LDFLAGS)

ds18b20_app: $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

clean:
	rm -f rbcfg *.o
