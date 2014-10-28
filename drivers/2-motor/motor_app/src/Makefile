CC = gcc
CFLAGS = -Wall
OBJS = motor_app.o

all: motor_app

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $< $(LDFLAGS)

motor_app: $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

clean:
	rm -f rbcfg *.o
