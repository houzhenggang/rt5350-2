
#include <stdio.h>
#include <curses.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define ESC                                 27
#define ENTER                               13

#define MOTOR_DEV                           "/dev/rt5350_motor"

#define WIFI_VEHICLE_MOVE_FORWARD           0
#define WIFI_VEHICLE_MOVE_BACKWARD          1
#define WIFI_VEHICLE_MOVE_LEFT              2
#define WIFI_VEHICLE_MOVE_RIGHT             3
#define WIFI_VEHICLE_STOP                   4
#define WIFI_VEHICLE_BUZZER_ON              5
#define WIFI_VEHICLE_BUZZER_OFF             6

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
    int key;
    int iFd;
    char val;
    int isOn = 0;

    libncurses_init();

    /* Step 1: Open device */
    iFd = open(MOTOR_DEV, O_RDWR);
    if (iFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", MOTOR_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", MOTOR_DEV);
    }

    /* Step 2: Enable EN_A & EN_B */
    val = 1;
    write(iFd, &val, 1);

    /* Step 3: Control the wifi vehicle by using ioctl */
    
    
    while ( (key = getch()) != ESC) {
        switch (key) {
        case KEY_UP:
            clear();                    // clear screen
            printw("KEY_UP\n");
            ioctl(iFd, WIFI_VEHICLE_MOVE_FORWARD);
            break;

        case KEY_DOWN:
            clear();                    // clear screen
            printw("KEY_DOWN\n");
            ioctl(iFd, WIFI_VEHICLE_MOVE_BACKWARD);
            break;

        case KEY_LEFT:
            clear();                    // clear screen
            printw("KEY_LEFT\n");
            ioctl(iFd, WIFI_VEHICLE_MOVE_LEFT);
            break;

        case KEY_RIGHT:
            clear();                    // clear screen
            printw("KEY_RIGHT\n");
            ioctl(iFd, WIFI_VEHICLE_MOVE_RIGHT);
            break;

        case ENTER: {
            clear();                    // clear screen
            printw("KEY_ENTER\n");

            if (!isOn) {
                ioctl(iFd, WIFI_VEHICLE_BUZZER_ON);
                isOn = 1;
            }else {
                ioctl(iFd, WIFI_VEHICLE_BUZZER_OFF);
                isOn = 0;
            }

            break;
        }
        
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

