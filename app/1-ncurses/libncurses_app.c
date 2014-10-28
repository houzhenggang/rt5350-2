

#include <stdio.h>
#include <curses.h>

#define ESC             27
#define ENTER           13

int main(int argc, char *argv[])
{
    int key;
    
    /* Initialize ncurses */
    initscr();

    /* Initialize keypad */
    nonl();                         /* Set the return char without using newline char */
    intrflush(stdscr, FALSE);       /* Whether to deal with interrupt signal */
    keypad(stdscr, TRUE);           /* Enable keypad functionality so that the system will receive the up,down,left,right keys */
    
    while ( (key = getch()) != ESC) {
        switch (key) {
        case KEY_UP:
            clear();                    // clear screen
            printw("KEY_UP\n");
            break;

        case KEY_DOWN:
            clear();                    // clear screen
            printw("KEY_DOWN\n");
            break;

        case KEY_LEFT:
            clear();                    // clear screen
            printw("KEY_LEFT\n");
            break;

        case KEY_RIGHT:
            clear();                    // clear screen
            printw("KEY_RIGHT\n");
            break;

        case ENTER:
            clear();                    // clear screen
            printw("KEY_ENTER\n");
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

