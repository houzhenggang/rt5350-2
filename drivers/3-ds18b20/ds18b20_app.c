
#include <stdio.h>
#include <curses.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define TEMP_CONVERT_RATIO_FOR_12BIT            0.0625

#define DS18B20_DEV                             "/dev/rt5350_ds18b20"

int main(int argc, char *argv[])
{
    int iFd;
    float temperature;
    unsigned int tmp = 0;
    
    /* Step 1: Open device */
    iFd = open(DS18B20_DEV, O_RDWR);
    if (iFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", DS18B20_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", DS18B20_DEV);
    }
    
    /* Step 2: Read the temperature */
    read(iFd, &tmp, sizeof(tmp));

    temperature = tmp * TEMP_CONVERT_RATIO_FOR_12BIT;
    printf("tmp: %d\n", tmp);
    printf("the current temperature is: %f\n", temperature);
    
    return 0;
}

