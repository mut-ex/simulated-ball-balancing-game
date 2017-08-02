/*
CSE 438 - Embedded Systems Programming
Project: 5
Description: This is the user space program for the ball balancing game.

Author: Adil Faqah
Date: 28th April 2016
*/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/input.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>

#define KEYFILE "/dev/input/event2"
#define ASYNCDEV "/dev/key_async"
#define RAD_TO_DEG 57.295779513082320876798154814105
#define MIN_BOUND 2

unsigned int timer = 0;

int asyncfd;
int display_fd = 0;

char writeBuf[2];

struct spi_ioc_transfer xfer;

int ball_x, ball_y;

int game_running = 1;

//2D array to store bitmap data for the display.
unsigned char bmap_data[22][8]=
{
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//0 -  All Blank
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},//1 -  All Filled
    {0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},//2 -  Center
    {0xE7,0x42,0x24,0x18,0x18,0x24,0x42,0xE7},//3 -  X
    {0x7E,0xFF,0xC3,0xC3,0xC3,0xC3,0xFF,0x7E},//4 -  0
    {0x1C,0x3C,0x7C,0x1C,0x1C,0x1C,0x1C,0x1C},//5 -  1
    {0x3E,0xFF,0xE7,0x0E,0x38,0xE0,0xFF,0xFF},//6 -  2
    {0xFE,0xFF,0x07,0xFE,0xFE,0x07,0xFF,0xFE},//7 -  3
    {0xEE,0xEE,0xEE,0xFF,0xFF,0x0E,0x0E,0x0E},//8 -  4
    {0xFF,0xFF,0xE0,0xFE,0xFF,0x07,0xFF,0x7E},//9 -  5
    {0x7F,0xFF,0xE0,0xFE,0xFF,0xE7,0xFF,0x7E},//10 - 6
    {0xFF,0xFF,0x07,0x0E,0x1C,0x38,0x38,0x38},//11 - 7
    {0x7E,0xFF,0xE7,0x7E,0x7E,0xE7,0xFF,0x7E},//12 - 8
    {0x7E,0xFF,0xE7,0xFF,0x7F,0x07,0xFF,0x7E},//13 - 9

    //Bitmaps for the level won animation.
    {0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x24,0x24,0x18,0x00,0x00},
    {0x00,0x18,0x24,0x42,0x42,0x24,0x18,0x00},
    {0x3c,0x42,0x81,0x81,0x81,0x81,0x42,0x3c},
    {0x3c,0x42,0xa5,0x81,0xa5,0x99,0x42,0x3c},
    {0x3c,0x42,0x81,0x81,0x81,0x81,0x42,0x3c},
    {0x3c,0x42,0xa5,0x81,0xa5,0x99,0x42,0x3c},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

//Time thread to keep track of time.
//Ticks every 1 second.
void* timerThread(void *params)
{
    int threadNum = *(int*)params;

    //Run only while the game is running.
    while(game_running)
    {
        if (timer > 0)
        {
            timer--;
            printf("%d: Tick! %d\n", threadNum, timer);
        }
        usleep(1000000);
    }
    return 0;
}

//Function to get the current RDTSC value.
unsigned long long int get_ticks(void)
{
    unsigned high_edx, low_eax;
    asm volatile ("rdtsc" : "=a"(low_eax), "=d"(high_edx));
    return ((unsigned long long)low_eax)|(((unsigned long long)high_edx)<<32);
}

//Function to return a random number that falls within the given range
unsigned int get_rand(unsigned int minimum, unsigned int maximum)
{
    int rndm;
    srand(get_ticks());
    const unsigned int range = 1 + maximum - minimum;
    const unsigned int chunks = RAND_MAX / range;
    const unsigned int limit = chunks * range;
    do
    {
        rndm = rand();
    }
    while (rndm >= limit);

    return (minimum + (rndm/chunks));
}

//Function to export all the required GPIOs, set their direction and initialize them.
void setup_GPIOs(void)
{
    int fd = 0;

    // CS Pin Mux Sel
    //Export
    fd = open("/sys/class/gpio/export", O_WRONLY);
    write(fd, "42", 2);
    close(fd);
    // Set GPIO Direction
    fd = open("/sys/class/gpio/gpio42/direction", O_WRONLY);
    write(fd, "out", 3);
    close(fd);
    // Set Value
    fd = open("/sys/class/gpio/gpio42/value", O_WRONLY);
    write(fd, "0", 1);
    close(fd);

    // MOSI Pin Mux Sel
    //Export
    fd = open("/sys/class/gpio/export", O_WRONLY);
    write(fd, "43", 2);
    close(fd);
    // Set GPIO Direction
    fd = open("/sys/class/gpio/gpio43/direction", O_WRONLY);
    write(fd, "out", 3);
    close(fd);
    // Set Value
    fd = open("/sys/class/gpio/gpio43/value", O_WRONLY);
    write(fd, "0", 1);
    close(fd);

    // SCK Pin Mux Sel
    //Export
    fd = open("/sys/class/gpio/export", O_WRONLY);
    write(fd, "55", 2);
    close(fd);
    // Set GPIO Direction
    fd = open("/sys/class/gpio/gpio55/direction", O_WRONLY);
    write(fd, "out", 3);
    close(fd);
    // Set Value
    fd = open("/sys/class/gpio/gpio55/value", O_WRONLY);
    write(fd, "0", 1);
    close(fd);
}

//Function to unexport all the GPIOs that were exported.
void GPIO_cleanup()
{
    unsigned display_fd = 0;

    // CS Pin Mux Sel
    //Unexport
    display_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    write(display_fd, "42", 2);
    close(display_fd);

    // MOSI Pin Mux Sel
    //Unexport
    display_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    write(display_fd, "43", 2);
    close(display_fd);

    // SCK Pin Mux Sel
    //Unexport
    display_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    write(display_fd, "55", 2);
    close(display_fd);
}

//Function to blank the LED display.
void display_clear(void)
{
    unsigned int i;

    for(i = 1; i < 9; i++)
    {
        // Setup a Write Transfer
        writeBuf[0] = i;//Select row
        writeBuf[1] = 0x00;//Write row data
        // Send Message
        if(ioctl(display_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
            perror("SPI Message Send Failed");
    }
}

//Write a row of data to the LED display.
void display_write(char a, char b)
{
    writeBuf[0] = a;//Select row
    writeBuf[1] = b;//Write row data
    // Send Message
    if(ioctl(display_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        perror("SPI Message Send Failed");
}

//Function to light up a specific pixel on the LED display (not persistent).
void display_set_pixel(unsigned int x, unsigned int y)
{
    display_clear();
    if ((x >= 1 && x <= 8) && (y >= 1 && y <= 8))
        display_write(9-y, (0b10000000)>>(x-1));
}

//Function to write an entire 8x8 bitmap to the LED display.
void display_put(unsigned int index)
{
    unsigned int i;

    for(i = 1; i < 9; i++)
    {
        display_write(i, bmap_data[index][i-1]);
    }
}

//Function to set up the SPU and LED display parameters.
int display_setup()
{
    unsigned char mytemp = 0;

    mytemp = 0;
    mytemp |= SPI_MODE_0;

    if(ioctl(display_fd, SPI_IOC_WR_MODE, &mytemp) < 0)
    {
        perror("SPI Set Mode Failed");
        close(display_fd);
        return -1;
    }

    mytemp = 0;
    if(ioctl(display_fd, SPI_IOC_WR_LSB_FIRST, &mytemp) < 0)
    {
        perror("SPI Set WR_LSB_FIRST Failed");
        close(display_fd);
        return -1;
    }

    mytemp = 8;
    if(ioctl(display_fd, SPI_IOC_WR_BITS_PER_WORD, &mytemp) < 0)
    {
        perror("SPI Set WR_BITS_PER_WORD Failed");
        close(display_fd);
        return -1;
    }

    mytemp = 0;
    if(ioctl(display_fd, SPI_IOC_WR_MAX_SPEED_HZ, &mytemp) < 0)
    {
        perror("SPI Set WR_MAX_SPEED_HZ Failed");
        close(display_fd);
        return -1;
    }

    // Setup Transaction
    xfer.tx_buf = (unsigned long) writeBuf;
    xfer.rx_buf = 0;
    xfer.len = 2; // Bytes to send
    xfer.delay_usecs = 0;
    xfer.bits_per_word = 8;
    xfer.speed_hz = 25000000;

    //Set the decoding mode as BCD.
    display_write(0x09, 0x00);

    //Set the brightness.
    display_write(0x0A, 0x03);

    //Set the scan limit to 8 LEDs.
    display_write(0x0B, 0x07);

    //Set to normal mode.
    display_write(0x0C, 0x01);

    //Set to non-testing mode
    display_write(0x0F, 0x00);

    //Blank the display.
    display_clear();

    return 0;
}

//Function to place the 'ball' at a random location.
void place_ball()
{
    //Generate a random x value.
    ball_x = get_rand(2, 7);

    //Generate a random value of y making sure ball_x and ball_y do not lie within the center 4 LEDs.
    do
    {
        ball_y = get_rand(2, 7);
    }
    while((ball_x == 4 && ball_y == 4) || (ball_x == 5 && ball_y == 4) || (ball_x == 4 && ball_y == 5) || (ball_x == 5 && ball_y == 5));

    //'Draw' the ball on the LED display.
    display_set_pixel(ball_x, ball_y);
}

//Function to flash the center 4 LEDs on the display 4 times.
void game_start()
{
    int i;

    for(i = 0; i < 5; i++)
    {
        display_put(2);
        usleep(250000);

        display_clear();
        usleep(250000);
    }
}

//Function to display the level won animation on the LED display.
void level_won()
{
    int i;

    for(i = 14; i <= 21; i++)
    {
        display_put(i);
        usleep(250000);

        //display_clear();
        //usleep(250000);
    }
}

//Function to display the level lost animation on the LED display.
void level_lost()
{
    int i;

    for(i = 0; i < 5; i++)
    {
        display_put(3);
        usleep(250000);

        display_clear();
        usleep(250000);
    }
}

//Function to flash the bitmap for the corresponding level on the LED display.
void show_level(unsigned int j)
{
    int i = 0;

    for(i = 0; i < 2; i++)
    {
        display_put(4+j);
        usleep(250000);

        display_clear();
        usleep(250000);
    }
}

//Function to 'clean up' everything if the user terminates the program.
void siginthandler(int signum)
{
    printf("\nUser terminated program.\n");
    display_clear();
    GPIO_cleanup();
    close(display_fd);
    exit(1);
}

int main(int argc, char* argv[])
{
    pthread_t timer_thread;

    int i = 0, temp = 0;

    int fd_imu = 0;

    struct input_event ie;

    double  AcX = 0, AcY = 0, AcZ = 0, GyX = 0, GyY = 0; //GyZ = 0;
    double gyroXrate = 0, gyroYrate = 0;
    double roll = 0, pitch = 0;
    double compAngleX = 0, compAngleY = 0;
    double valX = 0, valY = 0;
    int old_x, old_y;
    double max = 0, min = 0, t1 = 0, t2 = 0, t3 = 0;

    int current_level = 0, reached_goal = 0;
    int threadNum1 = 1;

    int typex[2], codex[2], valuex[2];
    typex[0] = 1;
    typex[1] = 2;
    codex[0] = 1;
    codex[1] = 2;
    valuex[0] = 1;
    valuex[1] = 2;

    signal(SIGINT, siginthandler);

    //Start the timer thread (used to confirm if the ball has been inside the centre 4 LEDs for at least 3 seconds).
    pthread_create(&timer_thread, NULL, timerThread, (void*)&threadNum1);

    //Open the keyfile to read the joystick events from.
    if((fd_imu = open(KEYFILE, O_RDONLY)) == -1)
    {
        perror("opening device");
        exit(EXIT_FAILURE);
    }

    //Export all the GPIOs that will be needed.
    setup_GPIOs();

    //Open the SPI device to communicate with the LED display.
    display_fd = open("/dev/spidev1.0", O_RDWR);

    //Set up the LED display.
    display_setup();

    //Calculate the initial max value for the bins.
    max = MIN_BOUND + 10*MIN_BOUND;

    while(1)
    {
        //Flash the current level number 2 times.
        show_level(current_level);
        printf("\nPlaying level: %d\n", current_level);

        //Flash the center 4 leds 5 times.
        game_start();

        //Place the 'ball' in a random location. Excluding the 'outer' boundary wall and the innermost 4 leds.
        place_ball();

        //Reset the value of angles in both the axis at the start of each level.
        compAngleX = 0;
        compAngleY = 0;

        //Calculate the new 'bin' thresholds based on the difficulty of the level.
        min = 0;
        t1 = max/4;
        t2 = t1 + max/4;
        t3 = t2 + max/4;

        while(1)
        {
            //Read the next joystick event.
            while(read(fd_imu, &ie, sizeof(struct input_event)))
            {
                //If it is a joystick event...
                if(ie.type == EV_ABS)
                {
                    //Update the corresponding sensor value based on the code.
                    switch(ie.code)
                    {
                    case 0:
                        GyX = ie.value;
                        break;
                    case 1:
                        GyY = ie.value;
                        break;
                    case 2:
                        //GyZ not used so no need to update it.
                        //GyZ = ie.value;
                        break;

                    case 3:
                        AcX = ie.value;
                        break;
                    case 4:
                        AcY = ie.value;
                        break;
                    case 5:
                        AcZ = ie.value;
                        break;
                    }
                }

                //Store the type, code and value for the last 2 events.
                typex[i] = ie.type;
                codex[i] = ie.code;
                valuex[i] = ie.value;

                temp = i;
                i = (i == 1) ? 0 : 1;

                //If the type, code and value for this event are all 0. That means all the sensor readings from the 'current' batch have been read.
                if ((typex[temp] == 0) &&
                    (codex[temp] == 0) &&
                   (valuex[temp] == 0))
                {
                    break;
                }
            }

            //printf("AcX: %d, AcY: %d, AcZ: %d, GyX: %d, GyY: %d, GyZ: %d\n", AcX, AcY, AcZ, GyX, GyY, GyZ);

            //If type, code and value is 0 for two joystick events in a row that means the kernel module has stopped/been removed.
            if ((typex[0] == typex[1]) && (codex[0] == codex[1]) && (valuex[0] == valuex[1]))
            {
                game_running = 0;
                break;
            }
            //Compute the gyroscope rate by dividing the X and Y axis readings by the LSB sensitivity for the gyroscope.
            gyroXrate = GyX/131;;
            gyroYrate = GyY/131;

            //Compute the roll and pitch from the acceleormeter X and Y axis readings.
            roll  = atan(AcY / sqrt(AcX*AcX + AcZ*AcZ)) * RAD_TO_DEG;
            pitch = atan2(-AcX, AcZ) * RAD_TO_DEG;

            //gyroXangle = gyroXangle + (gyroXrate*0.02);
            compAngleX = 0.93 * (compAngleX + gyroXrate * 0.000020) + 0.07 * roll; // Calculate the angle using a Complimentary filter
            compAngleY = 0.93 * (compAngleY + gyroYrate * 0.000020) + 0.07 * pitch;

            //printf("compAngleX: %lf compAngleY: %lf Time: %lf\n", compAngleX, compAngleY, dt);

            valX = compAngleX; //- OLDcompAngleX;
            valY = compAngleY; //- OLDcompAngleY;
            
			//Calculate the x and y co-ordinates of the ball on the display based on which bin the angle falls in.
            if (valX > 0)
            {
                if (valX > min && valX < t1)
                    ball_x = 4;
                if (valX >= t1 && valX < t2)
                    ball_x = 3;
                if (valX >= t2 && valX < t3)
                    ball_x = 2;
                if (valX >= t3 && valX < max)
                    ball_x = 1;
                if (valX >= max)
                    ball_x = 0;
            }
            else if (valX < 0)
            {
                if (valX <= min && valX > -t1)
                    ball_x = 5;
                if (valX <= -t1 && valX > -t2)
                    ball_x = 6;
                if (valX <= -t2 && valX > -t3)
                    ball_x = 7;
                if (valX <= -t3 && valX > -max)
                    ball_x = 8;
                if (valX <= -max)
                    ball_x = 9;
            }

            if (valY > 0)
            {
                if (valY > min && valY < t1)
                    ball_y = 4;
                if (valY >= t1 && valY < t2)
                    ball_y = 3;
                if (valY >= t2 && valY < t3)
                    ball_y = 2;
                if (valY >= t3 && valY < max)
                    ball_y = 1;
                if (valY >= max)
                    ball_y = 0;
            }
            else if (valY < 0)
            {
                if (valY <= min && valY > -t1)
                    ball_y = 5;
                if (valY <= -t1 && valY > -t2)
                    ball_y = 6;
                if (valY <= -t2 && valY > -t3)
                    ball_y = 7;
                if (valY <= -t3 && valY > -max)
                    ball_y = 8;
                if (valY <= -max)
                    ball_y = 9;
            }

			//Only update the position of the ball if the co-ordinates have changed.
            if (ball_x != old_x || ball_y != old_y)
                display_set_pixel(ball_x, ball_y);
				
            old_x = ball_x;
            old_y = ball_y;
           
            //printf("compAngleX: %lf compAngleY: %lf Time: %lf\n", compAngleX, compAngleY, dt);

			//If the ball is within the centre 4 leds, start the timer and set the goal reached flag.
            if ((ball_x == 4 && ball_y == 4) ||
                    (ball_x == 5 && ball_y == 4) ||
                    (ball_x == 4 && ball_y == 5) ||
                    (ball_x == 5 && ball_y == 5))
            {
                if (reached_goal != 1)
                {
                    timer = 3;
                    reached_goal = 1;
                }
            }
            else
            {
                timer = 0;
                reached_goal = 0;
            }

            if (timer == 0 && reached_goal == 1)
            {
                printf("\nYou won! :)");
                level_won();
                current_level += 1;
                break;
            }

            if ((ball_x == 0 || ball_x == 9) || (ball_y == 0 || ball_y == 9))
            {
                printf("\nYou lost... :(");
                level_lost();
                max = 2*MIN_BOUND + 10*MIN_BOUND;
                current_level = 0;
                break;
            }
        }

        //End the game if the kernel module has stopped/been removed.
        if (game_running == 0)
            break;

        //End the game after the last level.
        if (current_level == 10)
            break;
       
        //If the last level was won...
        if (reached_goal == 1)
        {
            //Reset the reached_goal flag.
            reached_goal = 0;
            //Decrement the max value for next level.
            max -= MIN_BOUND;

        }
    }

    //Blank the LED display.
    display_clear();

    //Free all the GPIOs that were exported.
    GPIO_cleanup();

    //Close the key file.
    close(fd_imu);

    //Close the file device.
    close(display_fd);

    return 0;
}