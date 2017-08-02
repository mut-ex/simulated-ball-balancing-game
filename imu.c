/*
CSE 438 - Embedded Systems Programming
Project: 5
Description: This is the kernel space program for the ball balancing game.

Author: Adil Faqah
Date: 28th April 2016
*/

#include <linux/module.h>  // Module Defines and Macros (THIS_MODULE)
#include <linux/kernel.h>  // 
#include <linux/fs.h>      // Inode and File types
#include <linux/cdev.h>    // Character Device Types and functions.
#include <linux/types.h>
#include <linux/slab.h>    // Kmalloc/Kfree
#include <asm/uaccess.h>   // Copy to/from user space
#include <linux/string.h>
#include <linux/device.h>  // Device Creation / Destruction functions
#include <linux/i2c.h>     // i2c Kernel Interfaces
#include <linux/i2c-dev.h>

#include <linux/gpio.h>
#include <asm/gpio.h>

#include <linux/init.h>
#include <linux/moduleparam.h> // Passing parameters to modules through insmod

#include <linux/kthread.h>

#include <linux/delay.h>
#include <linux/interrupt.h>

#include <linux/input.h>

#define DEVICE_NAME "imu"  // device name to be created and registered
#define DEVICE_ADDR 0x68
#define I2CMUX 29 // Clear for I2C function

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGREEN  "\x1B[32m"
#define KYELLOW  "\x1B[33m"
#define KBLUE  "\x1B[34m"
#define RESET "\033[0m"

#define RA_XG_OFFS_TC 0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_YG_OFFS_TC 0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_ZG_OFFS_TC 0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_X_FINE_GAIN 0x03 //[7:0] X_FINE_GAIN
#define RA_Y_FINE_GAIN 0x04 //[7:0] Y_FINE_GAIN
#define RA_Z_FINE_GAIN 0x05 //[7:0] Z_FINE_GAIN
#define RA_XA_OFFS_H 0x06 //[15:0] XA_OFFS
#define RA_XA_OFFS_L_TC 0x07
#define RA_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define RA_YA_OFFS_L_TC 0x09
#define RA_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS
#define RA_ZA_OFFS_L_TC 0x0B
#define RA_XG_OFFS_USRH 0x13 //[15:0] XG_OFFS_USR
#define RA_XG_OFFS_USRL 0x14
#define RA_YG_OFFS_USRH 0x15 //[15:0] YG_OFFS_USR
#define RA_YG_OFFS_USRL 0x16
#define RA_ZG_OFFS_USRH 0x17 //[15:0] ZG_OFFS_USR
#define RA_ZG_OFFS_USRL 0x18
#define RA_SMPLRT_DIV 0x19
#define RA_CONFIG 0x1A
#define RA_GYRO_CONFIG 0x1B
#define RA_ACCEL_CONFIG 0x1C
#define RA_FF_THR 0x1D
#define RA_FF_DUR 0x1E
#define RA_MOT_THR 0x1F
#define RA_MOT_DUR 0x20
#define RA_ZRMOT_THR 0x21
#define RA_ZRMOT_DUR 0x22
#define RA_FIFO_EN 0x23
#define RA_I2C_MST_CTRL 0x24
#define RA_I2C_SLV0_ADDR 0x25
#define RA_I2C_SLV0_REG 0x26
#define RA_I2C_SLV0_CTRL 0x27
#define RA_I2C_SLV1_ADDR 0x28
#define RA_I2C_SLV1_REG 0x29
#define RA_I2C_SLV1_CTRL 0x2A
#define RA_I2C_SLV2_ADDR 0x2B
#define RA_I2C_SLV2_REG 0x2C
#define RA_I2C_SLV2_CTRL 0x2D
#define RA_I2C_SLV3_ADDR 0x2E
#define RA_I2C_SLV3_REG 0x2F
#define RA_I2C_SLV3_CTRL 0x30
#define RA_I2C_SLV4_ADDR 0x31
#define RA_I2C_SLV4_REG 0x32
#define RA_I2C_SLV4_DO 0x33
#define RA_I2C_SLV4_CTRL 0x34
#define RA_I2C_SLV4_DI 0x35
#define RA_I2C_MST_STATUS 0x36
#define RA_INT_PIN_CFG 0x37
#define RA_INT_ENABLE 0x38
#define RA_DMP_INT_STATUS 0x39
#define RA_INT_STATUS 0x3A
#define RA_ACCEL_XOUT_H 0x3B
#define RA_ACCEL_XOUT_L 0x3C
#define RA_ACCEL_YOUT_H 0x3D
#define RA_ACCEL_YOUT_L 0x3E
#define RA_ACCEL_ZOUT_H 0x3F
#define RA_ACCEL_ZOUT_L 0x40
#define RA_TEMP_OUT_H 0x41
#define RA_TEMP_OUT_L 0x42
#define RA_GYRO_XOUT_H 0x43
#define RA_GYRO_XOUT_L 0x44
#define RA_GYRO_YOUT_H 0x45
#define RA_GYRO_YOUT_L 0x46
#define RA_GYRO_ZOUT_H 0x47
#define RA_GYRO_ZOUT_L 0x48
#define RA_EXT_SENS_DATA_00 0x49
#define RA_EXT_SENS_DATA_01 0x4A
#define RA_EXT_SENS_DATA_02 0x4B
#define RA_EXT_SENS_DATA_03 0x4C
#define RA_EXT_SENS_DATA_04 0x4D
#define RA_EXT_SENS_DATA_05 0x4E
#define RA_EXT_SENS_DATA_06 0x4F
#define RA_EXT_SENS_DATA_07 0x50
#define RA_EXT_SENS_DATA_08 0x51
#define RA_EXT_SENS_DATA_09 0x52
#define RA_EXT_SENS_DATA_10 0x53
#define RA_EXT_SENS_DATA_11 0x54
#define RA_EXT_SENS_DATA_12 0x55
#define RA_EXT_SENS_DATA_13 0x56
#define RA_EXT_SENS_DATA_14 0x57
#define RA_EXT_SENS_DATA_15 0x58
#define RA_EXT_SENS_DATA_16 0x59
#define RA_EXT_SENS_DATA_17 0x5A
#define RA_EXT_SENS_DATA_18 0x5B
#define RA_EXT_SENS_DATA_19 0x5C
#define RA_EXT_SENS_DATA_20 0x5D
#define RA_EXT_SENS_DATA_21 0x5E
#define RA_EXT_SENS_DATA_22 0x5F
#define RA_EXT_SENS_DATA_23 0x60
#define RA_MOT_DETECT_STATUS 0x61
#define RA_I2C_SLV0_DO 0x63
#define RA_I2C_SLV1_DO 0x64
#define RA_I2C_SLV2_DO 0x65
#define RA_I2C_SLV3_DO 0x66
#define RA_I2C_MST_DELAY_CTRL 0x67
#define RA_SIGNAL_PATH_RESET 0x68
#define RA_MOT_DETECT_CTRL 0x69
#define RA_USER_CTRL 0x6A
#define RA_PWR_MGMT_1 0x6B
#define RA_PWR_MGMT_2 0x6C
#define RA_BANK_SEL 0x6D
#define RA_MEM_START_ADDR 0x6E
#define RA_MEM_R_W 0x6F
#define RA_DMP_CFG_1 0x70
#define RA_DMP_CFG_2 0x71
#define RA_FIFO_COUNTH 0x72
#define RA_FIFO_COUNTL 0x73
#define RA_FIFO_R_W 0x74
#define RA_WHO_AM_I 0x75

#define USS_ECHO 14
#define GPIO_2_PU_MUXSEL 0
#define GPIO_2_MUXSEL 31

static struct input_dev *input_imu_dev;

static struct task_struct * WorkThread = NULL; //WorkThread thread struct.
//unsigned int echo_irq = 0;
unsigned int data_ready = 0;

/* per device structure */
struct imu_dev {
    struct cdev cdev;               /* The cdev structure */
    // Local variables
    struct i2c_client *client;
    struct i2c_adapter *adapter;

} *imu_devp;

static dev_t imu_dev_number;      /* Allotted device number */
struct class *imu_dev_class;          /* Tie with the device model */
static struct device *imu_dev_device;

//Timing variables.
unsigned long long int start = 0, end = 0;
unsigned long long int dt;

//Function to get the current RDTSC value.
unsigned long long int get_ticks(void)
{
    unsigned high_edx, low_eax;
    asm volatile ("rdtsc" : "=a"(low_eax), "=d"(high_edx));
    return ((unsigned long long)low_eax)|(((unsigned long long)high_edx)<<32);
}

/*
* Open driver
*/
int imu_driver_open(struct inode *inode, struct file *file)
{
    struct imu_dev *imu_devp;

    /* Get the per-device structure that contains this cdev */
    imu_devp = container_of(inode->i_cdev, struct imu_dev, cdev);

    /* Easy access to imu_devp from rest of the entry points */
    file->private_data = imu_devp;
    
    return 0;
}

/*
 * Release driver
 */
int imu_driver_release(struct inode *inode, struct file *file)
{
    //struct imu_dev *imu_devp = file->private_data;
    
    return 0;
}

/* File operations structure. Defined in linux/fs.h */
static struct file_operations tmp_fops = {
    .owner      = THIS_MODULE,           /* Owner */
    .open       = imu_driver_open,        /* Open method */
    .release    = imu_driver_release,     /* Release method */
};

//Function read the specified number of bytes from the specified address into the specified buffer.
int i2c_read(int address, char *read_buf, unsigned int length)
{
    char buf[1];
    int ret = 0;

    buf[0] = address;
    ret = i2c_master_send(imu_devp->client, buf, 1);
    if(ret < 0)
    {
        printk("Error: could not send addr ptr.\n");
        return -1;
    }
    ret = i2c_master_recv(imu_devp->client, read_buf, length);
    if(ret < 0)
    {
        printk("Error: could not read data.\n");
        return -1;
    }
    return 0;
}

//Function to read and return 2 bytes of data starting from the specified address.
int16_t i2c_read_word(char address)
{
    char buf[2];
    uint16_t  MSB, LSB;
    int16_t result;

    i2c_read(address, buf, 2);
    MSB = (uint8_t)buf[0];
    LSB = (uint8_t)buf[1];

    result = ((MSB) << 8) | LSB;

    return(result);
}

//Function to read and return 1 byte of data from the specified address.
int8_t i2c_read_byte(char address)
{
    char buf[1];
    int16_t result;

    i2c_read(address, buf, 1);

    result = (uint8_t)buf[0];

    return(result);
}

//Function to write 1 byte of data to the specified address.
int i2c_write(char address, char data)
{
    int ret = 0;
    char buf[2];

    buf[0] = address;
    buf[1] = data;
    ret = i2c_master_send(imu_devp->client, buf, 2);
    if(ret < 0)
    {
        printk("Error: could not send addr ptr.\n");
        return -1;
    }
    // i2c_read(address, buf, 1);
    // if (buf[0] != data)
    //     printk("\n Write failed! Address: %x Data Written: %x, Data Read: %x", address, data, buf[0]);
    // else
    //     printk("\n Write successful! Address: %x, Data Written: %x", address, buf[0]);

    return 0;
}

static int my_work(void *data)
{
    int16_t  AcX, AcY, AcZ, GyX, GyY, GyZ;//, static_angle = 0;
    int8_t my_address;

    char my_buff[14];

    my_address = i2c_read_byte(RA_WHO_AM_I);
    if (DEVICE_ADDR != my_address)
    //     printk(KGREEN"\n >> MPU-6050 found! Address: 0x%x"RESET, my_address);
    // else
    {
        printk(KRED"\n >> MPU-6050 not found."RESET);
        return -1;
    }

    //Set the PLL with z-axis gyroscope as reference for clock. 
    i2c_write(RA_PWR_MGMT_1, 0b00000011);

    //Set the configuration register to 1 i.e. Acceleormeter filter bandwidth to 260 Hz and Gyroscope filter bandwidth to 188 Hz.
    i2c_write(RA_CONFIG, 0b00000001);

    //Set the GYRO config register to the default values.
    i2c_write(RA_GYRO_CONFIG, 0x00);

    //Set the ACCEL config register to the default values.
    i2c_write(RA_ACCEL_CONFIG, 0x00);

    //Set the sample rate divide register to the default values.
    i2c_write(RA_SMPLRT_DIV, 0x00);

    while(!kthread_should_stop())
    {   
        //Read 14 registers at once.
        i2c_read(RA_ACCEL_XOUT_H, my_buff, 14);

        //Combine the lower and higher 8 bits to form a 16 bit value for each of the paramters.
        AcX = (((uint8_t)my_buff[0]) << 8) | ((uint8_t)my_buff[1]);
        AcY = (((uint8_t)my_buff[2]) << 8) | ((uint8_t)my_buff[3]);
        AcZ = (((uint8_t)my_buff[4]) << 8) | ((uint8_t)my_buff[5]);

        GyX = (((uint8_t)my_buff[8]) << 8) | ((uint8_t)my_buff[9]);
        GyY = (((uint8_t)my_buff[10]) << 8) | ((uint8_t)my_buff[11]);
        GyZ = (((uint8_t)my_buff[12]) << 8) | ((uint8_t)my_buff[13]);

        // AcX = i2c_read_word(RA_ACCEL_XOUT_H);
        // AcY = i2c_read_word(RA_ACCEL_YOUT_H);
        // AcZ = i2c_read_word(RA_ACCEL_ZOUT_H);

        // GyX = i2c_read_word(RA_GYRO_XOUT_H);
        // GyY = i2c_read_word(RA_GYRO_YOUT_H);
        // GyZ = i2c_read_word(RA_GYRO_ZOUT_H);

        //printk("\nAcX: %d, AcY: %d, AcZ: %d, GyX: %d, GyY: %d, GyZ: %d", AcX, AcY, AcZ, GyX, GyY, GyZ);

        //Post the values as joystick events.
        input_report_abs(input_imu_dev, ABS_X, GyX);
        input_report_abs(input_imu_dev, ABS_Y, GyY);
        input_report_abs(input_imu_dev, ABS_Z, GyZ);

        input_report_abs(input_imu_dev, ABS_RX, AcX);
        input_report_abs(input_imu_dev, ABS_RY, AcY);
        input_report_abs(input_imu_dev, ABS_RZ, AcZ);

        //Synchronise.
        input_sync(input_imu_dev);

        //Atomic delay for 1ms.
        udelay(1000);
    }
    return 0;
}

/*
 * Driver Initialization
 */
int __init imu_driver_init(void)
{
    int ret;
    int dummy = 0;
    
    /* Request dynamic allocation of a device major number */
    if (alloc_chrdev_region(&imu_dev_number, 0, 1, DEVICE_NAME) < 0) {
            printk(KERN_DEBUG "Can't register device\n"); return -1;
    }

    /* Populate sysfs entries */
    imu_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

    /* Allocate memory for the per-device structure */
    imu_devp = kmalloc(sizeof(struct imu_dev), GFP_KERNEL);
        
    if (!imu_devp) {
        printk("Bad Kmalloc\n"); return -ENOMEM;
    }

    /* Request I/O region */

    /* Connect the file operations with the cdev */
    cdev_init(&imu_devp->cdev, &tmp_fops);
    imu_devp->cdev.owner = THIS_MODULE;

    /* Connect the major/minor number to the cdev */
    ret = cdev_add(&imu_devp->cdev, (imu_dev_number), 1);

    if (ret) {
        printk("Bad cdev\n");
        return ret;
    }

    /* Send uevents to udev, so it'll create /dev nodes */
    imu_dev_device = device_create(imu_dev_class, NULL, MKDEV(MAJOR(imu_dev_number), 0), NULL, DEVICE_NAME);    

    ret = gpio_request(I2CMUX, "I2CMUX");
    if(ret)
    {
        printk("GPIO %d is not requested.\n", I2CMUX);
    }

    ret = gpio_direction_output(I2CMUX, 0);
    if(ret)
    {
        printk("GPIO %d is not set as output.\n", I2CMUX);
    }
    gpio_set_value_cansleep(I2CMUX, 0); // Direction output didn't seem to init correctly.  

    ret = gpio_request(GPIO_2_MUXSEL, "gpio_2_Mux");
    if(ret < 0)
    {
        printk("Error Requesting GPIO2_Mux.\n");
        return -1;
    }
    ret = gpio_direction_output(GPIO_2_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting GPIO_2_MUXSEL output.\n");
    }

    ret = gpio_request(GPIO_2_PU_MUXSEL, "gpio_2_puMux");
    if(ret < 0)
    {
        printk("Error Requesting GPIO2_puMux.\n");

        return -1;
    }
    ret = gpio_direction_output(GPIO_2_PU_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting GPIO_2_puMux output.\n");
    }

    ret = gpio_request(USS_ECHO, "gpio_echo");
    if(ret < 0)
    {
        printk("Error Requesting echo Pin.\n");
        return -1;
    }
    ret = gpio_direction_input(USS_ECHO);
    if(ret < 0)
    {
        printk("Error Setting echo Pin Input.\n");
    }

    gpio_set_value_cansleep(GPIO_2_MUXSEL, 0); // Set GPIO 2 Mux to 0.
    gpio_set_value_cansleep(GPIO_2_PU_MUXSEL, 0); // Set GPIO 2 pullup Mux to 0.


    // Create Adapter using:
    imu_devp->adapter = i2c_get_adapter(0); // /dev/i2c-0
    if(imu_devp->adapter == NULL)
    {
        printk("Could not acquire i2c adapter.\n");
        return -1;
    }

    // Create Client Structure
    imu_devp->client = (struct i2c_client*) kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
    imu_devp->client->addr = DEVICE_ADDR; // Device Address (set by hardware)
    snprintf(imu_devp->client->name, I2C_NAME_SIZE, "i2c_imu");
    imu_devp->client->adapter = imu_devp->adapter;
   
    // Initialize Input Interface
    input_imu_dev = input_allocate_device();
    if (!input_imu_dev) {
        printk(KERN_ERR "Key Test: Not enough memory\n");
        return -(ENOMEM);
    }

    input_imu_dev->name = "Key Example";
    /* phys is unique on a running system */
        input_imu_dev->phys = "A/Fake/Path";
        input_imu_dev->id.bustype = BUS_HOST;
        input_imu_dev->id.vendor = 0x0001;
        input_imu_dev->id.product = 0x0001;
        input_imu_dev->id.version = 0x0100;

    //Initialize joystick events.
    set_bit(EV_ABS, input_imu_dev->evbit);

    //Initialize ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ.
    set_bit(ABS_X, input_imu_dev->absbit);
    set_bit(ABS_Y, input_imu_dev->absbit);
    set_bit(ABS_Z, input_imu_dev->absbit);

    set_bit(ABS_RX, input_imu_dev->absbit);
    set_bit(ABS_RY, input_imu_dev->absbit);
    set_bit(ABS_RZ, input_imu_dev->absbit);

    //Set the parameters for all the ABS events.
    input_set_abs_params(input_imu_dev, ABS_X, -32767, 32767, 4, 8);
    input_set_abs_params(input_imu_dev, ABS_Y, -32767, 32767, 4, 8);
    input_set_abs_params(input_imu_dev, ABS_Z, -32767, 32767, 4, 8);

    input_set_abs_params(input_imu_dev, ABS_RX, -32767, 32767, 4, 8);
    input_set_abs_params(input_imu_dev, ABS_RY, -32767, 32767, 4, 8);
    input_set_abs_params(input_imu_dev, ABS_RZ, -32767, 32767, 4, 8);

    //Register the input device.
    ret = input_register_device(input_imu_dev);
    if (ret) {
        printk(KERN_ERR "Key Test: Failed to register device\n");
        return -1;
    }

    //Start the main thread.
    WorkThread = kthread_run(my_work, (void *)dummy,"work_test");

    return 0;
}
/* Driver Exit */
void __exit imu_driver_exit(void)
{

    if(WorkThread)
    {
        kthread_stop(WorkThread);
    }

    gpio_free(I2CMUX);

    //Release all the GPIOs.
    gpio_free(GPIO_2_MUXSEL);
    gpio_free(GPIO_2_PU_MUXSEL);
    gpio_free(USS_ECHO);

    input_unregister_device(input_imu_dev);

    //Free the IRQ.
    //free_irq(echo_irq, (void *)(echo_irq));

    // Close and cleanup
    i2c_put_adapter(imu_devp->adapter);
    kfree(imu_devp->client);

    /* Release the major number */
    unregister_chrdev_region((imu_dev_number), 1);

    /* Destroy device */
    device_destroy (imu_dev_class, MKDEV(MAJOR(imu_dev_number), 0));
    cdev_del(&imu_devp->cdev);
    kfree(imu_devp);
    
    /* Destroy driver_class */
    class_destroy(imu_dev_class);
}

module_init(imu_driver_init);
module_exit(imu_driver_exit);
MODULE_LICENSE("GPL v2");
