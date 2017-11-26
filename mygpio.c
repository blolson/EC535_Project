#ifndef __KERNEL__
#define __KERNEL__
#endif

#ifndef MODULE
#define MODULE
#endif

#include <linux/timer.h>
#include <linux/jiffies.h>

//this is from proc_gpio.c
#include <linux/init.h>
//#include <asm/hardware.h>
#include <asm/uaccess.h>
//this is from proc_gpio.c


#include <linux/module.h>
#include <linux/interrupt.h>
#include <asm/arch/pxa-regs.h>
#include <asm-arm/arch/hardware.h>
#include <asm/irq.h>
#include <linux/slab.h> /* kmalloc() */
#include <linux/kernel.h> /* printk() */
#include <linux/module.h>
#include <linux/fs.h> /* everything... */
#include <asm/uaccess.h> /* copy_from/to_user */

//Added---
#include <linux/i2c.h>
#include <linux/i2c-dev.h> /* for I2C_SLAVE */
//--------

//Gyrometer registers
#define LSM9DS0_REGISTER_WHO_AM_I_G             0x0F
#define LSM9DS0_REGISTER_CTRL_REG1_G            0x20
#define LSM9DS0_REGISTER_CTRL_REG3_G            0x22
#define LSM9DS0_REGISTER_CTRL_REG4_G            0x23
#define LSM9DS0_REGISTER_OUT_X_L_G              0x28
#define LSM9DS0_REGISTER_OUT_X_H_G              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_G              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_G              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_G              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_G              0x2D

//Accelerometer and Magnetometer registers
#define LSM9DS0_REGISTER_TEMP_OUT_L_XM          0x05
#define LSM9DS0_REGISTER_TEMP_OUT_H_XM          0x06
#define LSM9DS0_REGISTER_STATUS_REG_M           0x07
#define LSM9DS0_REGISTER_OUT_X_L_M              0x08
#define LSM9DS0_REGISTER_OUT_X_H_M              0x09
#define LSM9DS0_REGISTER_OUT_Y_L_M              0x0A
#define LSM9DS0_REGISTER_OUT_Y_H_M              0x0B
#define LSM9DS0_REGISTER_OUT_Z_L_M              0x0C
#define LSM9DS0_REGISTER_OUT_Z_H_M              0x0D
#define LSM9DS0_REGISTER_WHO_AM_I_XM            0x0F
#define LSM9DS0_REGISTER_INT_CTRL_REG_M         0x12
#define LSM9DS0_REGISTER_INT_SRC_REG_M          0x13
#define LSM9DS0_REGISTER_CTRL_REG1_XM           0x20
#define LSM9DS0_REGISTER_CTRL_REG2_XM           0x21
#define LSM9DS0_REGISTER_CTRL_REG5_XM           0x24
#define LSM9DS0_REGISTER_CTRL_REG6_XM           0x25
#define LSM9DS0_REGISTER_CTRL_REG7_XM           0x26
#define LSM9DS0_REGISTER_OUT_X_L_A              0x28

#define LSM9DS0_REGISTER_OUT_Y_H_G              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_G              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_G              0x2D

//Accelerometer and Magnetometer registers
#define LSM9DS0_REGISTER_TEMP_OUT_L_XM          0x05
#define LSM9DS0_REGISTER_TEMP_OUT_H_XM          0x06
#define LSM9DS0_REGISTER_STATUS_REG_M           0x07
#define LSM9DS0_REGISTER_OUT_X_L_M              0x08
#define LSM9DS0_REGISTER_OUT_X_H_M              0x09
#define LSM9DS0_REGISTER_OUT_Y_L_M              0x0A
#define LSM9DS0_REGISTER_OUT_Y_H_M              0x0B
#define LSM9DS0_REGISTER_OUT_Z_L_M              0x0C
#define LSM9DS0_REGISTER_OUT_Z_H_M              0x0D
#define LSM9DS0_REGISTER_WHO_AM_I_XM            0x0F
#define LSM9DS0_REGISTER_INT_CTRL_REG_M         0x12
#define LSM9DS0_REGISTER_INT_SRC_REG_M          0x13
#define LSM9DS0_REGISTER_CTRL_REG1_XM           0x20
#define LSM9DS0_REGISTER_CTRL_REG2_XM           0x21
#define LSM9DS0_REGISTER_CTRL_REG5_XM           0x24
#define LSM9DS0_REGISTER_CTRL_REG6_XM           0x25
#define LSM9DS0_REGISTER_CTRL_REG7_XM           0x26
#define LSM9DS0_REGISTER_OUT_X_L_A              0x28
#define LSM9DS0_REGISTER_OUT_X_H_A              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_A              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_A              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_A              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_A              0x2D

//Angular rate SAD+read/write patterns
#define LSM9DS0_ADDRESS_GYRO_READ               0xd5 //(11010101) - Read
#define LSM9DS0_ADDRESS_GYRO_WRITE              0xd4 //(11010100) - Write

//Linear acceleration and magnetic sensor SAD+read/write patterns
#define LSM9DS0_ADDRESS_ACCELMAG_READ           0x3d //(00111101) - Read
#define LSM9DS0_ADDRESS_ACCELMAG_WRITE          0x3c //(00111100) - Write


static int mygpio_major = 61;
static unsigned capacity = 128;
static int mygpio_len;
static char *mygpio_buffer;

void write_byte(int, char, char, char);
char read8(int, char, char);
static int mygpio_open(struct inode *inode, struct file *filp);
static int mygpio_release(struct inode *inode, struct file *filp);

struct file_operations mygpio_fops = {
	open: mygpio_open,
	release: mygpio_release
};

irqreturn_t i2c_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	//Read ISR: IDBR transmit empty (1), Unit Busy (1), R/nW bit (0).
	if(ISR & ISR_ITE & ISR_UB & (~ISR & 1))
	{
		//Write 0b1 to the ISR[ITE] bit to clear interrupt.
		ISR &= ~ISR_ITE;
		printk("progress!");	
	}
	else
	{
		printk("nope! no good!\n");
	}
	
        return IRQ_HANDLED;
}


static int my_init_module(void)
{
	int result;
	/* Registering device */
	/*
	result = register_chrdev(mygpio_major, "mygpio", &mygpio_fops);
	if (result < 0)
	{
		printk(KERN_ALERT
			"mygpio: cannot obtain major number %d\n", mygpio_major);
		return result;
	}

	mygpio_buffer = kmalloc(capacity, GFP_KERNEL); 
	if (!mygpio_buffer)
	{ 
		printk(KERN_ALERT "Insufficient kernel memory\n"); 
		result = -ENOMEM;
	} 
	memset(mygpio_buffer, 0, capacity);
	mygpio_len = 0;	
	*/
	printk("test\n");

	pxa_gpio_mode(GPIO117_I2CSCL_MD);	
	pxa_gpio_mode(GPIO118_I2CSDA_MD);	
	pxa_set_cken(CKEN14_I2C, 1);

	//Set the slave address in the ISAR.
	ISAR = LSM9DS0_ADDRESS_ACCELMAG_WRITE >> 1; 
	
	//Set the ICR[IUE] and ICR[SCLE] bits to enable the I2C interface and SCL.
	ICR |= ICR_IUE;
	ICR |= ICR_SCLE;

	//Load target slave address and R/nW bit in the IDBR. R/nW must be 0 for a write.
	IDBR = LSM9DS0_ADDRESS_ACCELMAG_WRITE;

	//Initiate the write. Set ICR[START], clear ICR[STOP], clear ICR[ALDIE], set ICR[TB].
	ICR |= ICR_START;
	ICR &= ~ICR_STOP;
	ICR &= ~ICR_ALDIE;
	ICR |= ICR_TB;

	int irq = IRQ_I2C;
	printk( "irq num: %d", IRQ_I2C);
	
	//When an IDBR transmit-empty interrupt occurs: Read ISR: IDBR transmit empty (1), Unit Busy (1), R/nW bit (0).
	if (request_irq(irq, &i2c_handler, SA_INTERRUPT | SA_TRIGGER_RISING | SA_TRIGGER_FALLING,
                                "mygpio", NULL) != 0 ) {
                printk ( "irq not acquired \n" );
                return -1;
        }

	
	return 0;
}

static void my_cleanup_module(void)
{
	/* Freeing the major number */
	//unregister_chrdev(mygpio_major, "mygpio");
	/* Freeing buffer memory */
	if (mygpio_buffer)
	{
		kfree(mygpio_buffer);
	}

	free_irq(IRQ_I2C, NULL);
}

static int mygpio_open(struct inode *inode, struct file *filp)
{
	/* Success */

	return 0;
}

static int mygpio_release(struct inode *inode, struct file *filp)
{
	/* Success */
	return 0;
}

void write_byte(int fileHandler, char _address, char _register, char _data)
{
        char data[2];

	data[0] = _register;
        data[1] = _data;

        /* write two bytes */
        //write(fileHandler, data, 2);
}

char read8(int fileHandler, char _address, char _register)
{
        char data[2];

        data[0] = _register;
        data[1] = 0;
	
	/*
        if (read(fileHandler, &data, 2) != 2) {
                return -1;
        }
	*/

	return data[1];
}

module_init(my_init_module);
module_exit(my_cleanup_module);
