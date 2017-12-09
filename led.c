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
#include <linux/slab.h> /* kmalloc() */
#include <linux/kernel.h> /* printk() */
#include <linux/module.h>
#include <linux/fs.h> /* everything... */
#include <asm/uaccess.h> /* copy_from/to_user */
#include <linux/proc_fs.h>

MODULE_LICENSE("Dual BSD/GPL");

#define LED_0 28
#define LED_1 29
#define LED_2 30
#define LED_3 31
#define BUTTON_0 113
#define BUTTON_1 117
#define BUTTON_2 101
#define PWM_1 16

static int led_major = 61;
static char *led_buffer;
static int capacity = 10;
static int led_open(struct inode *inode, struct file *filp);
static int led_release(struct inode *inode, struct file *filp);
static ssize_t led_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static ssize_t led_write(struct file *filp,const char *buf, size_t timer_length, loff_t *f_pos);
static int buffer_len;

struct file_operations led_fops = {
	open: led_open,
	release: led_release,
	write:led_write,
	read: led_read
};


static int led_open(struct inode *inode, struct file *filp)
{
	/* Success */
	return 0;
}

static int led_release(struct inode *inode, struct file *filp)
{
	/* Success */
	return 0;
}

static int my_init_module(void)
{
	int result;
	/* Registering device */
	result = register_chrdev(led_major, "led", &led_fops);
	if (result < 0)
	{
		printk(KERN_ALERT
			"led: cannot obtain major number %d\n", led_major);
		return result;
	}

	led_buffer = kmalloc(capacity*sizeof(char), GFP_KERNEL); 
	if (!led_buffer)
	{ 
		// printk(KERN_ALERT "Insufficient kernel memory\n"); 
		result = -ENOMEM;
		//		goto fail;
		return result;
	} 
	memset(led_buffer, 0, (capacity*sizeof(char)));

	//printk("test\n");

	pxa_gpio_mode(LED_1 | GPIO_OUT);	
	pxa_gpio_mode(LED_2 | GPIO_OUT);
	pxa_gpio_mode(LED_3 | GPIO_OUT);	

	pxa_gpio_set_value(LED_1, 1);		
	pxa_gpio_set_value(LED_2, 1);
	pxa_gpio_set_value(LED_3, 1);		


	return 0;
}

static ssize_t led_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{ 
  printk("Starting read \n");
  if ((led_buffer + *f_pos) == 0){
    return 0;
  }
  printk("1 \n");


  
  //  timer_len = strlen(timer_buffer);

  if (*f_pos >= capacity) {
    return 0; //end of buffer reached
  }

  if (count > capacity - *f_pos)
    count = capacity - *f_pos;

  if (copy_to_user(buf, led_buffer + *f_pos, count)) {
    printk("Not copying to user /n");        
    return -EFAULT;
  }

  
  *f_pos += count;
  return count;
}

static ssize_t led_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
  if (copy_from_user(led_buffer, buf, count)){
    return -EFAULT;
  }

  *f_pos += count; 
  return count;

}


module_init(my_init_module);
//module_exit(my_cleanup_module);