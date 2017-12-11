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
#include <linux/timer.h>

MODULE_LICENSE("Dual BSD/GPL");

#define LED_1 28
#define LED_2 30
#define LED_3 29
#define LED_ON_DURATION 2

static short led_binary[3] = {1, 2, 4};
static short led_GPIO[3] = {28, 30, 29};
static struct timer_list led_off_timer[3];
static int led_major = 61;
static char *led_buffer;
static int capacity = 10;
static int led_open(struct inode *inode, struct file *filp);
static int led_release(struct inode *inode, struct file *filp);
static ssize_t led_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static ssize_t led_write(struct file *filp,const char *buf, size_t timer_length, loff_t *f_pos);
static void leds_off(unsigned long);

struct file_operations led_fops = {
	open: led_open,
	release: led_release,
	write:led_write,
	read: led_read
};


static int led_open(struct inode *inode, struct file *filp)
{
	/* Success */
	printk("file opened \n");
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
	int iter;

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

        for(iter = 0; iter < 3; iter++)
        {
		printk("Turning on led %d \n", led_GPIO[iter]);
                pxa_gpio_set_value(led_GPIO[iter], 1);
                if(timer_pending(&(led_off_timer[iter])))
                {
			del_timer(&(led_off_timer[iter]));
                }

                setup_timer(&(led_off_timer[iter]), leds_off, iter);
                mod_timer(&(led_off_timer[iter]), jiffies + (HZ * LED_ON_DURATION));
        }

	return 0;
}

static ssize_t led_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{ 
  // printk("Starting read \n");
  if ((led_buffer + *f_pos) == 0){
    return 0;
  }
  //printk("1 \n");


  
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
	int iter;
	int led;
	int mode;
	
	//printk("START LED WRITE %d\n", count);
	if (copy_from_user(led_buffer, buf, count)){
		return -EFAULT;
	}
  
	sscanf(led_buffer, "%d,%d",&led,&mode);
	//printk("Here's what I got...mode:%d, led:%d\n", mode, led);

	//mode == 0 means 'overwrite the current led configuration'
	if(mode == 0)
	{
                pxa_gpio_set_value(LED_1, 0);
                pxa_gpio_set_value(LED_2, 0);
                pxa_gpio_set_value(LED_3, 0);
	}

	//mode == 'anything else' means 'add to the current led configuration'

	for(iter = 0; iter < 3; iter++)
	{
		if(led & led_binary[iter])
		{
			//printk("Turning on led %d \n", led_GPIO[iter]);
			pxa_gpio_set_value(led_GPIO[iter], 1);		
			if(timer_pending(&(led_off_timer[iter])))
			{
				del_timer(&(led_off_timer[iter]));
			}
	
			setup_timer(&(led_off_timer[iter]), leds_off, iter);
			mod_timer(&(led_off_timer[iter]), jiffies + (HZ * LED_ON_DURATION));
		}
	}

	*f_pos += count; 
	return count;
}

static void my_cleanup_module(void)
{
	/* Freeing the major number */
	unregister_chrdev(led_major, "led");
	/* Freeing buffer memory */
	if (led_buffer)
	{
		kfree(led_buffer);
	}
}

static void leds_off(unsigned long data) {
	

	pxa_gpio_set_value(LED_1, 0);		
	pxa_gpio_set_value(LED_2, 0);
	pxa_gpio_set_value(LED_3, 0);		
}

module_init(my_init_module);
module_exit(my_cleanup_module);
