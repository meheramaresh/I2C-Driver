/* ----------------------------------------------- DRIVER bus ---------------------------------------------------
 
 I2C Driver to perform basic operations on an EEPROM
 
 ----------------------------------------------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/moduleparam.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define INPUT_DEVICE_NAME 	"i2c_flash"  	/* Device name input queue */
#define I2C_MINORS		256		/* I2c Minor number */
#define SUCCESS			0		/* Macro for Success */
#define FAILURE			-1		/* Macro for Failure */
#define GP_LED			26 		/* GPIO26 corresponds to Arduino PIN8 */
#define GPIO_I2C		29		/* GPIO for SDA */
#define EEPROM_READY		0x0A		/* Macro for EEPRO ready state */
#define EEPROM_BUSY		0x0B		/* Macro for EEPRO busy state */

/* IOCTL cases */
#define FLASHGETS		0xA0
#define FLASHGETP		0xB0
#define FLASHSETP		0xC0
#define FLASHERASE		0xD0

/* macros for write buffer */
#define BUF_EMPTY		0x10
#define BUF_FULL		0x20


/* Global variables */
static LIST_HEAD(i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

int i2c_major = 0;					/* Major number */
short g_currentAddr = 0x0000;				/* Current write Address */
int g_minorNumber;					/* Minor Number */
int g_eepromStatus = EEPROM_READY;
static struct class *i2c_dev_class;          		/* Tie with the device model */
static struct workqueue_struct *g_i2cflash_wq;	/* Work queue for i2c_flash */
int g_readBufStatus = BUF_EMPTY;

/*Device message structure */
struct work_data_struct {
	struct work_struct workStruct;	/* Work Structure */
	struct i2c_client *client;	/* I2c Client structure */
	char *kernelBuf;		/* Kernel message data */
	int count;			/* Page count */		
	short currentAddr;		/* Current address location */
};

/* Per device structure */
struct i2c_dev {
	struct i2c_adapter *adap;       /* The i2c client structure */
	struct device *dev;		/* i2c Device */
	struct list_head list;
	short currentAddr;
	struct mutex busMutex;		/* Mutex for the device */
};

/* global structures for work queue */
struct work_data_struct *g_workDataRead;
struct work_data_struct *g_workDataWrite;

/* Global buffers */
char *g_writeData;
char *g_readData;

/* Function Protypes */
int eeprom_write(struct work_struct *work);
int eeprom_read(struct work_struct *work);
int eeprom_erase(struct i2c_client *client);

/*
 * get I2c dev structure using device minor number
 */
static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
        struct i2c_dev *i2c_dev;

        spin_lock(&i2c_dev_list_lock);
	
	/* Search through the list */
        list_for_each_entry(i2c_dev, &i2c_dev_list, list) {
                if (i2c_dev->adap->nr == index)
                        goto found;
        }
        i2c_dev = NULL;
found:
       spin_unlock(&i2c_dev_list_lock);
        return i2c_dev;
}

/*
 * Get free i2c dev.
 */
static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap){
        struct i2c_dev *i2c_dev;

        if (adap->nr >= I2C_MINORS) {
                printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
                       adap->nr);
                return ERR_PTR(-ENODEV);
        }

        i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
        if (!i2c_dev)
                return ERR_PTR(-ENOMEM);
        i2c_dev->adap = adap;

        spin_lock(&i2c_dev_list_lock);
        list_add_tail(&i2c_dev->list, &i2c_dev_list);
        spin_unlock(&i2c_dev_list_lock);
        return i2c_dev;
}

/*
 * free the i2c_dev 
 */
static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
        spin_lock(&i2c_dev_list_lock);
        list_del(&i2c_dev->list);
        spin_unlock(&i2c_dev_list_lock);
        kfree(i2c_dev);
}

/*
* Open i2c_flash driver
*/
int i2cflash_driver_open(struct inode *inode, struct file *file){
        unsigned int minor = g_minorNumber;	
        struct i2c_client *client;
        struct i2c_adapter *adap;
        struct i2c_dev *i2c_dev;

        i2c_dev = i2c_dev_get_by_minor(minor);
        if (!i2c_dev)
                return -ENODEV;

        adap = i2c_get_adapter(i2c_dev->adap->nr);
        if (!adap)
                return -ENODEV;

        /* This creates an anonymous i2c_client, which may later be
         * pointed to some address using I2C_SLAVE or I2C_SLAVE_FORCE.
         *
         * This client is ** NEVER REGISTERED ** with the driver model
         * or I2C core code!!  It just holds private copies of addressing
         * information and maybe a PEC flag.
         */
        client = kzalloc(sizeof(*client), GFP_KERNEL);
        if (!client) {
                i2c_put_adapter(adap);
                return -ENOMEM;
        }
        snprintf(client->name, I2C_NAME_SIZE, "i2c_flas");

        client->adapter = adap;
	client->addr = 0x54;
        file->private_data = client;
	
        return 0;
}

/*
 * Release i2c_flash driver
 */
int i2cflash_driver_release(struct inode *inode, struct file *file){
	struct i2c_client *client = file->private_data;

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*
 * IOCTL operations for i2c_flash
 */
static long i2cflash_driver_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	struct i2c_client *client = file->private_data;
	int byteCount = 0;
	short pageNumber = 0;
	short tempAddr = 0x0000;
	short addVal[2];
	int ret = FAILURE;
	
	switch(cmd){
		case FLASHGETS:
			ret = g_eepromStatus;
			break;
		
		case FLASHGETP:	
			/* Return back the current Address */
			ret = g_currentAddr;
			break;

		case FLASHSETP:
			/* Get the address from the user */
			pageNumber = arg;

			/* Calculate the address */
			byteCount = (pageNumber * 64);
			tempAddr += byteCount;
						
			/* Update the address */
			if(tempAddr <= 0x7FFF){
				/* Set the current address */
				g_currentAddr = tempAddr;
				
				addVal[0] = (g_currentAddr >> 8) & 0x00FF; 		/* Save the higher byte */
				addVal[1] = g_currentAddr & 0x00FF;		/* Save the lower byte */

				ret = (i2c_master_send(client, addVal, 2) > 0) ? SUCCESS : FAILURE;				
			}

			break;

		case FLASHERASE:
			/* Check if eeprom is busy, if not busy erase it */
			if(g_eepromStatus == EEPROM_BUSY){
				printk("flash erase busy\n");
				ret = -EBUSY;
			}
			else{
				printk("flash erase free\n");
				ret = eeprom_erase(client);
			}
			break;

		default:
			break;
	}

	return ret;

}

/* Internal function to erase EEPROM */
int eeprom_erase(struct i2c_client *client){
	short tempAddr = 0x0000;
	int ret = FAILURE;
	int tempRet = 0;
	int loopCount;
	char *dataBuf;
	char *tmpBuf;

	/* Set EEPROM status to BUSY */
	g_eepromStatus = EEPROM_BUSY;

	/* Turn on LED */
	gpio_set_value(GP_LED, 1);

	dataBuf = kzalloc(64, GFP_KERNEL);
	tmpBuf = kzalloc(66, GFP_KERNEL);

	printk("STRING - ");
	/* Create a buffer of 64 bytes */
	loopCount = 0;
	while(loopCount < 64){
		*(dataBuf + loopCount) = 0xFF;
		printk("%c", *(dataBuf + loopCount));
		loopCount++;
	}
	printk("\n");

	for(loopCount = 0; loopCount < 512; loopCount++){
	
		/* Assign address at the first two bytes */
		printk("\nErasing page address 0x%x\n", tempAddr);
		*tmpBuf = (tempAddr >> 8) & 0x00FF; 		/* Save the higher byte */
		*(tmpBuf + 1) = tempAddr & 0x00FF;		/* Save the lower byte */

		/* Copy the data to the buffer */
		memcpy(tmpBuf + 2, dataBuf, 64);
	
		/* Write to the EEPROM */
		tempRet = i2c_master_send(client, tmpBuf, 64 + 2);
		if(tempRet < 0){
			printk("Write Failed");
			ret = FAILURE;
			goto Exit;		
		}
		
		/* Move to the next page */		
		tempAddr += 64;

	}

Exit:

	/* Set EEPROM status to READY */
	g_eepromStatus = EEPROM_READY;

	/* Turn off LED */
	gpio_set_value(GP_LED, 0);

	kfree(tmpBuf);
	kfree(dataBuf);
	
	return ret;
}


/*
 * Internal function to write into the EEPROM
 */
int eeprom_write(struct work_struct *work){
	struct work_data_struct *workData = container_of(work, struct work_data_struct, workStruct);

	struct i2c_client *client = workData->client;
	char *buf = g_writeData;// = workData->kernelBuf;
	int pageCount = workData->count;
	short currentAddr = workData->currentAddr;

	int ret = SUCCESS;
	int tempRet;
	int loopCount = 0;
	char tmpBuf[66];

	/* Set EEPROM status */
	g_eepromStatus = EEPROM_BUSY;

	/* Turn on LED */
	gpio_set_value_cansleep(GP_LED, 1);

	while(loopCount < pageCount){
		/* Assign address at the first two bytes */
		tmpBuf[0] = (currentAddr >> 8) & 0x00FF; 		/* Save the higher byte */
		tmpBuf[1] = currentAddr & 0x00FF;		/* Save the lower byte */

		/* Copy the data to the buffer */
		memcpy(&tmpBuf[2], buf, 64);
		
		/* Write to the EEPROM */
		tempRet = i2c_master_send(client, tmpBuf, 64 + 2);

		//printk("gpio value %d\n", gpio_get_value(GPIO_I2C));
		gpio_direction_output(GPIO_I2C, 0);

		if(tempRet < 0){
			printk("Write Failed, Ret value : %d", tempRet);
			ret = FAILURE;
			break;		
		}

		memset(tmpBuf, 0, 66); 
		currentAddr += 64;		
		currentAddr %= 0x8000;
		buf += 64;
		loopCount++;
	}

	
	if(ret != FAILURE){
		g_currentAddr += (pageCount * 64);
		g_currentAddr %= 0x8000;
		printk("Read/Write pointer moved to 0x%x\n", g_currentAddr);
	}

	/* Turn off LED */
	gpio_set_value_cansleep(GP_LED, 0);

	/* Set EEPROM status */
	g_eepromStatus = EEPROM_READY;

	return ret;
}

/*
 * Write to bus driver
 */
ssize_t i2cflash_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos){
        int ret = 0;
        char *tmp;
	int byteCount = count * 64;

	if(count <= 0)
	{
		ret = -EINVAL;
	}
	else if(g_eepromStatus != EEPROM_BUSY){
		/* Check if the number of pages required is more than the memory available */
	  	printk("count - %d\n", count);

		/* Copy the data from the user */
		tmp = memdup_user(buf, byteCount);
		if (IS_ERR(tmp))
		        return PTR_ERR(tmp);

		g_writeData = (char*)kmalloc(byteCount, GFP_KERNEL);

		memcpy(g_writeData, tmp, byteCount);

		/* Update queue */
		INIT_WORK((struct work_struct*)g_workDataWrite, eeprom_write);
		g_workDataWrite->client = file->private_data;
		g_workDataWrite->count = count;
		g_workDataWrite->currentAddr = g_currentAddr;
		g_workDataWrite->kernelBuf = g_writeData;
		queue_work(g_i2cflash_wq, (struct work_struct*)g_workDataWrite);

		ret = SUCCESS;
	}
	else{
		ret = -EBUSY;
	}
	
        return ret;
}

/*
 * Internal function to read from the EEPROM
 */
int eeprom_read(struct work_struct *work){
	struct work_data_struct *workData = (struct work_data_struct*)work;
	struct i2c_client *client = workData->client;
	int pageCount = workData->count;
	int ret = FAILURE;

	/* Set EEPROM status */
	g_eepromStatus = EEPROM_BUSY;
	
	/* Turn on LED */
//	gpio_set_value(GP_LED, 1);

	/* Read the data from EEPROM */
	ret = i2c_master_recv(client, g_readData, (pageCount * 64));
	if(ret > 0){
		g_readBufStatus = BUF_FULL;
		ret = SUCCESS;
	}
	else{
		printk("Reading failed\n");
	}


	/* Turn off LED */
//	gpio_set_value(GP_LED, 0);

	/* Set EEPROM status */
	g_eepromStatus = EEPROM_READY;
	
	return ret;
		
}

/*
 * Read to bus driver
 */
ssize_t i2cflash_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos){
        char *tmp = g_readData;
        int ret = 0;
	int byteCount = count * 64;
        struct i2c_client *client = file->private_data;

	if(count <= 0){
		ret = -EINVAL;	
	}
	else if(g_eepromStatus == EEPROM_READY && g_readBufStatus == BUF_EMPTY){

		g_readData = (char*)kzalloc(byteCount, GFP_KERNEL);
	
		/* Update queue */
		INIT_WORK((struct work_struct*)g_workDataRead, eeprom_read);				
		g_workDataRead->client = client;
		g_workDataRead->kernelBuf = tmp;
		g_workDataRead->count = count;
		g_workDataRead->currentAddr = g_currentAddr;
		queue_work(g_i2cflash_wq, (struct work_struct*)g_workDataRead);
		
		ret = -EAGAIN;
	}
	else if(g_eepromStatus == EEPROM_BUSY && g_readBufStatus == BUF_EMPTY){
		ret = -EBUSY;
	}
	else if(g_readBufStatus == BUF_FULL){
		/* Update the current address pointer */		
		g_currentAddr += byteCount;
		g_currentAddr %= 0x8000;

		/* Copy the data to the user buffer */
	        ret = copy_to_user(buf, g_readData, byteCount) ? -EFAULT : ret;
	
		g_readBufStatus = BUF_EMPTY;
	}	

        return ret;
}


/* File operations structure. Defined in linux/fs.h */
static struct file_operations i2cflash_fops = {
	.owner		= THIS_MODULE,			/* Owner */
	.open		= i2cflash_driver_open,		/* Open method */
	.release	= i2cflash_driver_release,	/* Release method */
	.unlocked_ioctl	= i2cflash_driver_ioctl,	/* Ioctl method */
	.write		= i2cflash_driver_write,	/* Write method */
	.read		= i2cflash_driver_read,		/* Read method */
    
};

/*
 * Attach adapter
 */
static int i2cflash_attach_adapter(struct device *dev, void *dummy){
	struct i2c_adapter *adap;
        struct i2c_dev *i2c_dev;
        int res;

        if (dev->type != &i2c_adapter_type)
                return 0;

	/* Get the adapter */
        adap = to_i2c_adapter(dev);

        i2c_dev = get_free_i2c_dev(adap);
        if (IS_ERR(i2c_dev))
                return PTR_ERR(i2c_dev);

        /* register this i2c device with the driver core */
        i2c_dev->dev = device_create(i2c_dev_class, &adap->dev,
                                     MKDEV(i2c_major, 0), NULL, "i2c_flash");
        if (IS_ERR(i2c_dev->dev)) {
                res = PTR_ERR(i2c_dev->dev);
                goto error;
        }

        pr_debug("i2c_flash: adapter [%s] registered as minor %d\n",
                 adap->name, adap->nr);
	
	/* Save the minor number */
	g_minorNumber = i2c_dev->adap->nr;
	i2c_dev->currentAddr = 0x0000;
	
        return 0;
error:
        return_i2c_dev(i2c_dev);
        return res;
}

/*
 * Function to detatch adapter
 */
static int i2cflash_detach_adapter(struct device *dev, void *dummy){
        struct i2c_adapter *adap;
        struct i2c_dev *i2c_dev;

        if (dev->type != &i2c_adapter_type)
                return 0;

	/* Get the adapter */
        adap = to_i2c_adapter(dev);

        i2c_dev = i2c_dev_get_by_minor(adap->nr);
        if (!i2c_dev) /* attach_adapter must have failed */
                return 0;

        return_i2c_dev(i2c_dev);
        device_destroy(i2c_dev_class, MKDEV(i2c_major, adap->nr));

        pr_debug("i2c_flash: adapter [%s] unregistered\n", adap->name);
        return 0;
}

/* Internal function for attach or detatch adapter */
static int i2cflash_notifier_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct device *dev = data;

	switch (action) {
		case BUS_NOTIFY_ADD_DEVICE:
		return i2cflash_attach_adapter(dev, NULL);

		case BUS_NOTIFY_DEL_DEVICE:
		return i2cflash_detach_adapter(dev, NULL);
	}

	return 0;
}

static struct notifier_block i2cflash_notifier = {
	.notifier_call = i2cflash_notifier_call,
};
 

/*
 * Driver Initialization
 */
int __init i2cflash_driver_init(void){
	int ret = FAILURE;

	/* Request dynamic allocation of a device major number for bus_in_q */
	i2c_major = register_chrdev(0, INPUT_DEVICE_NAME, &i2cflash_fops);
	if (i2c_major < 0) {
		goto out;	
	}

	/* Populate sysfs entries */
	i2c_dev_class = class_create(THIS_MODULE, INPUT_DEVICE_NAME);
	if (IS_ERR(i2c_dev_class)) {
                ret = PTR_ERR(i2c_dev_class);
	        goto out_unreg_chrdev;
        }

	ret = bus_register_notifier(&i2c_bus_type, &i2cflash_notifier);	
	if (ret)
		goto out_unreg_class;

	i2c_for_each_dev(NULL, i2cflash_attach_adapter);

	/* Create work queue */
	g_i2cflash_wq = create_workqueue("i2c_flash_work_queue");	

	/* Global work structures */
	g_workDataWrite = (struct work_data_struct*)kmalloc(sizeof(struct work_data_struct), GFP_KERNEL);			
	g_workDataRead = (struct work_data_struct*)kmalloc(sizeof(struct work_data_struct), GFP_KERNEL);

	/* Turn On the LED */
	gpio_request_one(GP_LED, GPIOF_OUT_INIT_LOW, "led1");
	gpio_set_value_cansleep(GP_LED, 0);

	/* Turn on SDA and SCL */
	gpio_request_one(GPIO_I2C, GPIOF_OUT_INIT_LOW, "i2c");
	gpio_set_value_cansleep(GPIO_I2C, 0);

	printk("I2c_flash driver initialized.\n");
	return 0;

out_unreg_class:
        class_destroy(i2c_dev_class);
out_unreg_chrdev:
        unregister_chrdev(i2c_major, INPUT_DEVICE_NAME);
out:
        printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
        return ret;
}

/* 
 * Driver Exit 
 */
void __exit i2cflash_driver_exit(void){
	
	/* Destroy work queue */
	if(g_i2cflash_wq != NULL){
		/* Cancel all the delayed works */
		//cancel_delayed_work(i2cflash_wq);

		/* Clear the work queue */
		flush_workqueue(g_i2cflash_wq);
	
		/* Destroy the work queue */
		destroy_workqueue(g_i2cflash_wq);
	}

	/* Release the major number */
	bus_unregister_notifier(&i2c_bus_type, &i2cflash_notifier);

	i2c_for_each_dev(NULL, i2cflash_detach_adapter);

	/* Destroy driver_class */
	class_destroy(i2c_dev_class);

	/* unregister device */
	unregister_chrdev(i2c_major, INPUT_DEVICE_NAME);

	/* Free the gpio pins */	
	gpio_free(GP_LED);
	gpio_free(GPIO_I2C);

	printk("I2c_flash driver removed.\n");
}

module_init(i2cflash_driver_init);
module_exit(i2cflash_driver_exit);
MODULE_LICENSE("GPL v2");
