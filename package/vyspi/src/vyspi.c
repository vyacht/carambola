/*
 * vYacht SPI device driver
 *
 * Copyright (C) 2013 Bernd Ocklin <bernd@vyacht.net>
 *
 * This work is derived from spidev with a lot of help from looking at the 
 * improved mcp2515 driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>

#include <linux/spi/spi.h>

#include <asm/uaccess.h>

#include "vyspi.h"
#include <linux/delay.h>

/*
 * This supports access to vYacht SPI devices.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/vyspiB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define VYSPI_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

#define SPI_BUS_SPEED 1000000

static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct vyspi_data {
	wait_queue_head_t       inq;
	wait_queue_head_t       outq;  /* read and write queues */
	dev_t			devt;
	spinlock_t		spi_lock;
	int			has_data;  // spi device signaled data via interrupt
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;

	/* Message, transfer and buffers for one async spi transaction */
	struct spi_message 	message;
	struct spi_transfer 	transfer;

        u8 			*rx_buf;
        u8 			*tx_buf;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 256;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "Data bytes in biggest supported vyacht SPI message");

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void vyspi_complete(void *arg)
{
	complete(arg);
}

static ssize_t
vyspi_sync(struct vyspi_data *spidev)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	spidev->message.complete = vyspi_complete;
	spidev->message.context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, &spidev->message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = spidev->message.status;
		if (status == 0) {
			status = spidev->message.actual_length;
		}
	}
	return status;
}

static inline ssize_t
vyspi_sync_write(struct vyspi_data *spidev, size_t len)
{
	spidev->transfer.len = len;
	return vyspi_sync(spidev);
}

static inline ssize_t
vyspi_sync_read(struct vyspi_data *spidev, size_t len)
{
	memset(spidev->tx_buf, 0, bufsiz);
	spidev->transfer.len = len;
	return vyspi_sync(spidev);
}

/*-------------------------------------------------------------------------*/

static void 
vyspi_requestHeader(struct vyspi_data *spidev) 
{
        u8 *buf = (u8 *)spidev->tx_buf;
	memset(buf, 0, bufsiz);
        buf[0] = VYSPI_READ | VYSPI_HEADER;
	vyspi_sync_write(spidev, 1);
}

static void 
vyspi_requestData(struct vyspi_data *spidev) 
{
        u8 *buf = (u8 *)spidev->tx_buf;
	memset(buf, 0, bufsiz);
        buf[0] = VYSPI_READ | VYSPI_DATA;
	vyspi_sync_write(spidev, 1);
}

static void 
vyspi_requestReset(struct vyspi_data *spidev) 
{
        u8 *buf = (u8 *)spidev->tx_buf;
	memset(buf, 0, 255);
        buf[0] = VYSPI_RESET;
	vyspi_sync_write(spidev, 1);
}

/*
 * function to read a complete packet from stm32
 * returns 0 only when an empty packet has been read
 */
static ssize_t 
vyspi_read_packet(struct vyspi_data *spidev, char __user * ubuf, size_t len) 
{
	ssize_t			status = 0;
        u8 			*rx_buf = NULL;
        uint16_t 		packet_len;
        uint16_t 		l;
	const uint8_t		headerlen = 1; 

        vyspi_requestHeader(spidev);
	vyspi_sync_read(spidev, headerlen);

        rx_buf = (uint8_t *)spidev->transfer.rx_buf;
        packet_len = *(uint8_t *)&rx_buf[0];

	//        printk("len= %u\n", packet_len);
	
	if(packet_len <= 0) {
        	printk("Exit with len= %u\n", packet_len);
		goto exit;
	}

        vyspi_requestData(spidev);
	vyspi_sync_read(spidev, packet_len);

	l = packet_len;
	if(len < packet_len)
		l = len;
	status = l; 

	if ((l>0) && copy_to_user(ubuf, rx_buf, l)) {
		/* There must have been an addressing problem */
		status = -EFAULT;
		goto exit;
	}

exit:
	return status;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup 
 *
 * Message will be discarded even if only partially 
 * fetched.  
 * 
 * return value is min(total number of bytes of the packet, buffer len)
 */
static ssize_t
vyspi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct vyspi_data	*spidev;
	ssize_t			status = 0;
	size_t			cnt = 0;

	// printk("reading: start\n");

	cnt = count;
	if(count > bufsiz)
		cnt = bufsiz;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);

	// we are blocking until data is available
        while (!spidev->has_data) { 
		// nothing to read
		// unlock to not stay blocking and potentially deadlock 
		mutex_unlock(&spidev->buf_lock);
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		printk("reading: going to sleep\n");
		if (wait_event_interruptible(spidev->inq, spidev->has_data))
		  return -ERESTARTSYS; // signal: tell the fs layer to handle it 
		// otherwise loop, but first reacquire the lock 
		mutex_lock(&spidev->buf_lock);
	}

	////  printk("reading: got data\n");

	if(spidev->has_data) {
		spidev->has_data = 0;
		status = vyspi_read_packet(spidev, buf, cnt);
	}

	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
vyspi_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct vyspi_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	missing = copy_from_user(spidev->tx_buf, buf, count);
	if (missing == 0) {
		status = vyspi_sync_write(spidev, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static long
vyspi_param(struct vyspi_data * spidev, struct vyspi_ioc_cmd_t * ioc) {

  // no locking here on the buffer as this has been done by ioctl

  u8 *buf = (u8 *)spidev->tx_buf;
  memset(buf, 0, bufsiz);

  buf[0] = VYSPI_WRITE | VYSPI_PARAM;
  buf[1] = ioc->type;

  // data was defined inline
  memcpy((uint8_t *)&buf[2], ioc->data, VYSPI_IOC_CMD_DATA_SIZE);

  vyspi_sync_write(spidev, 1 + 1 + VYSPI_IOC_CMD_DATA_SIZE);
}

static long
vyspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct vyspi_data	*spidev;
	struct spi_device	*spi;
        struct vyspi_ioc_cmd_t  *ioc;
	unsigned long           tmp;

	/* Check access direction once here; don't repeat below.
	 * RESET is a write command so we need to check for READ
	 * from a kernel perspective.
	 */
	err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case VYSPI_RESET:
		vyspi_requestReset(spidev);
		break;

	case VYSPI_PARAM:

	  printk("vyspi_ioctl() param case\n");

	  tmp = sizeof(struct vyspi_ioc_cmd_t);

	  /* copy into scratch area */
	  ioc = kmalloc(tmp, GFP_KERNEL);
	  if (!ioc) {
	    retval = -ENOMEM;
	    break;
	  }
	  if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
	    kfree(ioc);
	    retval = -EFAULT;
	    break;
	  }

	  /* translate to spi_message, execute */
	  retval = vyspi_param(spidev, ioc);
	  kfree(ioc);
	  break;
  
	default:
		retval = -ENOTTY;
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

unsigned int vyspi_poll(struct file *filp, poll_table *wait)
{
	struct vyspi_data *dev = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &dev->inq, wait);
	// poll_wait(filp, &dev->outq, wait);

	// printk("vyspi_poll has data: %d\n", dev->has_data);

	// TODO: locking

	if (dev->has_data)
		mask |= POLLIN | POLLRDNORM;	/* readable */

	// mask |= POLLOUT | POLLWRNORM;	/* writable */

	return mask;
}

#ifdef CONFIG_COMPAT
static long
vyspi_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return vyspi_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define vyspi_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

/*
 * Interrupt handler.
 * We are not fetching any data here. Just set flags that 
 * there is data available.  
 */
static irqreturn_t vyspi_interrupt(int irq, void *dev_id)
{

	struct vyspi_data	*dev;
	dev = (struct vyspi_data *)dev_id;
	
	// TODO: locking
	dev->has_data = 1;

	// printk("vyspi_interrupt irq %d\n", irq);

	// signal read wait queue
	wake_up_interruptible(&dev->inq);

        return IRQ_HANDLED;
}

static int vyspi_open(struct inode *inode, struct file *filp)
{
	struct vyspi_data	*spidev;
	int			status = -ENXIO;
	struct spi_device * spi        = NULL;

	printk("open device request\n");
	
	//	msleep(1000-1);
	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	printk("open device with status %d and %d users\n", status, spidev->users);
	if (status == 0) {
		spi = spidev->spi;

		if (!spidev->users) {
			/* set the has data indicator to no-data before 
			   asking for interrupt handling 
			*/
			spidev->has_data = 0;

			status = request_irq(spi->irq, vyspi_interrupt,
				  IRQF_TRIGGER_FALLING, "vyspi", spidev);
			printk("open/request irq %d (%d)\n", spi->irq, status);
		}

		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
		
		if(status) {
		  free_irq(spi->irq, spidev);
		  printk("freeing irq %d (%d)\n", spi->irq, status);
		}
	} else
		pr_debug("vyspi: nothing for minor %d\n", iminor(inode));


	mutex_unlock(&device_list_lock);
	printk("vyspi: open finished\n");
	return status;
}

static int vyspi_release(struct inode *inode, struct file *filp)
{
	struct vyspi_data	*spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);

	        free_irq(spidev->spi->irq, spidev);
		printk("freeing irq on release %d\n", spidev->spi->irq);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations vyspi_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	vyspi_write,
	.read =		vyspi_read,
	.unlocked_ioctl = vyspi_ioctl,
	.compat_ioctl = vyspi_compat_ioctl,
	.open =		vyspi_open,
	.poll =         vyspi_poll,
	.release =	vyspi_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/vyspiB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *vyspi_class;

/*-------------------------------------------------------------------------*/

/*
 * Set up SPI messages.
 */
static int vyspi_setup_spi_messages(struct vyspi_data *spidev)
{
	int			status = 0;
	u8 			* buf = NULL;

	spi_message_init(&spidev->message);

	if (!spidev->rx_buf && !spidev->tx_buf) {
		spidev->rx_buf = kmalloc(bufsiz, GFP_KERNEL);
		if(spidev->rx_buf) 
			spidev->tx_buf = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->rx_buf || !spidev->tx_buf) {
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
		}
	}

	if(status == 0) {

		spidev->transfer.tx_buf = spidev->tx_buf;
		spidev->transfer.rx_buf = spidev->rx_buf;

		spi_message_add_tail(&spidev->transfer, &spidev->message);
	}

	return status;
}

static void vyspi_remove_spi_messages(struct vyspi_data *spidev)
{
	// tx points to entire buffer
	kfree(spidev->tx_buf);
	kfree(spidev->rx_buf);
	spidev->rx_buf = NULL;
	spidev->tx_buf = NULL;
}

static int __devinit vyspi_probe(struct spi_device *spi)
{
	struct vyspi_data	*spidev;
	int			status;
	unsigned long		minor;

	printk("vyspi_probe() enter\n");

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	status = vyspi_setup_spi_messages(spidev);

	if (status) {
		printk("vyspi_setup_spi_messages failed with status %d\n", status);
		kfree(spidev);
		return status;
	}

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	init_waitqueue_head(&spidev->inq);
	//init_waitqueue_head(&spidev->outq);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	printk("vyspi_probe() starting to setup device\n");
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(VYSPI_MAJOR, minor);
		dev = device_create(vyspi_class, &spi->dev, spidev->devt,
				    spidev, "vyspi%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		printk("vyspi_probe() device created\n");
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);

	printk("vyspi_probe() exit with status %d\n", status);
	return status;
}

static int __devexit vyspi_remove(struct spi_device *spi)
{
	struct vyspi_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(vyspi_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	vyspi_remove_spi_messages(spidev);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver vyspi_spi_driver = {
	.driver = {
		.name =		"vyspi",
		.owner =	THIS_MODULE,
	},
	.probe =	vyspi_probe,
	.remove =	__devexit_p(vyspi_remove),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/
static int __init vyspi_add_device_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	struct device *pdev;
	char buff[64];
	int status = 0;

	spi_master = spi_busnum_to_master(0);
	if (!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n", 0);
		return -1;
	}

	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		printk(KERN_ALERT "spi_alloc_device() failed\n");
		return -1;
	}

	spi_device->chip_select = 0;

	/* Check whether this SPI bus.cs is already claimed */
	snprintf(buff, sizeof(buff), "%s.%u",
		dev_name(&spi_device->master->dev),
		spi_device->chip_select);

	pdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buff);
	if (pdev) {
		printk("spi_add_device() device found by name %s with status %d\n", 
			buff, status);
		/* We are not going to use this spi_device, so free it */
		spi_dev_put(spi_device);

		/*
		* There is already a device configured for this bus.cs
		* It is okay if it us, otherwise complain and fail.
		*/
		if (pdev->driver && pdev->driver->name &&
			strcmp("vyspi", pdev->driver->name)) {

			printk(KERN_ALERT
				"Driver [%s] already registered for %s\n",
				pdev->driver->name, buff);
			status = -1;
		}
		if (pdev->driver && pdev->driver->name)
			printk("spi_add_device() device %s found by name %s with status %d\n", 
				pdev->driver->name, buff, status);
	} else {
		spi_device->max_speed_hz = SPI_BUS_SPEED;
		spi_device->mode = SPI_MODE_0;
		spi_device->bits_per_word = 8;
		spi_device->irq = 54;
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, "vyspi", SPI_NAME_SIZE);

		status = spi_add_device(spi_device);	
		if (status < 0) {	
			spi_dev_put(spi_device);
			printk(KERN_ALERT "spi_add_device() failed: %d\n",
			status);	
		}	
		printk("spi_add_device() status %d\n", status);
	}

	put_device(&spi_master->dev);

	return status;
}

static int __init vyspi_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	printk("vyspi_init() start\n");
	status = register_chrdev(VYSPI_MAJOR, "spi", &vyspi_fops);
	if (status < 0)
		return status;

	printk("vyspi_init() create class\n");
	vyspi_class = class_create(THIS_MODULE, "vyspi");
	if (IS_ERR(vyspi_class)) {
		unregister_chrdev(VYSPI_MAJOR, vyspi_spi_driver.driver.name);
		return PTR_ERR(vyspi_class);
	}

	status = spi_register_driver(&vyspi_spi_driver);
	if (status < 0) {
		class_destroy(vyspi_class);
		unregister_chrdev(VYSPI_MAJOR, vyspi_spi_driver.driver.name);
	}

	printk("vyspi_init() register driver %d\n", status);
	/*

	if(status == 0) {
		status = vyspi_add_device_to_bus();
	}
	printk("vyspi_init() add device to bus %d\n", status);
	*/

	return status;
}
module_init(vyspi_init);

static void __exit vyspi_exit(void)
{
	printk("vyspi_exit()\n");
	spi_unregister_driver(&vyspi_spi_driver);
	class_destroy(vyspi_class);
	unregister_chrdev(VYSPI_MAJOR, vyspi_spi_driver.driver.name);
}
module_exit(vyspi_exit);

MODULE_AUTHOR("Bernd Ocklin, <bernd@vyacht.net>");
MODULE_DESCRIPTION("User mode vYacht SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:vyspi");
