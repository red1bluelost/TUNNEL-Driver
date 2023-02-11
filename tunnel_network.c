#include <linux/errno.h>   // Needed for error handling
#include <linux/init.h>    // Needed for the macros
#include <linux/kernel.h>  // Needed for KERN_INFO
#include <linux/kref.h>    // Needed for kref
#include <linux/module.h>  // Needed by all modules
#include <linux/mutex.h>   // Need for mutex locks
#include <linux/slab.h>    // Needed for kmalloc/kfree
#include <linux/uaccess.h> // Needed for copy to/from user
#include <linux/usb.h>     // Needed by USB driver

/// The license type -- this affects runtime behavior
MODULE_LICENSE("Dual MIT/GPL");
/// The author -- visible when you use modinfo
MODULE_AUTHOR("Tunnel Team");
/// The description -- see modinfo
MODULE_DESCRIPTION("Network driver for the TUNNEL Device");
/// The version of the module
MODULE_VERSION("0.1");

/// USB VIP/PID must match values used in Rust firmware
#define TUNNEL_USB_VID 0x0000
#define TUNNEL_USB_PID 0x6969

static struct usb_device_id tunnel_usb_table[] = {
    {USB_DEVICE(TUNNEL_USB_VID, TUNNEL_USB_PID)}, {}};
MODULE_DEVICE_TABLE(usb, tunnel_usb_table);

// Get a minor range for your devices from the usb maintainer
#define TUNNEL_USB_MINOR_BASE 192 // TODO: PROBABLY

// our private defines. if this grows any larger, use your own .h file

/// MAX_TRANSFER is chosen so that the VM is not stressed by
/// allocations > PAGE_SIZE and the number of packets in a page is an integer
/// 512 is the largest possible packet on EHCI. Arbitrarily chosen
#define MAX_TRANSFER     (PAGE_SIZE - 512)
#define WRITES_IN_FLIGHT 8

/// Structure to hold all of our device specific stuff
struct tunnel_usb {
  /// the usb device for this device
  struct usb_device* udev;
  /// the interface for this device
  struct usb_interface* interface;
  /// limiting the number of writes in progress
  struct semaphore limit_sem;
  /// in case we need to retract our submissions
  struct usb_anchor submitted;
  /// the urb to read data with
  struct urb* bulk_in_urb;
  /// the buffer to receive data
  unsigned char* bulk_in_buffer;
  /// the size of the receive buffer
  size_t bulk_in_size;
  /// number of bytes in the buffer
  size_t bulk_in_filled;
  /// already copied to user space
  size_t bulk_in_copied;
  /// the address of the bulk in endpoint
  uint8_t bulk_in_endpointAddr;
  /// the address of the bulk out endpoint
  uint8_t bulk_out_endpointAddr;
  /// the last request tanked
  int errors;
  /// a read is going on
  bool ongoing_read;
  /// lock for errors
  spinlock_t err_lock;
  /// reference count for driver
  struct kref kref;
  /// synchronize I/O with disconnect
  struct mutex  io_mutex;
  unsigned long disconnected : 1;
  /// to wait for an ongoing read
  wait_queue_head_t bulk_in_wait;
};
#define to_tunnel_usb_driver(d) container_of(d, struct tunnel_usb, kref)

// Must forward declare due to a cycle in chain of functions
static struct usb_driver tunnel_usb_driver;

static void tunnel_usb_draw_down(struct tunnel_usb* dev) {
  int time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
  if (!time) usb_kill_anchored_urbs(&dev->submitted);
  usb_kill_urb(dev->bulk_in_urb);
}

static void tunnel_usb_delete(struct kref* kref) {
  struct tunnel_usb* dev = to_tunnel_usb_driver(kref);

  usb_free_urb(dev->bulk_in_urb);
  usb_put_intf(dev->interface);
  usb_put_dev(dev->udev);
  kfree(dev->bulk_in_buffer);
  kfree(dev);
}

static int tunnel_usb_open(struct inode* inode, struct file* file) {
  int subminor = (int)iminor(inode);

  struct usb_interface* interface =
      usb_find_interface(&tunnel_usb_driver, subminor);
  if (!interface) {
    pr_err("%s - error, can't find device for minor %d\n", __func__, subminor);
    return -ENODEV;
  }

  struct tunnel_usb* dev = usb_get_intfdata(interface);
  if (!dev) return -ENODEV;

  int retval = usb_autopm_get_interface(interface);
  if (retval) return retval;

  // increment our usage count for the device
  kref_get(&dev->kref);

  // save our object in the file's private structure
  file->private_data = dev;

  return 0;
}

static int tunnel_release(struct inode* inode, struct file* file) {
  struct tunnel_usb* dev = file->private_data;
  if (dev == NULL) return -ENODEV;

  // allow the device to be autosuspended
  usb_autopm_put_interface(dev->interface);

  // decrement the count on our device
  kref_put(&dev->kref, tunnel_usb_delete);
  return 0;
}

static int tunnel_flush(struct file* file, fl_owner_t id) {
  struct tunnel_usb* dev = file->private_data;
  if (dev == NULL) return -ENODEV;

  // wait for io to stop
  mutex_lock(&dev->io_mutex);
  tunnel_usb_draw_down(dev);

  // read out errors, leave subsequent opens a clean slate
  spin_lock_irq(&dev->err_lock);
  int res     = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
  dev->errors = 0;
  spin_unlock_irq(&dev->err_lock);

  mutex_unlock(&dev->io_mutex);

  return res;
}

static void tunnel_read_bulk_callback(struct urb* urb) {
  struct tunnel_usb* dev = urb->context;

  unsigned long flags;
  spin_lock_irqsave(&dev->err_lock, flags);
  // sync/async unlink faults aren't errors
  if (urb->status) {
    if (!(urb->status == -ENOENT || urb->status == -ECONNRESET ||
          urb->status == -ESHUTDOWN)) {
      dev_err(
          &dev->interface->dev,
          "%s - nonzero write bulk status received: %d\n",
          __func__,
          urb->status
      );
    }

    dev->errors = urb->status;
  } else {
    dev->bulk_in_filled = urb->actual_length;
  }
  dev->ongoing_read = 0;
  spin_unlock_irqrestore(&dev->err_lock, flags);

  wake_up_interruptible(&dev->bulk_in_wait);
}

static int tunnel_usb_do_read_io(struct tunnel_usb* dev, size_t count) {
  // prepare a read
  usb_fill_bulk_urb(
      dev->bulk_in_urb,
      dev->udev,
      usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
      dev->bulk_in_buffer,
      (int)min(dev->bulk_in_size, count),
      tunnel_read_bulk_callback,
      dev
  );
  // tell everybody to leave the URB alone
  spin_lock_irq(&dev->err_lock);
  dev->ongoing_read = 1;
  spin_unlock_irq(&dev->err_lock);

  // submit bulk in urb, which means no data to deliver
  dev->bulk_in_filled = 0;
  dev->bulk_in_copied = 0;

  // do it
  int rv = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
  if (rv < 0) {
    dev_err(
        &dev->interface->dev,
        "%s - failed submitting read urb, error %d\n",
        __func__,
        rv
    );
    rv = (rv == -ENOMEM) ? rv : -EIO;
    spin_lock_irq(&dev->err_lock);
    dev->ongoing_read = 0;
    spin_unlock_irq(&dev->err_lock);
  }

  return rv;
}

static ssize_t
tunnel_usb_read(struct file* file, char* buffer, size_t count, loff_t* ppos) {
  struct tunnel_usb* dev = file->private_data;

  if (!count) return 0;

  // no concurrent readers
  int rv = mutex_lock_interruptible(&dev->io_mutex);
  if (rv < 0) return rv;

// From here out use macro return since mutex must be unlocked
#define UNLOCK_MUTEX(_rv_)                                                     \
  ({                                                                           \
    mutex_unlock(&dev->io_mutex);                                              \
    (_rv_);                                                                    \
  })

  // check if disconnect was called
  if (dev->disconnected) return UNLOCK_MUTEX(-ENODEV);

  bool ongoing_io;
  // if IO is under way, we must not touch things
  while (1) {
    spin_lock_irq(&dev->err_lock);
    ongoing_io = dev->ongoing_read;
    spin_unlock_irq(&dev->err_lock);

    if (ongoing_io) {
      // nonblocking IO shall not wait
      if (file->f_flags & O_NONBLOCK) return UNLOCK_MUTEX(-EAGAIN);

      // IO may take forever hence wait in an interruptible state
      rv = wait_event_interruptible(dev->bulk_in_wait, (!dev->ongoing_read));
      if (rv < 0) return UNLOCK_MUTEX(rv);
    }

    // errors must be reported
    rv = dev->errors;
    if (rv < 0) {
      // any error is reported once
      dev->errors = 0;
      // to preserve notifications about reset
      if (rv != -EPIPE) rv = -EIO;
      // report it
      return UNLOCK_MUTEX(rv);
    }

    // if the buffer is filled we may satisfy the read else we need to start IO
    if (dev->bulk_in_filled) {
      // we had read data
      size_t available = dev->bulk_in_filled - dev->bulk_in_copied;
      size_t chunk     = min(available, count);

      if (!available) {
        // all data has been used, actual IO needs to be done
        rv = tunnel_usb_do_read_io(dev, count);
        if (rv < 0) return UNLOCK_MUTEX(rv);
        continue;
      }

      // data is available, chunk tells us how much shall be copied
      if (copy_to_user(
              buffer, dev->bulk_in_buffer + dev->bulk_in_copied, chunk
          ))
        rv = -EFAULT;
      else rv = chunk;

      dev->bulk_in_copied += chunk;

      // if we are asked for more than we have, we start IO but don't wait
      if (available < count) tunnel_usb_do_read_io(dev, count - chunk);
      return UNLOCK_MUTEX(rv);
    }

    // no data in the buffer
    rv = tunnel_usb_do_read_io(dev, count);
    if (rv < 0) return UNLOCK_MUTEX(rv);
  }
#undef UNLOCK_MUTEX
}

static void tunnel_usb_write_bulk_callback(struct urb* urb) {
  struct tunnel_usb* dev = urb->context;

  // sync/async unlink faults aren't errors
  if (urb->status) {
    if (!(urb->status == -ENOENT || urb->status == -ECONNRESET ||
          urb->status == -ESHUTDOWN)) {
      dev_err(
          &dev->interface->dev,
          "%s - nonzero write bulk status received: %d\n",
          __func__,
          urb->status
      );
    }

    unsigned long flags;
    spin_lock_irqsave(&dev->err_lock, flags);
    dev->errors = urb->status;
    spin_unlock_irqrestore(&dev->err_lock, flags);
  }

  // free up our allocated buffer
  usb_free_coherent(
      urb->dev,
      urb->transfer_buffer_length,
      urb->transfer_buffer,
      urb->transfer_dma
  );
  up(&dev->limit_sem);
}

static ssize_t tunnel_usb_write(
    struct file* file, const char* user_buffer, size_t count, loff_t* ppos
) {
  const size_t       writesize = min_t(size_t, count, MAX_TRANSFER);
  struct tunnel_usb* dev       = file->private_data;

  // verify that we actually have some data to write
  if (count == 0) return 0;

  // limit the number of URBs in flight to stop a user from using up all RAM
  if (!(file->f_flags & O_NONBLOCK) && down_interruptible(&dev->limit_sem))
    return -ERESTARTSYS;

  if (down_trylock(&dev->limit_sem)) return -EAGAIN;

    // From here out use macros for return since locks are in place
#define ERROR_DEC_SEM(_rv_)                                                    \
  ({                                                                           \
    up(&dev->limit_sem);                                                       \
    (_rv_);                                                                    \
  })

  spin_lock_irq(&dev->err_lock);
  int retval = dev->errors;
  if (retval < 0) {
    // any error is reported once
    dev->errors = 0;
    // to preserve notifications about reset
    retval = (retval == -EPIPE) ? retval : -EIO;
  }
  spin_unlock_irq(&dev->err_lock);
  if (retval < 0) return ERROR_DEC_SEM(retval);

  // create urb, and buffer for it, and copy the data to the urb
  struct urb* urb = usb_alloc_urb(0, GFP_KERNEL);
  if (!urb) return ERROR_DEC_SEM(-ENOMEM);

#define ERROR_FREE_URB_DEC_SEM(_rv_, _urb_, _buf_)                             \
  ({                                                                           \
    usb_free_coherent(dev->udev, writesize, (_buf_), (_urb_)->transfer_dma);   \
    usb_free_urb(_urb_);                                                       \
    ERROR_DEC_SEM(_rv_);                                                       \
  })

  char* buf =
      usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
  if (!buf) return ERROR_FREE_URB_DEC_SEM(-ENOMEM, urb, buf);

  if (copy_from_user(buf, user_buffer, writesize))
    return ERROR_FREE_URB_DEC_SEM(-EFAULT, urb, buf);

  // this lock makes sure we don't submit URBs to gone devices
  mutex_lock(&dev->io_mutex);
  if (dev->disconnected) { // disconnect() was called
    mutex_unlock(&dev->io_mutex);
    return ERROR_FREE_URB_DEC_SEM(-ENODEV, urb, buf);
  }

  // initialize the urb properly
  usb_fill_bulk_urb(
      urb,
      dev->udev,
      usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
      buf,
      (int)writesize,
      tunnel_usb_write_bulk_callback,
      dev
  );
  urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
  usb_anchor_urb(urb, &dev->submitted);

  // send the data out the bulk port
  retval = usb_submit_urb(urb, GFP_KERNEL);
  mutex_unlock(&dev->io_mutex);
  if (retval) {
    dev_err(
        &dev->interface->dev,
        "%s - failed submitting write urb, error %d\n",
        __func__,
        retval
    );
    usb_unanchor_urb(urb);
    return ERROR_FREE_URB_DEC_SEM(retval, urb, buf);
  }

  // release our reference to this urb, the USB core will eventually free it
  // entirely
  usb_free_urb(urb);

  return (ssize_t)writesize;
#undef ERROR_FREE_URB_DEC_SEM
#undef ERROR_DEC_SEM
}

static const struct file_operations tunnel_usb_fops = {
    .owner   = THIS_MODULE,
    .read    = tunnel_usb_read,
    .write   = tunnel_usb_write,
    .open    = tunnel_usb_open,
    .release = tunnel_release,
    .flush   = tunnel_flush,
    .llseek  = noop_llseek,
};

// usb class driver info in order to get a minor number from the usb core, and
// to have the device registered with the driver core
static struct usb_class_driver tunnel_usb_class = {
    .name       = "tunnel%d",
    .fops       = &tunnel_usb_fops,
    .minor_base = TUNNEL_USB_MINOR_BASE,
};

static int tunnel_usb_probe(
    struct usb_interface* const interface, const struct usb_device_id* const id
) {
  dev_info(
      &interface->dev,
      "USB Skeleton device probing for USBSkel-%d",
      interface->minor
  );

  // allocate memory for our device state and initialize it
  struct tunnel_usb* dev = kzalloc(sizeof(*dev), GFP_KERNEL);
  if (!dev) return -ENOMEM;

  kref_init(&dev->kref);
  sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
  mutex_init(&dev->io_mutex);
  spin_lock_init(&dev->err_lock);
  init_usb_anchor(&dev->submitted);
  init_waitqueue_head(&dev->bulk_in_wait);

  dev->udev      = usb_get_dev(interface_to_usbdev(interface));
  dev->interface = usb_get_intf(interface);

  // On failures use to decrement the reference count
#define ERROR_DEC_REF(_rv_)                                                    \
  ({                                                                           \
    kref_put(&dev->kref, tunnel_usb_delete);                                   \
    (_rv_);                                                                    \
  })

  struct usb_endpoint_descriptor *bulk_in, *bulk_out;
  // set up the endpoint information
  // use only the first bulk-in and bulk-out endpoints
  int retval = usb_find_common_endpoints(
      interface->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL
  );
  if (retval) {
    dev_err(
        &interface->dev, "Could not find both bulk-in and bulk-out endpoints\n"
    );
    return ERROR_DEC_REF(retval);
  }

  dev->bulk_in_size         = (size_t)usb_endpoint_maxp(bulk_in);
  dev->bulk_in_endpointAddr = bulk_in->bEndpointAddress;

  dev->bulk_in_buffer = kmalloc(dev->bulk_in_size, GFP_KERNEL);
  if (!dev->bulk_in_buffer) return ERROR_DEC_REF(-ENOMEM);

  dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
  if (!dev->bulk_in_urb) return ERROR_DEC_REF(-ENOMEM);

  dev->bulk_out_endpointAddr = bulk_out->bEndpointAddress;

  // save our data pointer in this interface device
  usb_set_intfdata(interface, dev);

  // we can register the device now, as it is ready
  retval = usb_register_dev(interface, &tunnel_usb_class);
  if (retval) {
    // something prevented us from registering this driver
    dev_err(&interface->dev, "Not able to get a minor for this device.\n");
    usb_set_intfdata(interface, NULL);
    return ERROR_DEC_REF(retval);
  }

  // let the user know what node this device is now attached to
  dev_info(
      &interface->dev,
      "USB Skeleton device now attached to USBSkel-%d",
      interface->minor
  );
  return 0;
#undef ERROR_DEC_REF
}

static void tunnel_usb_disconnect(struct usb_interface* interface) {
  int minor = interface->minor;

  struct tunnel_usb* dev = usb_get_intfdata(interface);

  // give back our minor
  usb_deregister_dev(interface, &tunnel_usb_class);

  // prevent more I/O from starting
  mutex_lock(&dev->io_mutex);
  dev->disconnected = 1;
  mutex_unlock(&dev->io_mutex);

  usb_kill_urb(dev->bulk_in_urb);
  usb_kill_anchored_urbs(&dev->submitted);

  // decrement our usage count
  kref_put(&dev->kref, tunnel_usb_delete);

  dev_info(&interface->dev, "USB Skeleton #%d now disconnected", minor);
}

static int tunnel_usb_suspend(struct usb_interface* intf, pm_message_t msg) {
  struct tunnel_usb* dev = usb_get_intfdata(intf);
  if (dev) tunnel_usb_draw_down(dev);
  return 0;
}

static int tunnel_usb_resume(struct usb_interface* intf) { return 0; }

static int tunnel_usb_pre_reset(struct usb_interface* intf) {
  struct tunnel_usb* dev = usb_get_intfdata(intf);
  mutex_lock(&dev->io_mutex);
  tunnel_usb_draw_down(dev);
  return 0;
}

static int tunnel_usb_post_reset(struct usb_interface* intf) {
  struct tunnel_usb* dev = usb_get_intfdata(intf);
  // we are sure no URBs are active - no locking needed
  dev->errors = -EPIPE;
  mutex_unlock(&dev->io_mutex);
  return 0;
}

static struct usb_driver tunnel_usb_driver = {
    .name                 = "tunnel_usb_driver",
    .probe                = tunnel_usb_probe,
    .disconnect           = tunnel_usb_disconnect,
    .suspend              = tunnel_usb_suspend,
    .resume               = tunnel_usb_resume,
    .pre_reset            = tunnel_usb_pre_reset,
    .post_reset           = tunnel_usb_post_reset,
    .id_table             = tunnel_usb_table,
    .supports_autosuspend = 1,
};

static int __init tunnel_init(void) {
  pr_info("Loading TUNNEL Network module...\n");

  int result = usb_register(&tunnel_usb_driver);
  if (result < 0) {
    pr_err(
        "usb_register failed for %s. Error number %d\n",
        tunnel_usb_driver.name,
        result
    );
    return -1;
  }

  return 0;
}

static void __exit tunnel_exit(void) {
  usb_deregister(&tunnel_usb_driver);
  pr_info("...Tunnel Network module unloaded.\n");
}

module_init(tunnel_init);
module_exit(tunnel_exit);