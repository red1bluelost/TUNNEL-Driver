#include <linux/init.h>   /* Needed for the macros */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/module.h> /* Needed by all modules */

/// The license type -- this affects runtime behavior
MODULE_LICENSE("Dual MIT/GPL");
/// The author -- visible when you use modinfo
MODULE_AUTHOR("Tunnel Team");
/// The description -- see modinfo
MODULE_DESCRIPTION("Network driver for the TUNNEL Device");
/// The version of the module
MODULE_VERSION("0.0");

static int __init tunnel_init(void) {
  printk(KERN_INFO "Loading TUNNEL Network module...\n");
  printk(KERN_INFO "Hello kernel\n");
  return 0;
}

static void __exit tunnel_exit(void) { printk(KERN_INFO "Goodbye kernel.\n"); }

module_init(tunnel_init);

module_exit(tunnel_exit);
