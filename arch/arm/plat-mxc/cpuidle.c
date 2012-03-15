/*
 * arch/arm/plat-mxc/cpuidle.c
 * Copyright (C) 2011 Amazon.com, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@amazon.com)
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/cpuidle.h>
#include <mach/cpuidle.h>
#include <linux/platform_device.h>

static struct cpuidle_driver imx_cpuidle_driver = {
       .name =         "imx_idle",
       .owner =        THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device, imx_cpuidle_device);

static int imx_cpuidle_register(struct imx_cpuidle_state *pstate)
{
       struct cpuidle_device *device;
       struct cpuidle_state *state = pstate->state;
       int i;

       cpuidle_register_driver(&imx_cpuidle_driver);

       device = &per_cpu(imx_cpuidle_device, smp_processor_id());
       device->state_count = pstate->state_number;

       for (i = 0; i < device->state_count; i++) {
               device->states[i].enter = state[i].enter;
               device->states[i].exit_latency = state[i].exit_latency;
               device->states[i].target_residency = state[i].target_residency;
               device->states[i].flags = state[i].flags;
               strcpy(device->states[i].name, state[i].name);
               strcpy(device->states[i].desc, state[i].desc);
       }

       if (cpuidle_register_device(device)) {
               printk(KERN_ERR "imx_cpuidle_register: Failed registering\n");
               return -EIO;
       }
       return 0;
}

static int __devinit imx_cpuidle_probe(struct platform_device *pdev)
{
       struct imx_cpuidle_state *state = pdev->dev.platform_data;

       return imx_cpuidle_register(state);
}

static struct platform_driver imx_cpuidle_platform_driver = {
       .driver         = {
               .name   = "imx_cpuidle",
       },
       .probe          = imx_cpuidle_probe,
};

static int __init imx_cpuidle_init(void)
{
       return platform_driver_register(&imx_cpuidle_platform_driver);
}
arch_initcall(imx_cpuidle_init);

static void __exit imx_cpuidle_exit(void)
{
       platform_driver_unregister(&imx_cpuidle_platform_driver);
}
module_exit(imx_cpuidle_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Manish Lachwani");
