#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/hardirq.h>
#include <linux/topology.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/msr-index.h>
#include <asm/msr.h>
#include <asm/cpufeature.h> //cpu_has
#include <asm/processor.h> //boot_cpu_data

#define PMC_IA32_PERF_FIXED_CTR1        0x40000001
#define PMC_IA32_PERF_FIXED_CTR2        0x40000002

#define IDA_DISENGAGE(bit)              (((bit) & 1) << 32)
#define IDA_DISENGAGE_MASK              (1ULL << 32)

#define THERM_STATUS_TEMP(bits)         (((bits) >> 16) & 0x1111111)
#define THERM_STATUS_RESOLUTION(bits)   (((bits) >> 27) & 0x1111)
#define THERM_STATUS_VALID(bits)        (((bits) >> 31) & 0x1)

extern struct platform_device *coretemp_get_pdev(unsigned int cpu);

struct cpu_data {
        u64 aperf;
        u64 mperf;
        u8 perf_bias;
        u64 ucc;
        u64 urc;
        u16 perf_target;
        u16 perf_status;
        u8 turbo_disengage;
     	u64 tsc;
/*      u16 lowest_perf;
        u16 guaranteed_perf; // dynamic
        u16 mostefficient_perf; // dynamic
        u16 highest_perf;*/
};

static struct cpu_data *cpu_data_buffer = NULL;
static u8 hwp = 0;

static int gather_cpu_data(struct cpu_data *d, int cpu)
{
        u64 perf_bias;
        u64 perf_ctl;
        u64 perf_status;
        unsigned long flags;

        local_irq_save(flags);
        rdmsrl_on_cpu(cpu, MSR_IA32_APERF, &(d->aperf)); // thread for snb
        rdmsrl_on_cpu(cpu, MSR_IA32_MPERF, &(d->mperf)); // thread for snb
        // package for snb, thread for phi, not in hsw
        rdmsrl_on_cpu(cpu, MSR_IA32_ENERGY_PERF_BIAS, &perf_bias);
        rdmsrl_on_cpu(cpu, MSR_CORE_PERF_FIXED_CTR1, &(d->ucc)); // thread for snb
        rdmsrl_on_cpu(cpu, MSR_CORE_PERF_FIXED_CTR2, &(d->urc)); // thread for snb
        //rdpmcl(PMC_IA32_PERF_FIXED_CTR1, d->ucc);
        //rdpmcl(PMC_IA32_PERF_FIXED_CTR2, d->urc);
        rdmsrl_on_cpu(cpu, MSR_IA32_PERF_CTL, &perf_ctl); // thread for snb
        rdmsrl_on_cpu(cpu, MSR_IA32_PERF_STATUS, &perf_status); // package for snb
        local_irq_restore(flags);
        rdtscll(d->tsc);
        d->perf_bias = perf_bias & 0xf;
        d->perf_target = perf_ctl & 0xffff;
        d->turbo_disengage = (perf_ctl & (1UL << 32));
        d->perf_status = perf_status & 0xffff;
        return 0;
}

static ssize_t cpustat_show(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        u32 cpu;
        struct cpu_data *d;

        cpu = dev->id;
        d = (struct cpu_data*)cpu_data_buffer + cpu;
        gather_cpu_data(d, cpu);
        return scnprintf(buf, PAGE_SIZE,
                         "id %20d\n"
                         "aperf %17llu\n"
                         "mperf %17llu\n"
                         "perf_bias %13u\n"
                         "ucc %19llu\n"
                         "urc %19llu\n"
                         "perf_target %11u\n"
                         "perf_status %11u\n"
                         "pstate %16d\n"
                         "turbo_disengage %7u\n"
                         "tsc %19llu\n"
                         ,cpu, d->aperf, d->mperf, d->perf_bias,
                         d->ucc, d->urc, d->perf_target, d->perf_status,
                         ((d->perf_status >> 8) & 0xff), d->turbo_disengage, d->tsc);
}

static ssize_t perf_bias_show(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        u32 cpu;
        struct cpu_data *d;

        cpu = dev->id;
        d = (struct cpu_data*)cpu_data_buffer + cpu;
        gather_cpu_data(d, cpu);
        return scnprintf(buf, PAGE_SIZE, "%u\n", d->perf_bias);
}

static ssize_t perf_bias_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
        unsigned int input;
        int ret;
        u64 val;

        ret = sscanf(buf, "%u", &input);
        if (ret != 1)
                return -EINVAL;

        val = clamp_t(int, input, 0, 0xf);
        wrmsrl_on_cpu(dev->id, MSR_IA32_ENERGY_PERF_BIAS, val);
        return count;
}

static ssize_t hwp_show(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        return scnprintf(buf, PAGE_SIZE, "%u\n", hwp);
}

static ssize_t turbo_show(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        u64 misc_enabled;

        rdmsrl(MSR_IA32_MISC_ENABLE, misc_enabled);
        return scnprintf(buf, PAGE_SIZE, "%u\n",
                         (misc_enabled & MSR_IA32_MISC_ENABLE_TURBO_DISABLE) ?
                         0 : 1);
}

static ssize_t turbo_disengage_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
        u64 perf_ctl;
        u8 turbo_disengage;

        rdmsrl_on_cpu(dev->id, MSR_IA32_PERF_CTL, &perf_ctl); // thread for snb
        turbo_disengage = (perf_ctl & IDA_DISENGAGE_MASK);
        return scnprintf(buf, PAGE_SIZE, "%u\n", turbo_disengage);
}

static ssize_t turbo_disengage_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
        unsigned int input;
        int ret;
        u64 disengage;
        u64 val;

        ret = sscanf(buf, "%u", &input);
        if (ret != 1)
                return -EINVAL;

        disengage = clamp_t(int, input, 0, 1);
        rdmsrl_on_cpu(dev->id, MSR_IA32_PERF_CTL, &val); // thread for snb
        val &= ~(val & IDA_DISENGAGE_MASK);
        val |= IDA_DISENGAGE(disengage);
        printk("writing %llx\n", val);
        wrmsrl_on_cpu(dev->id, MSR_IA32_PERF_CTL, val);
        return count;
}

static bool cpu_has_tjmax(struct cpuinfo_x86 *c)
{
	u8 model = c->x86_model;

	return model > 0xe &&
	       model != 0x1c &&
	       model != 0x26 &&
	       model != 0x27 &&
	       model != 0x35 &&
	       model != 0x36;
}

static int get_tjmax(struct cpuinfo_x86 *c, u32 id)
{
	int err;
	u32 eax, edx;
	u32 val;

	/*
	 * A new feature of current Intel(R) processors, the
	 * IA32_TEMPERATURE_TARGET contains the TjMax value
	 */
	err = rdmsr_safe_on_cpu(id, MSR_IA32_TEMPERATURE_TARGET, &eax, &edx);
	if (err) {
		if (cpu_has_tjmax(c))
			printk("Unable to read TjMax from CPU %u\n", id);
	} else {
		val = (eax >> 16) & 0xff;
		/*
		 * If the TjMax is not plausible, an assumption
		 * will be used
		 */
		if (val) {
			//printk("TjMax is %d degrees C\n", val);
			return val * 1000;
		}
	}


	/*
	 * An assumption is made for early CPUs and unreadable MSR.
	 * NOTE: the calculated value may not be correct.
	 */
        printk("Assuming TjMax is 100 degrees C\n");
	return 100; //return adjust_tjmax(c, id, dev);
}

static ssize_t temp_show(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        u32 tj_max;
        u32 temp;
        int cpu;
        struct cpuinfo_x86 *c;
        u32 eax, edx;

        cpu = dev->id;
        c  = &cpu_data(cpu);

        /* TODO: read tj-max upon module initialization with per_cpu */
        tj_max = get_tjmax(c, cpu);

        rdmsr_on_cpu(cpu, MSR_IA32_THERM_STATUS, &eax, &edx);
        temp = tj_max - ((eax >> 16) & 0x7f) * 1000;
        //printk("[%d] tjmax is %d, temp is %d, status is %d\n", cpu,
               //tj_max, temp, ((eax >> 16) & 0x7f) * 1000);
        return scnprintf(buf, PAGE_SIZE, "%u\n", temp);
}

static ssize_t temp_resolution_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
        u64 therm_status;
        u32 temp_resolution;
        u8 temp_valid;
        int cpu = dev->id;

        rdmsrl_on_cpu(cpu, MSR_IA32_THERM_STATUS, &therm_status);
        temp_resolution = THERM_STATUS_RESOLUTION(therm_status);
        temp_valid = THERM_STATUS_VALID(therm_status);
        if (!temp_valid)
                temp_resolution = -1;

        return scnprintf(buf, PAGE_SIZE, "%u\n", temp_resolution);
}

static DEVICE_ATTR_RO(cpustat);
static DEVICE_ATTR_RW(perf_bias);
static DEVICE_ATTR_RO(hwp);
static DEVICE_ATTR_RO(turbo);
static DEVICE_ATTR_RW(turbo_disengage);
static DEVICE_ATTR_RO(temp);
static DEVICE_ATTR_RO(temp_resolution);

static struct attribute *default_attrs[] = {
	&dev_attr_cpustat.attr,
	&dev_attr_perf_bias.attr,
	&dev_attr_hwp.attr,
	&dev_attr_turbo.attr,
	&dev_attr_turbo_disengage.attr,
	&dev_attr_temp.attr,
	&dev_attr_temp_resolution.attr,
	NULL
};

static struct attribute_group cpustat_attr_group = {
	.attrs = default_attrs,
	.name = "cpustat"
};

static int cpustat_add_dev(unsigned int cpu)
{
	struct device *dev = get_cpu_device(cpu);

	return sysfs_create_group(&dev->kobj, &cpustat_attr_group);
}

static void cpustat_remove_dev(unsigned int cpu)
{
	struct device *dev = get_cpu_device(cpu);

	sysfs_remove_group(&dev->kobj, &cpustat_attr_group);
}

/*static int cpustat_cpu_callback(struct notifier_block *nfb,
				 unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	int rc = 0;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		rc = cpustat_add_dev(cpu);
                printk("Add cpu %d\n", cpu);
		break;
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		cpustat_remove_dev(cpu);
                printk("Del cpu %d\n", cpu);
		break;
	}
	return notifier_from_errno(rc);
}*/

static int cpustat_sysfs_init(void)
{
	int cpu;
	int rc = 0;
        struct cpuinfo_x86 *cpu_info = &boot_cpu_data;

        if (cpu_has(cpu_info, X86_FEATURE_HWP))
                hwp = 1;
        else
                hwp = 0;

        cpu_data_buffer = (struct cpu_data*)__get_free_pages(GFP_KERNEL, 0);
	if (cpu_data_buffer == NULL) {
		printk("__get_free_pages fail\n");
                return -ENOMEM;
        }

	cpu_notifier_register_begin();

	for_each_online_cpu(cpu) {
		rc = cpustat_add_dev(cpu);
		if (rc)
			goto out;
	}
        /* XXX When this is enabled, module will crash at second insmod
         __hotcpu_notifier(cpustat_cpu_callback, 0);
         */

out:
	cpu_notifier_register_done();
	return rc;
}

static void cpustat_sysfs_exit(void)
{
	int cpu;

	cpu_notifier_register_begin();

	for_each_online_cpu(cpu) {
		cpustat_remove_dev(cpu);
	}
//        __unregister_hotcpu_notifier(&cpustat_cpu_callback_nb);

	cpu_notifier_register_done();

        free_pages((unsigned long)cpu_data_buffer, 0);
	return;
}

static int __init cpustat_module_init(void)
{
        return cpustat_sysfs_init();
}

static void __exit cpustat_module_exit(void)
{
        cpustat_sysfs_exit();
}

module_init(cpustat_module_init);

module_exit(cpustat_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("pablo.llopis@gmail.com");
MODULE_AUTHOR("kazutomo@mcs.anl.gov");
