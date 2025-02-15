// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#define pr_fmt(fmt) "msm_thermal_simple: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/sysfs.h>

#define OF_READ_U32(node, prop, dst)						\
({										\
	int ret = of_property_read_u32(node, prop, &(dst));			\
	if (ret)								\
		pr_err("%s: " prop " property missing\n", (node)->name);	\
	ret;									\
})

#define OF_READ_STRING(node, prop, dst)						\
({										\
	int ret = of_property_read_string(node, prop, &(dst));			\
	if (ret)								\
		pr_err("%s: " prop " property missing\n", (node)->name);	\
	ret;									\
})

#define WINDOW 10

struct thermal_zone {
	u32 gold_khz;
	u32 prime_khz;
	u32 silver_khz;
	s32 trip_deg;
};

struct thermal_drv {
	const char *zone_name;
	struct notifier_block cpu_notif;
	struct delayed_work throttle_work;
	struct workqueue_struct *wq;
	struct thermal_zone *zones;
	struct thermal_zone *curr_zone;
	u32 poll_jiffies;
	u32 start_delay;
	u32 nr_zones;
	int temp_history[WINDOW];
	int temp_index;
	bool wait;
};

static bool throttle_enabled = true; // Default: thermal throttling enabled
static struct thermal_drv *thermal_drv_instance;

static void update_online_cpu_policy(void)
{
	unsigned int cpu;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		if (cpumask_intersects(cpumask_of(cpu), cpu_lp_mask))
			cpufreq_update_policy(cpu);
		if (cpumask_intersects(cpumask_of(cpu), cpu_perf_mask))
			cpufreq_update_policy(cpu);
		if (cpumask_intersects(cpumask_of(cpu), cpu_prime_mask))
			cpufreq_update_policy(cpu);
	}
	cpus_read_unlock();
}

static void thermal_throttle_worker(struct work_struct *work)
{
	struct thermal_drv *t = container_of(to_delayed_work(work), typeof(*t), throttle_work);
	struct thermal_zone *new_zone = NULL, *old_zone = t->curr_zone;
	int temp = 0, temp_final = 0;
	s64 temp_sum = 0;
	short i = 0;
	char zone[15];
	struct thermal_zone_device *tz;

	/* Return if thermal throttling disabled */
	if (!throttle_enabled)
		return;

	if (t->zone_name) {
		tz = thermal_zone_get_zone_by_name(t->zone_name);
		if (!tz) {
			pr_err("Thermal zone %s not found\n", t->zone_name);
			return;
		}
		thermal_zone_get_temp(tz, &temp);
		temp_final = temp;
		snprintf(zone, sizeof(zone), "%s", t->zone_name);
	} else {
		for (i; i < NR_CPUS; i++) {
			snprintf(zone, sizeof(zone), "cpu-1-%i-usr", i);
			tz = thermal_zone_get_zone_by_name(zone);
			if (!tz) {
				pr_err("Thermal zone %s not found\n", t->zone_name);
				return;
			}
			thermal_zone_get_temp(tz, &temp);
			temp_sum += temp;
		}
		temp_final = temp_sum / NR_CPUS;
		snprintf(zone, sizeof(zone), "average");
	}

	// Store the current temperature
	t->temp_history[t->temp_index] = temp_final;
	t->temp_index = (t->temp_index + 1) % WINDOW;

	// Wait until history is ready
	if (t->wait) {
		if (t->temp_index == 0) {
			pr_info("init 100%%\n");
			t->wait = false;
		} else {
			pr_info("init %i%%\n", (t->temp_index * 100) / WINDOW);
			queue_delayed_work(t->wq, &t->throttle_work, t->poll_jiffies);
			return;
		}
	}

	// Calculate average temperatures
	temp_sum = 0;
	for (i = 0; i < WINDOW; i++) {
		temp_sum += t->temp_history[i];
	}
	temp_final = temp_sum / WINDOW;

	for (i = t->nr_zones - 1; i >= 0; i--) {
		if (temp_final >= t->zones[i].trip_deg) {
			new_zone = t->zones + i;
			break;
		}
	}

	/* Update thermal zone if it changed */
	if (new_zone != old_zone) {
		pr_info("temp=%i, zone=%s\n", temp_final, zone);
		t->curr_zone = new_zone;
		update_online_cpu_policy();
	}

	queue_delayed_work(t->wq, &t->throttle_work, t->poll_jiffies);
}

static u32 get_throttle_freq(struct thermal_zone *zone, u32 cpu)
{
	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		return zone->silver_khz;
	else if (cpumask_test_cpu(cpu, cpu_perf_mask))
		return zone->gold_khz;

	return zone->prime_khz;
}

static int cpu_notifier_cb(struct notifier_block *nb, unsigned long val,
			   void *data)
{
	struct thermal_drv *t = container_of(nb, typeof(*t), cpu_notif);
	struct cpufreq_policy *policy = data;
	struct thermal_zone *zone;

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	zone = t->curr_zone;
	if (zone && throttle_enabled)
		policy->max = get_throttle_freq(zone, policy->cpu);
	else
		policy->max = policy->user_policy.max;

	if (policy->max < policy->min)
		policy->min = policy->max;

	return NOTIFY_OK;
}

static int msm_thermal_simple_parse_dt(struct platform_device *pdev,
				       struct thermal_drv *t)
{
	struct device_node *child, *node = pdev->dev.of_node;
	int ret;

	ret = OF_READ_U32(node, "qcom,poll-ms", t->poll_jiffies);
	if (ret)
		return ret;

	/* Specifying a start delay is optional */
	OF_READ_U32(node, "qcom,start-delay", t->start_delay);

	/* Specifying a thermal zone is optional */
	OF_READ_STRING(node, "qcom,thermal-zone", t->zone_name);

	/* Convert polling milliseconds to jiffies */
	t->poll_jiffies = msecs_to_jiffies(t->poll_jiffies);

	/* Calculate the number of zones */
	for_each_child_of_node(node, child)
		t->nr_zones++;

	if (!t->nr_zones) {
		pr_err("No zones specified\n");
		return -EINVAL;
	}

	t->zones = kmalloc(t->nr_zones * sizeof(*t->zones), GFP_KERNEL);
	if (!t->zones)
		return -ENOMEM;

	for_each_child_of_node(node, child) {
		struct thermal_zone *zone;
		u32 reg;

		ret = OF_READ_U32(child, "reg", reg);
		if (ret)
			goto free_zones;

		zone = t->zones + reg;

		ret = OF_READ_U32(child, "qcom,silver-khz", zone->silver_khz);
		if (ret)
			goto free_zones;

		ret = OF_READ_U32(child, "qcom,gold-khz", zone->gold_khz);
		if (ret)
			goto free_zones;

		ret = OF_READ_U32(child, "qcom,prime-khz", zone->prime_khz);
		if (ret)
			goto free_zones;

		ret = OF_READ_U32(child, "qcom,trip-deg", zone->trip_deg);
		if (ret)
			goto free_zones;
	}

	return 0;

free_zones:
	kfree(t->zones);
	return ret;
}

static ssize_t throttle_enabled_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", throttle_enabled);
}

static ssize_t throttle_enabled_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int value;
	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	throttle_enabled = (value != 0);
	pr_info("Thermal throttling %s\n", throttle_enabled ? "enabled" : "disabled");

	if (throttle_enabled && thermal_drv_instance) {
		struct thermal_drv *t = thermal_drv_instance;
		memset(t->temp_history, 0, sizeof(t->temp_history));
		t->temp_index = 0;
		t->wait = true;
		queue_delayed_work(t->wq, &t->throttle_work, t->poll_jiffies);
	}

	return count;
}

static struct kobj_attribute throttle_enabled_attr = __ATTR(throttle_enabled, 0644, throttle_enabled_show, throttle_enabled_store);

static struct kobject *thermal_kobj;

static int create_sysfs_interface(void)
{
	int ret;

	thermal_kobj = kobject_create_and_add("msm_thermal_simple", kernel_kobj);
	if (!thermal_kobj)
		return -ENOMEM;

	ret = sysfs_create_file(thermal_kobj, &throttle_enabled_attr.attr);
	if (ret) {
		kobject_put(thermal_kobj);
		return ret;
	}

	return 0;
}

static int msm_thermal_simple_probe(struct platform_device *pdev)
{
	struct thermal_drv *t;
	int ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->wq = alloc_workqueue("msm_thermal_simple", WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!t->wq) {
		ret = -ENOMEM;
		goto free_t;
	}

	ret = msm_thermal_simple_parse_dt(pdev, t);
	if (ret)
		goto destroy_wq;

	/* Initialize the temperature history with 0 */
	memset(t->temp_history, 0, sizeof(t->temp_history));
	t->temp_index = 0;
	t->wait = true;

	/* Set the priority to INT_MIN so throttling can't be tampered with */
	t->cpu_notif.notifier_call = cpu_notifier_cb;
	t->cpu_notif.priority = INT_MIN;
	ret = cpufreq_register_notifier(&t->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
		goto free_zones;
	}

	/* Initialize sysfs thermal throttling switch */
	ret = create_sysfs_interface();
	if (ret) {
		pr_err("Failed to create sysfs interface, err: %d\n", ret);
		goto cpufreq_unregister;
	}

	/* Fire up the persistent worker */
	INIT_DELAYED_WORK(&t->throttle_work, thermal_throttle_worker);
	queue_delayed_work(t->wq, &t->throttle_work, t->start_delay * HZ);

	thermal_drv_instance = t;

	return 0;

cpufreq_unregister:
	cpufreq_unregister_notifier(&t->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
free_zones:
	kfree(t->zones);
destroy_wq:
	destroy_workqueue(t->wq);
free_t:
	kfree(t);
	return ret;
}

static const struct of_device_id msm_thermal_simple_match_table[] = {
	{ .compatible = "qcom,msm-thermal-simple" },
	{ }
};

static struct platform_driver msm_thermal_simple_device = {
	.probe = msm_thermal_simple_probe,
	.driver = {
		.name = "msm-thermal-simple",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_simple_match_table
	}
};

static int __init msm_thermal_simple_init(void)
{
	return platform_driver_register(&msm_thermal_simple_device);
}
device_initcall(msm_thermal_simple_init);
