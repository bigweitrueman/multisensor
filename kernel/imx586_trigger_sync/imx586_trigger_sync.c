#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define DEVICE_NAME "imx586_trigger"
#define SYSFS_NAME "imx586_trigger"
#define EVENT_RING_SIZE 512
#define NS_PER_SEC 1000000000ULL

struct trigger_event {
	unsigned long long timestamp_ns;
	long long counter;
};

struct edge_event {
	u64 time_ns;
	int output;
	bool high;
};

enum sync_output {
	OUTPUT_CAMERA = 0,
	OUTPUT_PPS = 1,
};

static int camera_gpio_pin = 54;
module_param(camera_gpio_pin, int, 0644);
MODULE_PARM_DESC(camera_gpio_pin, "Camera trigger GPIO number");

static int pps_gpio_pin = 135;
module_param(pps_gpio_pin, int, 0644);
MODULE_PARM_DESC(pps_gpio_pin, "PPS output GPIO number");

static unsigned long long camera_period_ns = 100000000ULL;
module_param(camera_period_ns, ullong, 0644);
MODULE_PARM_DESC(camera_period_ns, "Camera trigger period in ns");

static unsigned long long camera_pulse_width_ns = 5000000ULL;
module_param(camera_pulse_width_ns, ullong, 0644);
MODULE_PARM_DESC(camera_pulse_width_ns, "Camera trigger high pulse width in ns");

static unsigned long long pps_period_ns = NS_PER_SEC;
module_param(pps_period_ns, ullong, 0644);
MODULE_PARM_DESC(pps_period_ns, "PPS period in ns");

static unsigned long long pps_pulse_width_ns = 10000000ULL;
module_param(pps_pulse_width_ns, ullong, 0644);
MODULE_PARM_DESC(pps_pulse_width_ns, "PPS high pulse width in ns");

static unsigned long long pps_phase_ns = 25778000ULL;
module_param(pps_phase_ns, ullong, 0644);
MODULE_PARM_DESC(pps_phase_ns, "PPS rising edge phase after camera trigger rising edge in ns");

static unsigned long long start_delay_ns = 10000000ULL;
module_param(start_delay_ns, ullong, 0644);
MODULE_PARM_DESC(start_delay_ns, "Delay from enable write to first scheduled edge in ns");

static struct hrtimer sync_timer;
static DEFINE_SPINLOCK(state_lock);
static DECLARE_WAIT_QUEUE_HEAD(event_wait);

static struct trigger_event event_ring[EVENT_RING_SIZE];
static unsigned int event_head;
static unsigned int event_tail;
static bool trigger_enabled;
static bool timer_active;
static u64 base_ns;
static u64 next_camera_rise_ns;
static u64 next_camera_fall_ns;
static u64 next_pps_rise_ns;
static u64 next_pps_fall_ns;
static long long camera_counter;
static struct kobject *trigger_kobj;

static u64 now_mono_ns(void)
{
	return ktime_to_ns(ktime_get());
}

static void ring_reset_locked(void)
{
	event_head = 0;
	event_tail = 0;
}

static void push_event_locked(u64 timestamp_ns, long long counter)
{
	unsigned int next_head = (event_head + 1) % EVENT_RING_SIZE;

	if (next_head == event_tail)
		event_tail = (event_tail + 1) % EVENT_RING_SIZE;

	event_ring[event_head].timestamp_ns = timestamp_ns;
	event_ring[event_head].counter = counter;
	event_head = next_head;
	wake_up_interruptible(&event_wait);
}

static bool pop_event(struct trigger_event *event)
{
	unsigned long flags;
	bool ok = false;

	spin_lock_irqsave(&state_lock, flags);
	if (event_tail != event_head) {
		*event = event_ring[event_tail];
		event_tail = (event_tail + 1) % EVENT_RING_SIZE;
		ok = true;
	}
	spin_unlock_irqrestore(&state_lock, flags);
	return ok;
}

static u64 min_nonzero(u64 a, u64 b)
{
	if (!a)
		return b;
	if (!b)
		return a;
	return a < b ? a : b;
}

static void schedule_next_locked(void)
{
	u64 next_ns = 0;

	next_ns = min_nonzero(next_ns, next_camera_rise_ns);
	next_ns = min_nonzero(next_ns, next_camera_fall_ns);
	next_ns = min_nonzero(next_ns, next_pps_rise_ns);
	next_ns = min_nonzero(next_ns, next_pps_fall_ns);

	if (!trigger_enabled || !next_ns) {
		timer_active = false;
		return;
	}

	timer_active = true;
	hrtimer_start(&sync_timer, ns_to_ktime(next_ns), HRTIMER_MODE_ABS);
}

static void handle_edge_locked(const struct edge_event *edge)
{
	if (edge->output == OUTPUT_CAMERA) {
		gpio_set_value(camera_gpio_pin, edge->high ? 1 : 0);
		if (edge->high) {
			push_event_locked(edge->time_ns, camera_counter);
			camera_counter++;
			next_camera_rise_ns += camera_period_ns;
			next_camera_fall_ns = edge->time_ns + camera_pulse_width_ns;
		} else {
			next_camera_fall_ns = 0;
		}
	} else {
		gpio_set_value(pps_gpio_pin, edge->high ? 1 : 0);
		if (edge->high) {
			next_pps_rise_ns += pps_period_ns;
			next_pps_fall_ns = edge->time_ns + pps_pulse_width_ns;
		} else {
			next_pps_fall_ns = 0;
		}
	}
}

static bool next_due_edge_locked(u64 now_ns, struct edge_event *edge)
{
	u64 due = now_ns + 1000ULL;

	if (next_camera_rise_ns && next_camera_rise_ns <= due) {
		edge->time_ns = next_camera_rise_ns;
		edge->output = OUTPUT_CAMERA;
		edge->high = true;
		return true;
	}
	if (next_camera_fall_ns && next_camera_fall_ns <= due) {
		edge->time_ns = next_camera_fall_ns;
		edge->output = OUTPUT_CAMERA;
		edge->high = false;
		return true;
	}
	if (next_pps_rise_ns && next_pps_rise_ns <= due) {
		edge->time_ns = next_pps_rise_ns;
		edge->output = OUTPUT_PPS;
		edge->high = true;
		return true;
	}
	if (next_pps_fall_ns && next_pps_fall_ns <= due) {
		edge->time_ns = next_pps_fall_ns;
		edge->output = OUTPUT_PPS;
		edge->high = false;
		return true;
	}

	return false;
}

static enum hrtimer_restart sync_timer_callback(struct hrtimer *timer)
{
	unsigned long flags;
	u64 now_ns;
	struct edge_event edge;

	spin_lock_irqsave(&state_lock, flags);
	if (!trigger_enabled) {
		timer_active = false;
		spin_unlock_irqrestore(&state_lock, flags);
		return HRTIMER_NORESTART;
	}

	now_ns = now_mono_ns();
	while (next_due_edge_locked(now_ns, &edge))
		handle_edge_locked(&edge);

	schedule_next_locked();
	spin_unlock_irqrestore(&state_lock, flags);
	return HRTIMER_NORESTART;
}

static int set_trigger_enabled(bool enable)
{
	unsigned long flags;

	if (enable) {
		if (camera_period_ns == 0 || pps_period_ns == 0 ||
		    camera_pulse_width_ns == 0 || pps_pulse_width_ns == 0 ||
		    camera_pulse_width_ns >= camera_period_ns ||
		    pps_pulse_width_ns >= pps_period_ns ||
		    pps_phase_ns >= pps_period_ns) {
			pr_err(DEVICE_NAME ": invalid timing parameters\n");
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&state_lock, flags);

	if (trigger_enabled == enable) {
		spin_unlock_irqrestore(&state_lock, flags);
		return 0;
	}

	if (!enable) {
		trigger_enabled = false;
		next_camera_rise_ns = 0;
		next_camera_fall_ns = 0;
		next_pps_rise_ns = 0;
		next_pps_fall_ns = 0;
		gpio_set_value(camera_gpio_pin, 0);
		gpio_set_value(pps_gpio_pin, 0);
		spin_unlock_irqrestore(&state_lock, flags);
		hrtimer_cancel(&sync_timer);
		return 0;
	}

	ring_reset_locked();
	camera_counter = 0;
	base_ns = now_mono_ns() + start_delay_ns;
	next_camera_rise_ns = base_ns;
	next_camera_fall_ns = 0;
	next_pps_rise_ns = base_ns + pps_phase_ns;
	next_pps_fall_ns = 0;
	gpio_set_value(camera_gpio_pin, 0);
	gpio_set_value(pps_gpio_pin, 0);
	push_event_locked(base_ns, -1);
	trigger_enabled = true;
	schedule_next_locked();

	spin_unlock_irqrestore(&state_lock, flags);
	return 0;
}

static ssize_t enable_show(struct kobject *kobj,
			   struct kobj_attribute *attr, char *buf)
{
	bool enabled;
	unsigned long flags;

	spin_lock_irqsave(&state_lock, flags);
	enabled = trigger_enabled;
	spin_unlock_irqrestore(&state_lock, flags);

	return sysfs_emit(buf, "%u\n", enabled ? 1 : 0);
}

static ssize_t enable_store(struct kobject *kobj,
			    struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	ret = set_trigger_enabled(enable);
	if (ret)
		return ret;

	return count;
}

static struct kobj_attribute enable_attr =
	__ATTR(enable, 0664, enable_show, enable_store);

static ssize_t trigger_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct trigger_event event;
	int ret;

	if (count < sizeof(event))
		return -EINVAL;

	if (!pop_event(&event)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(event_wait,
					       event_tail != event_head);
		if (ret)
			return ret;

		if (!pop_event(&event))
			return -EAGAIN;
	}

	if (copy_to_user(buf, &event, sizeof(event)))
		return -EFAULT;

	return sizeof(event);
}

static __poll_t trigger_poll(struct file *file, poll_table *wait)
{
	__poll_t mask = 0;
	unsigned long flags;

	poll_wait(file, &event_wait, wait);

	spin_lock_irqsave(&state_lock, flags);
	if (event_tail != event_head)
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&state_lock, flags);

	return mask;
}

static const struct file_operations trigger_fops = {
	.owner = THIS_MODULE,
	.read = trigger_read,
	.poll = trigger_poll,
	.llseek = no_llseek,
};

static struct miscdevice trigger_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &trigger_fops,
	.mode = 0666,
};

static int request_output_gpio(int gpio, const char *label)
{
	int ret;

	if (!gpio_is_valid(gpio)) {
		pr_err(DEVICE_NAME ": invalid %s GPIO %d\n", label, gpio);
		return -EINVAL;
	}

	if (gpio_cansleep(gpio)) {
		pr_err(DEVICE_NAME ": %s GPIO %d can sleep; not safe in hrtimer\n",
		       label, gpio);
		return -EINVAL;
	}

	ret = gpio_request(gpio, label);
	if (ret) {
		pr_err(DEVICE_NAME ": request %s GPIO %d failed: %d\n",
		       label, gpio, ret);
		return ret;
	}

	ret = gpio_direction_output(gpio, 0);
	if (ret) {
		gpio_free(gpio);
		pr_err(DEVICE_NAME ": set %s GPIO %d output failed: %d\n",
		       label, gpio, ret);
		return ret;
	}

	return 0;
}

static int __init trigger_init(void)
{
	int ret;

	ret = request_output_gpio(camera_gpio_pin, "imx586_trigger");
	if (ret)
		return ret;

	ret = request_output_gpio(pps_gpio_pin, "imx586_sync_pps");
	if (ret)
		goto free_camera_gpio;

	hrtimer_init(&sync_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	sync_timer.function = sync_timer_callback;

	ret = misc_register(&trigger_miscdev);
	if (ret) {
		pr_err(DEVICE_NAME ": misc_register failed: %d\n", ret);
		goto free_pps_gpio;
	}

	trigger_kobj = kobject_create_and_add(SYSFS_NAME, kernel_kobj);
	if (!trigger_kobj) {
		ret = -ENOMEM;
		goto deregister_misc;
	}

	ret = sysfs_create_file(trigger_kobj, &enable_attr.attr);
	if (ret)
		goto put_kobj;

	pr_info(DEVICE_NAME
		": loaded camera_gpio=%d pps_gpio=%d camera_period=%llu ns "
		"camera_width=%llu ns pps_period=%llu ns pps_width=%llu ns "
		"pps_phase=%llu ns\n",
		camera_gpio_pin, pps_gpio_pin, camera_period_ns,
		camera_pulse_width_ns, pps_period_ns, pps_pulse_width_ns,
		pps_phase_ns);
	return 0;

put_kobj:
	kobject_put(trigger_kobj);
	trigger_kobj = NULL;
deregister_misc:
	misc_deregister(&trigger_miscdev);
free_pps_gpio:
	gpio_free(pps_gpio_pin);
free_camera_gpio:
	gpio_free(camera_gpio_pin);
	return ret;
}

static void __exit trigger_exit(void)
{
	set_trigger_enabled(false);
	if (trigger_kobj) {
		sysfs_remove_file(trigger_kobj, &enable_attr.attr);
		kobject_put(trigger_kobj);
		trigger_kobj = NULL;
	}
	misc_deregister(&trigger_miscdev);
	gpio_set_value(camera_gpio_pin, 0);
	gpio_set_value(pps_gpio_pin, 0);
	gpio_free(pps_gpio_pin);
	gpio_free(camera_gpio_pin);
	pr_info(DEVICE_NAME ": unloaded\n");
}

module_init(trigger_init);
module_exit(trigger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("scanark");
MODULE_DESCRIPTION("Synchronized IMX586 camera trigger and PPS GPIO generator");
