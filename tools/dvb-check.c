// SPDX-License-Identifier: GPL-2.0-or-later
#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <glob.h>
#include <inttypes.h>
#include <limits.h>
#include <linux/dvb/frontend.h>
#include <linux/magic.h>
#include <math.h>
#include <signal.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/klog.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef SYS_ROOT
#define SYS_ROOT "/sys"
#endif
#ifndef DEV_ROOT
#define DEV_ROOT "/dev"
#endif
#define VERSION "1.4.0"

static volatile sig_atomic_t stopping;
static volatile sig_atomic_t read_expired;
static double read_timeout = 0.25;
static bool specialized;
static int collection_errors;
static bool usbmon_loaded, debugfs_mounted;
static const unsigned parameter_cmds[] = {
	DTV_DELIVERY_SYSTEM, DTV_FREQUENCY, DTV_BANDWIDTH_HZ,
	DTV_SYMBOL_RATE, DTV_MODULATION, DTV_INVERSION, DTV_STREAM_ID,
};
static const char *const parameter_names[] = {
	"delivery_system", "frequency", "bandwidth_hz", "symbol_rate",
	"modulation", "inversion", "stream_id",
};
static const unsigned stat_cmds[] = {
	DTV_STAT_SIGNAL_STRENGTH, DTV_STAT_CNR,
	DTV_STAT_PRE_ERROR_BIT_COUNT, DTV_STAT_PRE_TOTAL_BIT_COUNT,
	DTV_STAT_POST_ERROR_BIT_COUNT, DTV_STAT_POST_TOTAL_BIT_COUNT,
	DTV_STAT_ERROR_BLOCK_COUNT, DTV_STAT_TOTAL_BLOCK_COUNT,
};
static const char *const stat_names[] = {
	"Signal", "C/N", "pre_error_bits", "pre_total_bits",
	"post_error_bits", "post_total_bits", "error_blocks", "total_blocks",
};
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
struct frontend {
	unsigned adapter, number;
	int fd;
	int open_error;
	bool previous_valid;
	bool profile_checked;
	const char *module;
	uint32_t parameters[ARRAY_SIZE(parameter_cmds)];
	struct dtv_fe_stats post_error, post_total;
};

/* O_NONBLOCK does not bound the DVB core's interruptible MFE open wait,
 * nor its frontend semaphore. Interrupt those waits without taking ownership
 * of the tuner. Uninterruptible driver I/O still depends on the driver.
 */
static void read_timeout_signal(int signal_number)
{
	(void)signal_number;
	read_expired = 1;
}

static void read_timer(bool start)
{
	struct itimerval timer = {0};
	if (start) {
		read_expired = 0;
		timer.it_value.tv_sec = (time_t)read_timeout;
		timer.it_value.tv_usec = (suseconds_t)((read_timeout - (time_t)read_timeout) * 1e6);
		/* Cover a deadline signal delivered between the flag check and
		 * entry into an interruptible syscall. Disarm before rendering.
		 */
		timer.it_interval.tv_usec = 10000;
	}
	if (setitimer(ITIMER_REAL, &timer, NULL) < 0) {
		perror("frontend read timer");
		exit(1);
	}
}

static int frontend_ioctl(int fd, unsigned long command, void *argument)
{
	if (read_expired || stopping) {
		errno = read_expired ? ETIMEDOUT : EINTR;
		return -1;
	}
	int result = ioctl(fd, command, argument);
	if (result < 0 && errno == EINTR && read_expired) errno = ETIMEDOUT;
	return result;
}

static int frontend_open(const char *path)
{
	if (read_expired || stopping) {
		errno = read_expired ? ETIMEDOUT : EINTR;
		return -1;
	}
	int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0 && errno == EINTR && read_expired) errno = ETIMEDOUT;
	return fd;
}

static bool temporary_error(int error)
{
	return error == EBUSY || error == EAGAIN || error == ETIMEDOUT;
}

static bool unsupported_error(int error)
{
	/* DVB core can return the kernel-only ENOTSUPP (524) unchanged when
	 * a legacy callback is absent. libc only defines EOPNOTSUPP.
	 */
	return error == EINVAL || error == EOPNOTSUPP || error == ENOTTY || error == ENOSYS || error == 524;
}

static void interrupted(int signal_number)
{
	(void)signal_number;
	stopping = 1;
}

static char *path_format(const char *format, ...)
{
	char *path;
	va_list args;
	va_start(args, format);
	int result = vasprintf(&path, format, args);
	va_end(args);
	if (result < 0) { perror("allocate path"); exit(1); }
	return path;
}

static void clean_text(char *text)
{
	for (; *text; text++)
		if (iscntrl((unsigned char)*text)) *text = '?';
}

static bool attribute(const char *directory, const char *name, char *value, size_t size)
{
	char *path = path_format("%s/%s", directory, name);
	FILE *file = fopen(path, "r");
	free(path);
	if (!file) return false;
	size_t length = fread(value, 1, size - 1, file);
	bool success = !ferror(file) && length > 0;
	fclose(file);
	while (length && isspace((unsigned char)value[length - 1])) length--;
	value[length] = 0;
	clean_text(value);
	return success;
}

static bool module_unsigned(const char *module, const char *parameter, unsigned *value)
{
	if (!module) return false;
	char *directory = path_format(SYS_ROOT "/module/%s/parameters", module);
	char text[64], *end;
	bool found = attribute(directory, parameter, text, sizeof(text));
	free(directory);
	if (!found || !isdigit((unsigned char)*text)) return false;
	errno = 0;
	unsigned long parsed = strtoul(text, &end, 0);
	if (errno || *end || parsed > UINT_MAX) return false;
	*value = (unsigned)parsed;
	return true;
}

static const char *frontend_module(const char *name)
{
	/* Identify the demodulator from its own reported name, never from a
	 * globally loaded module or an assumed USB/tuner pairing.
	 */
	const struct { const char *prefix, *module; } profiles[] = {
		{ "Panasonic MN88472", "mn88472" },
		{ "Panasonic MN88473", "mn88473" },
		{ "Realtek RTL2832 (DVB-T)", "rtl2832" },
		{ "Sony CXD2837ER ", "cxd2841er" },
		{ "Sony CXD2838ER ", "cxd2841er" },
		{ "Sony CXD2841ER ", "cxd2841er" },
		{ "Sony CXD2843ER ", "cxd2841er" },
		{ "Sony CXD2854ER ", "cxd2841er" },
	};
	for (size_t i = 0; i < ARRAY_SIZE(profiles); i++) {
		size_t length = strlen(profiles[i].prefix);
		if (!strncmp(name, profiles[i].prefix, length) &&
		    (profiles[i].prefix[length - 1] == ' ' || !name[length] || name[length] == ' '))
			return profiles[i].module;
	}
	return NULL;
}

static bool family(const struct frontend *f, const char *module)
{
	return f->module && !strcmp(f->module, module);
}

static void profile_information(const struct frontend *f)
{
	if (!specialized) return;
	printf("    specialized_profile=%s (kernel-reported frontend name; repository driver rules)\n",
	       f->module ? f->module : "no chip-specific profile");
	if (family(f, "cxd2841er")) {
		puts("    Sony: terrestrial/cable/ISDB strength is relative AGC; most locked statistics use a one-second cache.\n"
		     "    Detected DVB-C QAM needs stats_mask bit 16; other modulation readback may be a tuning request.\n"
		     "    TS-enable verification and register/gate failures require --kernel-log; lock alone does not prove TS delivery.");
	} else if (family(f, "mn88472")) {
		puts("    MN88472: bit counters unsupported; stats_mask bit 8 cannot enable BER.\n"
		     "    Strength is relative AGC; block counters are driver-accumulated, hardware sample lifetime unverified.\n"
		     "    Firmware-running/parity and wake/standby failures require --kernel-log; no live register probing.");
	} else if (family(f, "mn88473")) {
		puts("    MN88473: DVB-T/C BER supported; DVB-T2 BER unsupported in the repository driver.\n"
		     "    Decoded modulation/FEC/guard readback is not implemented; tune properties may be cached requests.\n"
		     "    Strength is relative AGC; accumulated BER/block counters have hardware sample lifetime unverified.\n"
		     "    Firmware/startup/TS state is not exposed by DVB: use --kernel-log for upload attempt/offset, parity,\n"
		     "    startup defaults, register 0x36 and serial variable-clock TS errors; compare cold and warm boots.");
	} else if (family(f, "rtl2832")) {
		puts("    RTL2832: DVB-T demodulator; block/UCB counters are not implemented by the repository driver.\n"
		     "    A sibling MN8847x frontend may share the physical RF tuner; use the active frontend.");
	}
}

static void print_attribute(const char *directory, const char *name)
{
	char value[512];
	if (attribute(directory, name, value, sizeof(value)))
		printf("    %s=%s\n", name, value);
}

static bool within(const char *path, const char *parent)
{
	size_t length = strlen(parent);
	return !strncmp(path, parent, length) && (!path[length] || path[length] == '/');
}

static const char *leaf(const char *path)
{
	const char *last = strrchr(path, '/');
	return last ? last + 1 : path;
}

static char *link_target(const char *directory, const char *name)
{
	char *path = path_format("%s/%s", directory, name);
	char *resolved = realpath(path, NULL);
	free(path);
	return resolved;
}

static void driver_information(const char *directory)
{
	char *driver = link_target(directory, "driver");
	if (!driver) return;
	printf("    bound_driver=%s\n", leaf(driver));
	char *module = link_target(driver, "module");
	if (module) {
		printf("    module=%s\n", leaf(module));
		print_attribute(module, "version");
		print_attribute(module, "srcversion");
		free(module);
	} else {
		puts("    module=not exposed (driver may be built in)");
	}
	free(driver);
}

static bool prepare_command(char *const command[], bool cleanup)
{
	char *sudo_command[8] = { "/usr/bin/sudo", cleanup ? "-n" : "--" };
	for (size_t i = 0; command[i] && i < ARRAY_SIZE(sudo_command) - 3; i++)
		sudo_command[i + 2] = command[i];
	pid_t child;
	char *const *args = geteuid() == 0 ? command : sudo_command;
	int error = posix_spawn(&child, args[0], NULL, NULL, args, environ);
	if (error) {
		fprintf(stderr, "USB tracing %s: %s: %s\n", cleanup ? "cleanup" : "setup", args[0], strerror(error));
		return false;
	}
	int status;
	while (waitpid(child, &status, 0) < 0) {
		if (errno == EINTR) continue;
		perror("USB tracing command wait");
		return false;
	}
	if (WIFEXITED(status) && WEXITSTATUS(status) == 0) return true;
	fprintf(stderr, "USB tracing %s failed: %s (status=%d)\n", cleanup ? "cleanup" : "setup", command[0], status);
	return false;
}

static void cleanup_usbmon(void)
{
	char *unmount[] = { "/bin/umount", SYS_ROOT "/kernel/debug", NULL };
	char *unload[] = { "/sbin/modprobe", "-r", "usbmon", NULL };
	if (debugfs_mounted) {
		debugfs_mounted = false;
		if (!prepare_command(unmount, true)) collection_errors++;
	}
	if (usbmon_loaded) {
		usbmon_loaded = false;
		if (!prepare_command(unload, true)) collection_errors++;
	}
}

static bool prepare_usbmon(void)
{
	struct statfs fs;
	bool mount_needed = statfs(SYS_ROOT "/kernel/debug", &fs) < 0 || fs.f_type != DEBUGFS_MAGIC;
	bool module_needed = access(SYS_ROOT "/module/usbmon", F_OK) != 0;
	if (!mount_needed && !module_needed) return true;
	if (!isatty(STDIN_FILENO)) {
		puts("USB tracing disabled: preparation needs interactive approval; tuner checks continue.");
		return false;
	}
	fprintf(stderr, "USB tracing preparation:%s%s\nChanges will be undone on exit. Allow setup%s? [y/N] ",
		module_needed ? " load usbmon;" : "", mount_needed ? " mount debugfs at /sys/kernel/debug;" : "",
		geteuid() == 0 ? "" : " using sudo");
	fflush(stderr);
	char answer[32];
	if (!fgets(answer, sizeof(answer), stdin) || stopping ||
	    (strcasecmp(answer, "y\n") && strcasecmp(answer, "yes\n"))) {
		puts("USB tracing disabled: setup declined; tuner checks continue.");
		return false;
	}
	char *module[] = { "/sbin/modprobe", "usbmon", NULL };
	char *mount[] = { "/bin/mount", "-t", "debugfs", "debugfs", SYS_ROOT "/kernel/debug", NULL };
	if (module_needed) {
		if (!prepare_command(module, false)) goto failed;
		usbmon_loaded = access(SYS_ROOT "/module/usbmon", F_OK) == 0;
	}
	if (mount_needed) {
		if (!prepare_command(mount, false)) goto failed;
		debugfs_mounted = true;
	}
	return true;
failed:
	cleanup_usbmon();
	puts("USB tracing disabled: setup failed; tuner checks continue.");
	return false;
}

static void usbmon_information(const char *usb)
{
	static bool checked, disabled;
	if (disabled) return;
	char bus[64];
	if (!attribute(usb, "busnum", bus, sizeof(bus))) return;
	char *end;
	errno = 0;
	unsigned long number = strtoul(bus, &end, 10);
	if (errno || !*bus || *end || number > UINT_MAX) return;
	char *path = path_format(SYS_ROOT "/kernel/debug/usb/usbmon/%luu", number);
	if (!checked && access(path, R_OK) != 0) {
		checked = true;
		disabled = !prepare_usbmon();
		if (disabled) { free(path); return; }
	}
	printf("    USB bus=%lu; usbmon=%s", number, path);
	if (access(path, R_OK) == 0) {
		puts(" (readable)");
		printf("    Optional capture: sudo timeout 20 cat %s > usbmon-bus%lu.txt\n", path, number);
	} else {
		printf(" (%s; text capture unavailable; try running with sudo if access is denied)\n", strerror(errno));
	}
	free(path);
	path = path_format(DEV_ROOT "/usbmon%lu", number);
	struct stat info;
	if (!stat(path, &info))
		printf("    usbmon binary interface=%s (%s)\n", path,
		       access(path, R_OK) == 0 ? "readable; optional pcap capture tool required" : "not readable");
	free(path);
}

static void hardware_information(unsigned adapter, unsigned frontend)
{
	char *class = path_format(SYS_ROOT "/class/dvb/dvb%u.frontend%u", adapter, frontend);
	char *device = link_target(class, "device");
	if (!device) device = realpath(class, NULL);
	free(class);
	if (!device) { puts("    sysfs device information not exposed"); return; }
	printf("    sysfs=%s\n", device);
	char *cursor = strdup(device), *anchor = NULL;
	if (!cursor) { perror("strdup"); exit(1); }
	bool bound = false;
	while (within(cursor, SYS_ROOT) && strcmp(cursor, SYS_ROOT)) {
		if (!bound) {
			char *driver = link_target(cursor, "driver");
			if (driver) { driver_information(cursor); bound = true; free(driver); }
		}
		char vendor[64], product[64];
		if (attribute(cursor, "idVendor", vendor, sizeof(vendor)) &&
		    attribute(cursor, "idProduct", product, sizeof(product))) {
			printf("    USB=%s:%s topology=%s\n", vendor, product, leaf(cursor));
			if (!strcmp(device, cursor)) {
				char *pattern = path_format("%s/%s:*", cursor, leaf(cursor));
				glob_t interfaces = {0};
				if (!glob(pattern, 0, NULL, &interfaces))
					for (size_t i = 0; i < interfaces.gl_pathc; i++) {
						printf("    USB interface=%s\n", leaf(interfaces.gl_pathv[i]));
						driver_information(interfaces.gl_pathv[i]);
					}
				globfree(&interfaces);
				free(pattern);
			}
			const char *const names[] = { "manufacturer", "product", "serial", "busnum", "devnum", "speed", "bcdDevice", "power/control", "power/runtime_status", "power/autosuspend_delay_ms" };
			for (size_t i = 0; i < ARRAY_SIZE(names); i++) print_attribute(cursor, names[i]);
			usbmon_information(cursor);
			anchor = strdup(cursor);
			break;
		}
		char *slash = strrchr(cursor, '/');
		if (!slash) break;
		*slash = 0;
	}
	free(cursor);
	if (!anchor) anchor = strdup(device);
	if (!anchor) { perror("strdup"); exit(1); }
	glob_t clients = {0};
	bool found = false;
	if (!glob(SYS_ROOT "/bus/i2c/devices/*", 0, NULL, &clients)) {
		for (size_t i = 0; i < clients.gl_pathc; i++) {
			char *real = realpath(clients.gl_pathv[i], NULL);
			char name[512];
			if (real && within(real, anchor) && attribute(real, "name", name, sizeof(name))) {
				printf("    I2C device=%s name=%s\n", leaf(clients.gl_pathv[i]), name);
				print_attribute(real, "modalias");
				driver_information(real);
				found = true;
			}
			free(real);
		}
	}
	globfree(&clients);
	if (!found) puts("    Bound I2C chip identities not exposed for this device");
	free(anchor);
	free(device);
}

static const char *delivery_name(unsigned system)
{
	switch (system) {
	case SYS_UNDEFINED: return "UNDEFINED";
	case SYS_DVBC_ANNEX_A: return "DVB-C/ANNEX_A";
	case SYS_DVBC_ANNEX_B: return "DVB-C/ANNEX_B";
	case SYS_DVBC_ANNEX_C: return "DVB-C/ANNEX_C";
	case SYS_DVBT: return "DVB-T";
	case SYS_DVBT2: return "DVB-T2";
	case SYS_DVBS: return "DVB-S";
	case SYS_DVBS2: return "DVB-S2";
	case SYS_DSS: return "DSS";
	case SYS_TURBO: return "TURBO";
	case SYS_ISDBS: return "ISDB-S";
	case SYS_ISDBT: return "ISDB-T";
	case SYS_ISDBC: return "ISDB-C";
	case SYS_ATSC: return "ATSC";
	case SYS_ATSCMH: return "ATSC-M/H";
	case SYS_DTMB: return "DTMB";
	case SYS_DAB: return "DAB";
	case SYS_DVBH: return "DVB-H";
	case SYS_CMMB: return "CMMB";
	default: return "unknown";
	}
}

static bool satellite(unsigned system)
{
	return system == SYS_DVBS || system == SYS_DVBS2 || system == SYS_DSS || system == SYS_TURBO || system == SYS_ISDBS;
}

static int property_error(const struct dtv_property *p)
{
	return p->result < 0 ? -p->result : p->result;
}

static void get_properties(int fd, struct dtv_property *props, unsigned count, int *errors)
{
	struct dtv_properties request = { .num = count, .props = props };
	if (frontend_ioctl(fd, FE_GET_PROPERTY, &request) == 0) {
		for (unsigned i = 0; i < count; i++) errors[i] = property_error(&props[i]);
		return;
	}
	int error = errno;
	for (unsigned i = 0; i < count; i++) {
		errors[i] = error;
		/* Older drivers may reject one unsupported property in a batch. */
		if (unsupported_error(error)) {
			unsigned command = props[i].cmd;
			memset(&props[i], 0, sizeof(props[i]));
			props[i].cmd = command;
			struct dtv_properties single = { .num = 1, .props = &props[i] };
			errors[i] = frontend_ioctl(fd, FE_GET_PROPERTY, &single) < 0 ? errno : property_error(&props[i]);
		}
	}
}

static void record_error(int error, bool optional)
{
	if (stopping && error == EINTR) return;
	if (!temporary_error(error) && (!optional || !unsupported_error(error)))
		collection_errors++;
}

static void unavailable(const char *name, int error, bool optional)
{
	printf("    %s=unavailable: %s (%d)\n", name, strerror(error), error);
	record_error(error, optional);
}

static int compare_frontends(const void *left, const void *right)
{
	const struct frontend *a = left, *b = right;
	if (a->adapter != b->adapter) return a->adapter > b->adapter ? 1 : -1;
	return (a->number > b->number) - (a->number < b->number);
}

static void print_capabilities(enum fe_caps caps)
{
#define CAP(name) { name, #name }
	const struct { unsigned bit; const char *name; } names[] = {
		CAP(FE_CAN_INVERSION_AUTO), CAP(FE_CAN_FEC_1_2), CAP(FE_CAN_FEC_2_3),
		CAP(FE_CAN_FEC_3_4), CAP(FE_CAN_FEC_4_5), CAP(FE_CAN_FEC_5_6),
		CAP(FE_CAN_FEC_6_7), CAP(FE_CAN_FEC_7_8), CAP(FE_CAN_FEC_8_9),
		CAP(FE_CAN_FEC_AUTO), CAP(FE_CAN_QPSK), CAP(FE_CAN_QAM_16),
		CAP(FE_CAN_QAM_32), CAP(FE_CAN_QAM_64), CAP(FE_CAN_QAM_128),
		CAP(FE_CAN_QAM_256), CAP(FE_CAN_QAM_AUTO), CAP(FE_CAN_TRANSMISSION_MODE_AUTO),
		CAP(FE_CAN_BANDWIDTH_AUTO), CAP(FE_CAN_GUARD_INTERVAL_AUTO), CAP(FE_CAN_HIERARCHY_AUTO),
		CAP(FE_CAN_8VSB), CAP(FE_CAN_16VSB), CAP(FE_HAS_EXTENDED_CAPS),
		CAP(FE_CAN_MULTISTREAM), CAP(FE_CAN_TURBO_FEC), CAP(FE_CAN_2G_MODULATION),
		CAP(FE_NEEDS_BENDING), CAP(FE_CAN_RECOVER), CAP(FE_CAN_MUTE_TS),
	};
#undef CAP
	for (size_t i = 0; i < ARRAY_SIZE(names); i++)
		if (caps & names[i].bit) printf("      %s\n", names[i].name);
}

static void add_frontend(struct frontend **items, size_t *count, unsigned adapter, unsigned number)
{
	for (size_t i = 0; i < *count; i++)
		if ((*items)[i].adapter == adapter && (*items)[i].number == number) return;
	struct frontend *grown = realloc(*items, (*count + 1) * sizeof(**items));
	if (!grown) { perror("allocate frontends"); exit(1); }
	*items = grown;
	(*items)[(*count)++] = (struct frontend){ .adapter = adapter, .number = number, .fd = -1 };
}

static size_t discover(struct frontend **items)
{
	size_t count = 0;
	const char *const patterns[] = { SYS_ROOT "/class/dvb/*.frontend*", DEV_ROOT "/dvb/adapter*/frontend*" };
	for (unsigned p = 0; p < ARRAY_SIZE(patterns); p++) {
		glob_t paths = {0};
		int result = glob(patterns[p], 0, NULL, &paths);
		if (result != 0 && result != GLOB_NOMATCH) {
			fprintf(stderr, "Cannot enumerate %s: glob error %d\n", patterns[p], result);
			collection_errors++;
		}
		for (size_t i = 0; i < paths.gl_pathc; i++) {
			unsigned adapter, number;
			int used = 0;
			const char *entry = p ? strstr(paths.gl_pathv[i] + strlen(DEV_ROOT), "/adapter") : leaf(paths.gl_pathv[i]);
			if (!entry) continue;
			int matched = sscanf(entry, p ? "/adapter%u/frontend%u%n" : "dvb%u.frontend%u%n", &adapter, &number, &used);
			if (matched == 2 && !entry[used]) add_frontend(items, &count, adapter, number);
		}
		globfree(&paths);
	}
	if (count) qsort(*items, count, sizeof(**items), compare_frontends);
	return count;
}

static void inventory(struct frontend *f)
{
	char *node = path_format(DEV_ROOT "/dvb/adapter%u/frontend%u", f->adapter, f->number);
	printf("\nDEVICE %s\n", node);
	hardware_information(f->adapter, f->number);
	read_timer(true);
	f->fd = frontend_open(node);
	free(node);
	if (f->fd < 0) {
		f->open_error = errno;
		read_timer(false);
		if (f->open_error == EBUSY) puts("    frontend busy (shared tuner in use; skipped)");
		else unavailable("open frontend read-only", f->open_error, false);
		return;
	}
	struct dvb_frontend_info info = {0};
	if (frontend_ioctl(f->fd, FE_GET_INFO, &info) < 0) {
		int error = errno;
		read_timer(false);
		close(f->fd); f->fd = -1;
		unavailable("FE_GET_INFO", error, false);
		return;
	}
	struct dtv_property props[2] = { { .cmd = DTV_API_VERSION }, { .cmd = DTV_ENUM_DELSYS } };
	int errors[2];
	get_properties(f->fd, props, 2, errors);
	read_timer(false);
	close(f->fd); f->fd = -1;
	info.name[sizeof(info.name) - 1] = 0;
	clean_text(info.name);
	f->module = frontend_module(info.name);
	f->profile_checked = true;
	printf("    frontend_name=%s\n    capabilities=0x%08x legacy_type=%u\n", info.name, info.caps, info.type);
	profile_information(f);
	print_capabilities(info.caps);
	printf("    frequency_range=%u..%u %s step=%u tolerance=%u\n", info.frequency_min, info.frequency_max,
	       info.type == FE_QPSK ? "kHz" : "Hz", info.frequency_stepsize, info.frequency_tolerance);
	printf("    symbol_rate_range=%u..%u baud\n", info.symbol_rate_min, info.symbol_rate_max);
	if (errors[0]) unavailable("DVB API", errors[0], true);
	else printf("    DVB API=%u.%u\n", props[0].u.data >> 8, props[0].u.data & 255);
	if (errors[1]) unavailable("delivery systems", errors[1], true);
	else if (props[1].u.buffer.len > sizeof(props[1].u.buffer.data)) {
		puts("    invalid delivery-system length from driver"); collection_errors++;
	} else {
		printf("    delivery_systems:");
		for (unsigned i = 0; i < props[1].u.buffer.len; i++)
			printf(" %s(%u)", delivery_name(props[1].u.buffer.data[i]), props[1].u.buffer.data[i]);
		puts("");
	}
}

static void module_information(void)
{
	puts("\nLoaded DVB module metadata (global; does not prove a particular chip is attached):");
	if (specialized) puts("  Module settings below are global, not a per-frontend tuner identification; no settings are changed.");
	glob_t modules = {0};
	if (!glob(SYS_ROOT "/module/*", 0, NULL, &modules)) {
		for (size_t i = 0; i < modules.gl_pathc; i++) {
			const char *name = leaf(modules.gl_pathv[i]);
			if (strncmp(name, "dvb", 3) && strncmp(name, "cxd", 3) && strncmp(name, "mn8847", 6) &&
			    strncmp(name, "dib", 3) && strncmp(name, "rtl28", 5) && strcmp(name, "r820t") &&
			    strcmp(name, "tda18250") && strcmp(name, "usbmon")) continue;
			printf("  %s\n", name);
			print_attribute(modules.gl_pathv[i], "version");
			print_attribute(modules.gl_pathv[i], "srcversion");
			print_attribute(modules.gl_pathv[i], "taint");
			print_attribute(modules.gl_pathv[i], "refcnt");
			char value[64];
			if (attribute(modules.gl_pathv[i], "parameters/no_imr_cal", value, sizeof(value)))
				printf("    no_imr_cal=%s\n", value);
			if (!specialized) continue;
			if (!strcmp(name, "cxd2841er") || !strcmp(name, "mn88472") || !strcmp(name, "mn88473")) {
				unsigned mask;
				if (module_unsigned(name, "stats_mask", &mask)) {
					printf("    stats_mask=%u: strength=%s CNR=%s blocks=%s bits=%s QAM=%s (repository bit assignments)\n",
					       mask, mask & 1 ? "on" : "off", mask & 2 ? "on" : "off", mask & 4 ? "on" : "off",
					       mask & 8 ? "on" : "off", mask & 16 ? "on" : "off");
					if (!strcmp(name, "mn88472")) puts("    Bit 8 is reserved: bit counters unsupported in the repository driver.");
					if (!strcmp(name, "mn88473")) puts("    Bit 8 only implements DVB-T/C BER; bit 16 is reserved (detected QAM unsupported).");
					if (mask & ~31u) printf("    Unrecognized stats_mask bits=0x%x; no meaning inferred.\n", mask & ~31u);
				} else puts("    stats_mask unavailable (not exposed, unreadable or invalid); no default assumed.");
			}
			const char *const parameters[] = { "debug", "dvb_mfe_wait_time", "dvb_shutdown_timeout", "nb_packet_buffer_size" };
			for (size_t p = 0; p < ARRAY_SIZE(parameters); p++) {
				char *parameter = path_format("parameters/%s", parameters[p]);
				if (attribute(modules.gl_pathv[i], parameter, value, sizeof(value))) printf("    %s=%s\n", parameters[p], value);
				free(parameter);
			}
			if (!strcmp(name, "r820t")) {
				unsigned disabled;
				if (module_unsigned(name, "no_imr_cal", &disabled))
					printf("    IMR calibration %s at module init; current calibration/PLL state needs kernel logs.\n", disabled ? "disabled" : "enabled");
				puts("    RF tuner PLL/calibration and demodulator lock are separate; relative AGC is not dBm.");
			} else if (!strcmp(name, "tda18250")) {
				puts("    TDA18250: power/charge-pump/standby failures require kernel logs; live PLL/standby state is not exposed by DVB.");
			} else if (!strcmp(name, "dvb_usb_dib0700")) {
				puts("    DiB0700: check kernel logs for short control/I2C transfers and bulk overflow/recovery; demodulator lock does not prove USB TS delivery.");
			}
		}
	}
	globfree(&modules);
}

static int validate_stat(const struct dtv_property *p, int error)
{
	if (error) return error;
	if (p->u.st.len > MAX_DTV_STATS) return EOVERFLOW;
	for (unsigned i = 0; i < p->u.st.len; i++)
		if (p->u.st.stat[i].scale > FE_SCALE_COUNTER) return EPROTO;
	return 0;
}

static bool missing_stat(const struct dtv_property *p, int error)
{
	/* An explicit NOT_AVAILABLE may mean stats_mask disabled the read.
	 * Do not override it or hide genuine I/O failures with legacy reads.
	 */
	return unsupported_error(error) || (!error && !p->u.st.len);
}

static void legacy_stat(int fd, struct dtv_property *p, int *error,
		unsigned long command, bool counter)
{
	if (!missing_stat(p, *error)) return;
	uint16_t relative = 0;
	uint32_t count = 0;
	if (frontend_ioctl(fd, command, counter ? (void *)&count : (void *)&relative) < 0) {
		if (!unsupported_error(errno)) *error = errno;
		return;
	}
	memset(&p->u.st, 0, sizeof(p->u.st));
	p->u.st.len = 1;
	p->u.st.stat[0].scale = counter ? FE_SCALE_COUNTER : FE_SCALE_RELATIVE;
	p->u.st.stat[0].uvalue = counter ? count : relative;
	*error = 0;
}

static void print_stat(const struct dtv_property *p, unsigned which, int error)
{
	if (error) {
		if (error == EOVERFLOW) puts("    invalid statistics length from driver");
		if (error == EPROTO) puts("    invalid statistics: unknown scale from driver");
		unavailable(stat_names[which], error, true);
		return;
	}
	if (!p->u.st.len) { printf("    %s=unavailable (not exposed)\n", stat_names[which]); return; }
	for (unsigned i = 0; i < p->u.st.len; i++) {
		const struct dtv_stats *s = &p->u.st.stat[i];
		printf("    %s[layer=%u]=", stat_names[which], i);
		switch (s->scale) {
		case FE_SCALE_NOT_AVAILABLE: printf("unavailable"); break;
		case FE_SCALE_DECIBEL:
			printf("%.3f %s", s->svalue / 1000.0, which == 0 ? "dBm" : "dB");
			if (which == 0 && s->svalue > 0) printf(" (check driver calibration/units)");
			break;
		case FE_SCALE_RELATIVE:
			printf("%" PRIu64 "/65535 relative (%.2f%%)", (uint64_t)s->uvalue, s->uvalue * (100.0 / 65535));
			break;
		case FE_SCALE_COUNTER: printf("%" PRIu64, (uint64_t)s->uvalue); break;
		default: printf("unavailable (unknown scale %u)", s->scale); collection_errors++; break;
		}
		puts("");
	}
}

enum ber_result { BER_MISSING, BER_RESET, BER_PAUSED, BER_VALID };

static enum ber_result counter_interval(const struct frontend *f,
		const struct dtv_fe_stats *error, const struct dtv_fe_stats *total,
		unsigned layer, uint64_t *delta_error, uint64_t *delta_total)
{
	if (layer >= error->len || layer >= total->len || layer >= f->post_error.len || layer >= f->post_total.len ||
	    error->stat[layer].scale != FE_SCALE_COUNTER || total->stat[layer].scale != FE_SCALE_COUNTER ||
	    f->post_error.stat[layer].scale != FE_SCALE_COUNTER || f->post_total.stat[layer].scale != FE_SCALE_COUNTER)
		return BER_MISSING;
	uint64_t e = error->stat[layer].uvalue, t = total->stat[layer].uvalue;
	uint64_t pe = f->post_error.stat[layer].uvalue, pt = f->post_total.stat[layer].uvalue;
	if (e < pe || t < pt || e - pe > t - pt) return BER_RESET;
	*delta_error = e - pe;
	*delta_total = t - pt;
	return *delta_total ? BER_VALID : BER_PAUSED;
}

static void interval_ber(const struct frontend *f, const struct dtv_fe_stats *error, const struct dtv_fe_stats *total)
{
	unsigned count = error->len < total->len ? error->len : total->len;
	for (unsigned i = 0; i < count; i++) {
		uint64_t e, t;
		switch (counter_interval(f, error, total, i, &e, &t)) {
		case BER_MISSING: break;
		case BER_RESET:
			printf("    postBER_interval unavailable: counter reset/inconsistent (layer=%u)\n", i);
			break;
		case BER_PAUSED:
			printf("    postBER_interval unavailable: no new counted bits (layer=%u)\n", i);
			break;
		case BER_VALID:
			printf("    postBER_interval=%.9g (layer=%u; errors=%" PRIu64 " bits=%" PRIu64 ")\n", (double)e / t, i, e, t);
			break;
		}
	}
}

static void table_stat(char *text, size_t size, const struct dtv_property *p, int error, unsigned which)
{
	snprintf(text, size, "n/a");
	if (error || !p->u.st.len) return;
	const struct dtv_stats *s = &p->u.st.stat[0];
	if (s->scale == FE_SCALE_DECIBEL)
		snprintf(text, size, "%.3f%s%s", s->svalue / 1000.0, which == 0 ? "dBm" : "dB",
			 which == 0 && s->svalue > 0 ? "!" : "");
	else if (s->scale == FE_SCALE_RELATIVE)
		snprintf(text, size, "%.1f%%rel", s->uvalue * (100.0 / 65535));
	else if (s->scale == FE_SCALE_COUNTER)
		snprintf(text, size, "%" PRIu64, (uint64_t)s->uvalue);
}

static void table_error(char *text, size_t size, const char *name, int error, bool optional)
{
	if (!error) return;
	record_error(error, optional);
	if (!*text) snprintf(text, size, "%s: %s (%d)", name, strerror(error), error);
}

static void diagnostic_note(char *text, size_t size, const char *format, ...)
{
	size_t used = strlen(text);
	if (used >= size - 1) return;
	if (used) {
		int added = snprintf(text + used, size - used, "; ");
		if (added < 0 || (size_t)added >= size - used) return;
		used += (size_t)added;
	}
	va_list args;
	va_start(args, format);
	vsnprintf(text + used, size - used, format, args);
	va_end(args);
}

static bool reported_stat(const struct dtv_property *p, int error)
{
	return !error && p->u.st.len && p->u.st.stat[0].scale != FE_SCALE_NOT_AVAILABLE;
}

static void specialized_diagnosis(const struct frontend *f,
		const struct dtv_property *parameters, const int *errors,
		const struct dtv_property *stats, const int *stat_errors,
		char *text, size_t size)
{
	if (!f->module) {
		diagnostic_note(text, size, "no chip-specific profile");
		return;
	}
	bool mn72 = family(f, "mn88472"), mn73 = family(f, "mn88473");
	bool system_valid = !errors[0];
	unsigned system = parameters[0].u.data;
	bool t2 = system_valid && system == SYS_DVBT2;
	bool missing_bits = !reported_stat(&stats[4], stat_errors[4]) || !reported_stat(&stats[5], stat_errors[5]);
	bool pre_bits = reported_stat(&stats[2], stat_errors[2]) && reported_stat(&stats[3], stat_errors[3]) &&
		stats[2].u.st.stat[0].scale == FE_SCALE_COUNTER && stats[3].u.st.stat[0].scale == FE_SCALE_COUNTER;
	bool missing_blocks = !reported_stat(&stats[6], stat_errors[6]);
	unsigned mask;
	bool mask_valid = module_unsigned(f->module, "stats_mask", &mask);
	if (!family(f, "rtl2832")) {
		if (!mask_valid) diagnostic_note(text, size, "stats_mask unavailable");
		else {
			if (!(mask & 1) && !reported_stat(&stats[0], stat_errors[0])) diagnostic_note(text, size, "strength disabled by stats_mask");
			if (!(mask & 2) && !reported_stat(&stats[1], stat_errors[1])) diagnostic_note(text, size, "C/N disabled by stats_mask");
			if (!(mask & 4) && missing_blocks) diagnostic_note(text, size, "UCB disabled by stats_mask");
			if (!(mask & 8) && missing_bits && !pre_bits && !mn72 && !(mn73 && (!system_valid || t2)))
				diagnostic_note(text, size, "BER disabled by stats_mask");
		}
	}
	if (mn72 && missing_bits) diagnostic_note(text, size, "MN88472 bit counters unsupported");
	if (mn73 && t2 && missing_bits) diagnostic_note(text, size, "MN88473 DVB-T2 BER unsupported");
	if (family(f, "rtl2832") && missing_blocks) diagnostic_note(text, size, "RTL2832 UCB unsupported");
	if (family(f, "cxd2841er") && pre_bits && missing_bits)
		diagnostic_note(text, size, "pre-FEC counters available; postBER not exposed");
	if (t2 && !errors[6]) {
		unsigned plp = parameters[6].u.data;
		if (plp == UINT32_MAX || (!mn73 && plp > 255)) diagnostic_note(text, size, "PLP request=auto (not detected PLP)");
		else if (plp > 255) diagnostic_note(text, size, "PLP request out of range: %u", plp);
		else diagnostic_note(text, size, "PLP request=%u (ID; not detected PLP)", plp);
	}
	if (reported_stat(&stats[0], stat_errors[0]) && stats[0].u.st.stat[0].scale == FE_SCALE_RELATIVE &&
	    stats[0].u.st.stat[0].uvalue > 65535)
		diagnostic_note(text, size, "relative AGC outside 0..65535");
	if ((mn72 || mn73) && reported_stat(&stats[1], stat_errors[1]) &&
	    stats[1].u.st.stat[0].scale == FE_SCALE_DECIBEL && stats[1].u.st.stat[0].svalue < 0)
		diagnostic_note(text, size, "negative C/N: check loaded driver revision");
	if (!*text) diagnostic_note(text, size, "no profile-specific finding; lock does not verify TS delivery");
}

static void sample_frontend(struct frontend *f, bool table, FILE *table_output)
{
	char node[64];
	snprintf(node, sizeof(node), "adapter%u/frontend%u", f->adapter, f->number);
	if (!table) printf("  adapter%u/frontend%u\n", f->adapter, f->number);
	read_timer(true);
	if (!f->open_error || temporary_error(f->open_error)) {
		char *path = path_format(DEV_ROOT "/dvb/adapter%u/frontend%u", f->adapter, f->number);
		f->fd = frontend_open(path);
		f->open_error = f->fd < 0 ? errno : 0;
		free(path);
		if (f->fd < 0) record_error(f->open_error, false);
	}
	if (f->fd < 0) {
		read_timer(false);
		f->previous_valid = false;
		if (table) {
			char issue[192] = "-";
			if (f->open_error != EBUSY)
				snprintf(issue, sizeof(issue), "%s (%d)", strerror(f->open_error), f->open_error);
			fprintf(table_output, "%-18s %-11s %8s %-9s %-8s %-8s %-6s %s\n", node,
			       f->open_error == EBUSY ? "BUSY" : f->open_error == ETIMEDOUT ? "TIMEOUT" :
			       f->open_error == EAGAIN ? "RETRY" : "UNAVAILABLE", "n/a", "n/a", "n/a", "n/a", "n/a", issue);
		} else if (f->open_error == EBUSY) puts("    frontend busy (shared tuner in use; skipped)");
		else printf("    frontend unavailable: %s (%d)\n", strerror(f->open_error), f->open_error);
		return;
	}
	if (specialized && !f->profile_checked) {
		struct dvb_frontend_info info = {0};
		if (frontend_ioctl(f->fd, FE_GET_INFO, &info) == 0) {
			info.name[sizeof(info.name) - 1] = 0;
			f->module = frontend_module(info.name);
			f->profile_checked = true;
		}
	}
	fe_status_t status = 0;
	bool status_valid = frontend_ioctl(f->fd, FE_READ_STATUS, &status) == 0;
	int status_error = status_valid ? 0 : errno;
	struct dtv_property parameters[ARRAY_SIZE(parameter_cmds)] = {0};
	int errors[ARRAY_SIZE(parameter_cmds)];
	for (size_t i = 0; i < ARRAY_SIZE(parameters); i++) parameters[i].cmd = parameter_cmds[i];
	get_properties(f->fd, parameters, ARRAY_SIZE(parameters), errors);
	bool parameters_valid = true, changed = false;
	for (size_t i = 0; i < ARRAY_SIZE(parameters); i++) {
		if (errors[i]) { parameters_valid = false; continue; }
		unsigned value = parameters[i].u.data;
		if (value != f->parameters[i]) changed = true;
		f->parameters[i] = value;
	}
	struct dtv_property stats[ARRAY_SIZE(stat_cmds)] = {0};
	int stat_errors[ARRAY_SIZE(stat_cmds)];
	for (size_t i = 0; i < ARRAY_SIZE(stats); i++) stats[i].cmd = stat_cmds[i];
	get_properties(f->fd, stats, ARRAY_SIZE(stats), stat_errors);
	for (size_t i = 0; i < ARRAY_SIZE(stats); i++) stat_errors[i] = validate_stat(&stats[i], stat_errors[i]);
	legacy_stat(f->fd, &stats[0], &stat_errors[0], FE_READ_SIGNAL_STRENGTH, false);
	legacy_stat(f->fd, &stats[1], &stat_errors[1], FE_READ_SNR, false);
	legacy_stat(f->fd, &stats[6], &stat_errors[6], FE_READ_UNCORRECTED_BLOCKS, true);
	uint32_t legacy_ber = 0;
	int legacy_ber_error = 0;
	bool legacy_ber_valid = false;
	if (missing_stat(&stats[4], stat_errors[4]) && missing_stat(&stats[5], stat_errors[5])) {
		legacy_ber_valid = frontend_ioctl(f->fd, FE_READ_BER, &legacy_ber) == 0;
		if (!legacy_ber_valid && !unsupported_error(errno)) legacy_ber_error = errno;
	}
	/* Empty or disabled counters must not establish a BER baseline. */
	bool counters_valid = !stat_errors[4] && !stat_errors[5] &&
		stats[4].u.st.len && stats[5].u.st.len &&
		stats[4].u.st.stat[0].scale == FE_SCALE_COUNTER &&
		stats[5].u.st.stat[0].scale == FE_SCALE_COUNTER;
	read_timer(false);
	char diagnosis[512] = "";
	if (specialized) specialized_diagnosis(f, parameters, errors, stats, stat_errors, diagnosis, sizeof(diagnosis));
	bool locked = status_valid && (status & FE_HAS_LOCK) && !(status & (FE_REINIT | FE_TIMEDOUT));
	if (table) {
		char signal[48], cnr[48], blocks[48], frequency[32] = "n/a", ber[32] = "n/a", issue[256] = "";
		const char *state = !status_valid ? (status_error == ETIMEDOUT ? "TIMEOUT" :
			temporary_error(status_error) ? "RETRY" : "ERROR") : status & FE_REINIT ? "REINIT" :
			status & FE_TIMEDOUT ? "TIMEOUT" : status & FE_HAS_LOCK ? "LOCKED" : "UNLOCKED";
		table_error(issue, sizeof(issue), "status", status_error, false);
		for (size_t i = 0; i < ARRAY_SIZE(parameters); i++) table_error(issue, sizeof(issue), parameter_names[i], errors[i], true);
		for (size_t i = 0; i < ARRAY_SIZE(stats); i++) table_error(issue, sizeof(issue), stat_names[i], stat_errors[i], true);
		table_error(issue, sizeof(issue), "legacy_BER_raw", legacy_ber_error, true);
		table_stat(signal, sizeof(signal), &stats[0], stat_errors[0], 0);
		table_stat(cnr, sizeof(cnr), &stats[1], stat_errors[1], 1);
		table_stat(blocks, sizeof(blocks), &stats[6], stat_errors[6], 6);
		if (!errors[0] && !errors[1])
			snprintf(frequency, sizeof(frequency), "%.3f", parameters[1].u.data / (satellite(parameters[0].u.data) ? 1000.0 : 1000000.0));
		if (status_valid && (status & FE_REINIT)) snprintf(ber, sizeof(ber), "reinit");
		else if (legacy_ber_valid && locked) snprintf(ber, sizeof(ber), "raw:%" PRIu32, legacy_ber);
		else if (parameters_valid && locked && counters_valid) {
			if (!f->previous_valid) snprintf(ber, sizeof(ber), "baseline");
			else if (changed) snprintf(ber, sizeof(ber), "retune");
			else {
				uint64_t e, t;
				switch (counter_interval(f, &stats[4].u.st, &stats[5].u.st, 0, &e, &t)) {
				case BER_MISSING: break;
				case BER_RESET: snprintf(ber, sizeof(ber), "reset"); break;
				case BER_PAUSED: snprintf(ber, sizeof(ber), "no-bits"); break;
				case BER_VALID: snprintf(ber, sizeof(ber), "%.3g", (double)e / t); break;
				}
			}
		}
		fprintf(table_output, "%-18s %-11s %8s %-9s %-8s %-8s %-6s %s\n", node, state, frequency, signal, cnr, ber, blocks,
		        *issue ? issue : *diagnosis ? diagnosis : "-");
	} else {
		if (!status_valid) unavailable("status", status_error, false);
		else {
			printf("    status=0x%02x %s", status, status & FE_HAS_LOCK ? "LOCKED" : "UNLOCKED (may be idle)");
			const struct { unsigned bit; const char *name; } flags[] = {
				{ FE_HAS_SIGNAL, "SIGNAL" }, { FE_HAS_CARRIER, "CARRIER" }, { FE_HAS_VITERBI, "VITERBI" },
				{ FE_HAS_SYNC, "SYNC" }, { FE_TIMEDOUT, "TIMEDOUT" }, { FE_REINIT, "REINIT" },
			};
			for (size_t i = 0; i < ARRAY_SIZE(flags); i++) if (status & flags[i].bit) printf(" %s", flags[i].name);
			puts("");
		}
		for (size_t i = 0; i < ARRAY_SIZE(parameters); i++) {
			if (errors[i]) { unavailable(parameter_names[i], errors[i], true); continue; }
			unsigned value = parameters[i].u.data;
			printf("    %s=%u", parameter_names[i], value);
			if (i == 0) printf(" (%s)", delivery_name(value));
			if (i == 1) printf(" %s", !errors[0] && satellite(parameters[0].u.data) ? "kHz" : "Hz");
			if (i == 4 && value == QAM_256) printf(" (QAM256)");
			puts("");
		}
		for (size_t i = 0; i < ARRAY_SIZE(stats); i++) print_stat(&stats[i], i, stat_errors[i]);
		if (specialized) printf("    diagnosis=%s\n", diagnosis);
		if (legacy_ber_valid) printf("    legacy_BER_raw=%" PRIu32 " (driver-defined scale; not an interval ratio)\n", legacy_ber);
		else if (legacy_ber_error) unavailable("legacy_BER_raw", legacy_ber_error, true);
		if (status_valid && (status & FE_REINIT)) puts("    postBER_interval unavailable: frontend reinitialization");
		if (f->previous_valid && parameters_valid && locked && counters_valid) {
			if (changed) puts("    postBER_interval unavailable: tune changed");
			else interval_ber(f, &stats[4].u.st, &stats[5].u.st);
		}
	}
	f->post_error = stats[4].u.st;
	f->post_total = stats[5].u.st;
	f->previous_valid = parameters_valid && locked && counters_valid;
	close(f->fd); f->fd = -1;
}

static void present_table(const char *frame, size_t size)
{
	/* Keep the previous frame visible during device reads. Only overwrite
	 * after the complete next frame is ready, without blanking the screen.
	 * stdout is fully buffered here, so line rendering has no read delays.
	 */
	fputs("\033[H", stdout);
	const char *start = frame, *end = frame + size;
	for (const char *cursor = frame; cursor < end; cursor++) {
		if (*cursor != '\n') continue;
		fwrite(start, 1, (size_t)(cursor - start), stdout);
		fputs("\033[K\n", stdout);
		start = cursor + 1;
	}
	fwrite(start, 1, (size_t)(end - start), stdout);
	fputs("\033[J", stdout);
	fflush(stdout);
}

static void kernel_log(void)
{
	puts("\nKERNEL LOG (full current ring, including USB messages; not cleared):");
	int size = klogctl(10, NULL, 0);
	if (size < 0) { unavailable("kernel log (try sudo)", errno, false); return; }
	if (size > 64 * 1024 * 1024) { puts("    kernel log exceeds 64 MiB safety limit"); collection_errors++; return; }
	char *buffer = malloc((size_t)size + 1);
	if (!buffer) { perror("allocate kernel log"); collection_errors++; return; }
	int length = klogctl(3, buffer, size);
	if (length < 0) unavailable("kernel log (try sudo)", errno, false);
	else {
		buffer[length] = 0;
		fwrite(buffer, 1, length, stdout);
		if (length && buffer[length - 1] != '\n') puts("");
	}
	free(buffer);
}

static void usage(FILE *stream)
{
	fputs("Usage: dvb-check [--table] [--watch SECONDS] [--count N] [--kernel-log]\n"
	      "  Default: inventory and one read-only sample of every DVB frontend.\n"
	      "  -a, --adapter N  restrict reads to adapter N\n"
	      "  -f, --frontend N restrict reads to frontend N (within selected adapters)\n"
	      "  --read-timeout SECONDS  interrupt frontend waits after 0.01..60 seconds (default 0.25)\n"
	      "  --table          one status row per frontend; refresh capable terminals\n"
	      "  --specialized    explain chip-specific statistics, driver settings and PLP requests\n"
	      "  --watch SECONDS  repeat (0.1..3600 seconds); Ctrl+C ends capture\n"
	      "  --count N        stop after N samples; default unlimited with --watch\n"
	      "  --kernel-log     append full kernel ring at exit (usually needs root)\n"
	      "  USB tracing setup prompts before loading usbmon/mounting debugfs; no/EOF skips setup.\n"
	      "  --help           show this help\n"
	      "Exit: 0 collection completed (busy/timeouts may be skipped), 1 collection errors,\n"
	      "      2 no matching frontends/invalid arguments.\n", stream);
}

int main(int argc, char **argv)
{
	double interval = 1;
	unsigned long count = 1;
	bool watch = false, count_set = false, logs = false, table = false;
	unsigned selected_adapter = 0, selected_frontend = 0;
	bool adapter_set = false, frontend_set = false;
	const struct option options[] = {
		{ "watch", required_argument, NULL, 'w' }, { "count", required_argument, NULL, 'c' },
		{ "kernel-log", no_argument, NULL, 'k' }, { "help", no_argument, NULL, 'h' },
		{ "table", no_argument, NULL, 't' },
		{ "adapter", required_argument, NULL, 'a' },
		{ "frontend", required_argument, NULL, 'f' },
		{ "read-timeout", required_argument, NULL, 'r' },
		{ "specialized", no_argument, NULL, 's' },
		{ "version", no_argument, NULL, 'v' }, { NULL, 0, NULL, 0 },
	};
	int option;
	while ((option = getopt_long(argc, argv, "a:f:", options, NULL)) != -1) {
		char *end;
		switch (option) {
		case 'a':
		case 'f': {
			errno = 0;
			unsigned long value = strtoul(optarg, &end, 10);
			if (errno || !isdigit((unsigned char)*optarg) || *end || value > UINT_MAX) goto bad_arguments;
			if (option == 'a') { selected_adapter = value; adapter_set = true; }
			else { selected_frontend = value; frontend_set = true; }
			break;
		}
		case 'r':
			errno = 0; read_timeout = strtod(optarg, &end);
			if (errno || !*optarg || *end || !isfinite(read_timeout) || read_timeout < 0.01 || read_timeout > 60) goto bad_arguments;
			break;
		case 'w':
			errno = 0; interval = strtod(optarg, &end);
			if (errno || !*optarg || *end || !isfinite(interval) || interval < 0.1 || interval > 3600) goto bad_arguments;
			watch = true; break;
		case 'c':
			errno = 0; count = strtoul(optarg, &end, 10);
			if (errno || !isdigit((unsigned char)*optarg) || *end || !count) goto bad_arguments;
			count_set = true; break;
		case 'k': logs = true; break;
		case 't': table = true; break;
		case 's': specialized = true; break;
		case 'h': usage(stdout); return 0;
		case 'v': puts("dvb-check " VERSION); return 0;
		default: goto bad_arguments;
		}
	}
	if (optind != argc) goto bad_arguments;
	if (watch && !count_set) count = 0;
	const char *term = getenv("TERM");
	bool refresh = table && isatty(STDOUT_FILENO) && term && *term && strcmp(term, "dumb");
	struct sigaction action = { .sa_handler = interrupted };
	sigemptyset(&action.sa_mask);
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGHUP, &action, NULL);
	sigaction(SIGPIPE, &action, NULL);
	action.sa_handler = read_timeout_signal;
	if (sigaction(SIGALRM, &action, NULL) < 0) { perror("frontend timeout signal"); return 1; }
	if (atexit(cleanup_usbmon)) { fputs("Cannot register USB cleanup\n", stderr); return 1; }
	setvbuf(stdout, NULL, refresh ? _IOFBF : _IOLBF, 0);
	struct utsname host;
	puts("dvb-check " VERSION " (read-only DVB diagnostics)");
	if (!uname(&host)) printf("Host: %s %s %s\n", host.nodename, host.release, host.machine);
	puts("Physical tuner chip IDs may be not exposed by DVB/sysfs; frontend names are kernel-reported.\n"
	     "Multiple frontends may share a physical tuner; frontend count is not a physical tuner count.\n"
	     "No raw I2C probing, tuning or debug-flag changes. USB tracing setup requires approval. Lock is not proof of transport delivery.\n"
	     "A relative AGC reading is not calibrated RF power. Devices are inventoried at startup; restart after hotplug.");
	struct frontend *frontends = NULL;
	size_t total = discover(&frontends);
	size_t selected = 0;
	for (size_t i = 0; i < total; i++) {
		if (adapter_set && frontends[i].adapter != selected_adapter) continue;
		if (frontend_set && frontends[i].number != selected_frontend) continue;
		frontends[selected++] = frontends[i];
	}
	printf("Frontends discovered: %zu; selected: %zu; read timeout: %.3fs\n", total, selected, read_timeout);
	total = selected;
	for (size_t i = 0; i < total; i++) inventory(&frontends[i]);
	module_information();
	if (!total) puts("No matching DVB frontends found under /sys/class/dvb or /dev/dvb.");
	fflush(stdout);
	for (unsigned long sample = 0; total && !stopping && (!count || sample < count); sample++) {
		struct timespec now;
		struct tm utc;
		char stamp[64] = "time unavailable", date[32];
		if (!clock_gettime(CLOCK_REALTIME, &now) && gmtime_r(&now.tv_sec, &utc) &&
		    strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &utc))
			snprintf(stamp, sizeof(stamp), "%s.%03ldZ", date, now.tv_nsec / 1000000);
		char *frame = NULL;
		size_t frame_size = 0;
		FILE *output = refresh ? open_memstream(&frame, &frame_size) : stdout;
		if (!output) { perror("allocate table frame"); collection_errors++; break; }
		fprintf(output, "\nSAMPLE %lu %s\n", sample + 1, stamp);
		if (table)
			fprintf(output, "%-18s %-11s %8s %-9s %-8s %-8s %-6s %s\n", "FRONTEND", "STATUS", "MHz", "SIGNAL", "C/N", "postBER", "UCB", specialized ? "NOTE (errors first)" : "ERROR");
		for (size_t i = 0; i < total && !stopping; i++) sample_frontend(&frontends[i], table, output);
		if (table)
			fprintf(output, "Global layer (0); rel=relative AGC; !=check dBm calibration; UCB=reported error blocks.\n"
			       "Collection errors so far: %d. Detailed chip/layer information: run without --table.\n"
			       "Reads are sequential; lock is not proof of transport delivery. Ctrl+C stops.\n", collection_errors);
		if (refresh) {
			int error = fclose(output);
			if (!error && !stopping) present_table(frame, frame_size);
			free(frame);
			if (error) { perror("render table frame"); collection_errors++; break; }
		}
		if (count && sample + 1 == count) break;
		struct timespec delay = { .tv_sec = (time_t)interval, .tv_nsec = (long)((interval - (time_t)interval) * 1e9) };
		while (!stopping && nanosleep(&delay, &delay) < 0 && errno == EINTR) {}
	}
	if (logs) kernel_log();
	for (size_t i = 0; i < total; i++) if (frontends[i].fd >= 0) close(frontends[i].fd);
	free(frontends);
	cleanup_usbmon();
	printf("\nCollection errors: %d (exit status is not a tuner health verdict)\n", collection_errors);
	if (fflush(stdout) || ferror(stdout)) { perror("write report"); return 1; }
	return collection_errors ? 1 : total ? 0 : 2;
bad_arguments:
	usage(stderr);
	return 2;
}
