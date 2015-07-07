/* Device driver for the 'jtagduino', Arduino as a JTAG progammer.
 * https://github.com/balau/JTAGduino
 */ 

#include "config.h"
#include "bitbang.h"
#include "jtag/interface.h"
#include <termios.h>
#include <unistd.h>


static char *jtagduino_serdev_name = "/dev/ttyACM0"; 
static int jtagduino_serdev_fd = -1;
 
/* jtagduino utilizes previous generic code from bitbang.c.
 * This struct records the callbacks to jtagduino
 * that the bitbang.c implementation uses. 
 */
static int jtagduino_bitbang_read(void);
static void jtagduino_bitbang_write(int tck, int tms, int tdi);
static void jtagduino_bitbang_reset(int trst, int srst);
static void jtagduino_bitbang_led(int on);
static struct bitbang_interface jtagduino_bitbang = {
	.read = &jtagduino_bitbang_read,
	.write = &jtagduino_bitbang_write,
	.reset = &jtagduino_bitbang_reset,
	.blink = &jtagduino_bitbang_led
};

COMMAND_HANDLER(jtagduino_handle_jtagduino_port_command)
{
	if (CMD_ARGC == 1) {
		jtagduino_serdev_name = malloc(strlen(CMD_ARGV[0]) + sizeof(char));
		if(jtagduino_serdev_name)
			strcpy(jtagduino_serdev_name, CMD_ARGV[0]);
	}
	//command_print(CMD_CTX, "parport port = 0x%" PRIx16 "", parport_port);

	return ERROR_OK;
}

static const struct command_registration jtagduino_command_handlers[] = {
	{
		/* TODO: is it correct to prefix the device name here? */
		.name = "jtagduino_port",
		.handler = jtagduino_handle_jtagduino_port_command, 
		.mode = COMMAND_CONFIG,
		.help = "set the port. Also document this better",
		.usage = "RTFS"
	},
	COMMAND_REGISTRATION_DONE
};

/* With this struct, jtagduino registers itself as a driver
 * to the openocd proper.
 */
static int jtagduino_init(void);
static int jtagduino_quit(void);
static int jtagduino_khz(int, int *);
static int jtagduino_speed_div(int speed, int *khz);
static int jtagduino_speed(int speed);
struct jtag_interface jtagduino_interface = {
	.name = "jtagduino",
	.supported = DEBUG_CAP_TMS_SEQ, /* the only capability listed in jtag.h */
	.commands = jtagduino_command_handlers,
	.init = jtagduino_init,
	.quit = jtagduino_quit,
	.khz  = jtagduino_khz,
	.speed_div = jtagduino_speed_div,
	.speed = jtagduino_speed,
	.execute_queue = bitbang_execute_queue
};


static int jtagduino_bitbang_read(void)
{
	LOG_DEBUG("jtagduino: read\n");
	return ERROR_OK;
}
static void jtagduino_bitbang_write(int tck, int tms, int tdi)
{
	LOG_DEBUG("jtagduino: write: tck = %d, tms = %d, tdi = %d\n", tck, tms, tdi);
}
static void jtagduino_bitbang_reset(int trst, int srst)
{
	LOG_DEBUG("jtagduino: reset\n");
}
static void jtagduino_bitbang_led(int on)
{
	LOG_DEBUG("jtagduino: led\n");
}

static int jtagduino_init(void)
{
	struct termios term_io;

	LOG_DEBUG("Opening serial device %s", jtagduino_serdev_name);
	assert(jtagduino_serdev_fd=-1);
	jtagduino_serdev_fd = open(jtagduino_serdev_name, O_RDWR | O_NOCTTY);
	if(jtagduino_serdev_fd < 0) {
		LOG_ERROR("Cannot open jtagduino serial device at path %s", jtagduino_serdev_name);
		/* TODO: would openocd guidlines allow this?:
		perror("when opening serial port device"); */
		return ERROR_JTAG_INIT_FAILED;
	}
	tcgetattr(jtagduino_serdev_fd, &term_io);
	/*TODO: 9600 is the default value for JTAGduino. It allows chaning them, perhaps we should too?*/
	cfsetispeed(&term_io, B9600);
	cfsetospeed(&term_io, B9600);
	term_io.c_cflag |= CLOCAL | CREAD;
	/* TODO: walk through the rest of the termios flags, asserting they are correct */
	tcsetattr(jtagduino_serdev_fd, TCSAFLUSH, &term_io);

	/* Confusing? bitbang.c defines a global 'bitbang_interface' of type 'struct bitbang_interface*'. */
	bitbang_interface = &jtagduino_bitbang;
	return ERROR_OK;
}
static int jtagduino_quit(void)
{
	LOG_DEBUG("jtagduino: quit\n");
	return ERROR_OK;
}
static int jtagduino_khz(int khz, int *jtag_speed)
{
	LOG_DEBUG("jtagduino_khz(%d, ..)", khz);
	return ERROR_OK;
}
static int jtagduino_speed_div(int speed, int *khz)
{
	LOG_DEBUG("jtagduino: speed_div(%d)", speed);
	return ERROR_OK;
}
static int jtagduino_speed(int speed)
{
	LOG_DEBUG("jtagduino: speed\n");
	return ERROR_OK;
}
