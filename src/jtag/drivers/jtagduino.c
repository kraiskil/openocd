/* Device driver for the 'jtagduino', Arduino as a JTAG progammer.
 * https://github.com/balau/JTAGduino
 */ 

#include "config.h"
#include "bitbang.h"
#include "jtag/interface.h"
#include <termios.h>
#include <unistd.h>

/* From JTAGduino.ino. Keep these in sync! */
enum jtagduino_cmd {
  CMD_IF_VER = 0x1,
  CMD_FW_VER = 0x2,
  CMD_SET_SERIAL_SPEED = 0x3,
  
  CMD_SET_PIN = 0x10,
  CMD_CLEAR_PIN = 0x11,
  CMD_GET_PIN = 0x12,
  CMD_PULSE_HIGH = 0x13,
  CMD_PULSE_LOW = 0x14,
  CMD_ASSIGN_PIN = 0x15,
  
  CMD_SET_JTAG_SPEED = 0x20,
  CMD_JTAG_CLOCK = 0x21,
  CMD_JTAG_SEQUENCE = 0x22,
  CMD_LED = 0x23,
};

enum jtagduino_rsp {
  RSP_OK = 0,
  RSP_ERROR_BAD_CMD = 1,
  RSP_ERROR_UNKNOWN = 2,
  RSP_ERROR_BAD_PIN = 3,
  RSP_ERROR_BAD_SPEED = 4,
  RSP_BAD_SEQUENCE_LEN = 5,
  RSP_BAD_BAUD = 6,
};

#if 0
enum jtagduino_constants {
  MAX_RSP_LEN = 1 + JTAG_MAX_SEQUENCE_LEN_BYTES,
  MAX_CMD_LEN = 1 + 1 + JTAG_MAX_SEQUENCE_LEN_BYTES + JTAG_MAX_SEQUENCE_LEN_BYTES,
  IF_VER_MAJOR = 0,
  IF_VER_MINOR = 1,
  FW_VER_MAJOR = 0,
  FW_VER_MINOR = 1,
  N_BAUD_RATES = 11,
};
#endif

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
	char data;
	int rv = read(jtagduino_serdev_fd, &data, 1);
	LOG_DEBUG("read %x (read %s)", data, rv==1?"OK":"FAIL");
	
	return data;
}
static void jtagduino_bitbang_write(int tck, int tms, int tdi)
{
	LOG_DEBUG("jtagduino: write: tck = %d, tms = %d, tdi = %d\n", tck, tms, tdi);
	if(tck)
	{
		char data = CMD_JTAG_SEQUENCE;
		write(jtagduino_serdev_fd, &data, 1);
		data = 1; //lenght of sequence
		write(jtagduino_serdev_fd, &data, 1);
		data = tms;
		write(jtagduino_serdev_fd, &data, 1);
		data = tdi;
		write(jtagduino_serdev_fd, &data, 1);
	}
}
static void jtagduino_bitbang_reset(int trst, int srst)
{
	LOG_DEBUG("jtagduino: reset\n");
}
static void jtagduino_bitbang_led(int on)
{
	LOG_DEBUG("jtagduino: LED %s", on?"ON":"OFF");
	char data=CMD_LED;
	write(jtagduino_serdev_fd, &data, 1);
	data = !!on;
	write(jtagduino_serdev_fd, &data, 1);
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
