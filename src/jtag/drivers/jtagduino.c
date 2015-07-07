/* Device driver for the 'jtagduino', Arduino as a JTAG progammer.
 * https://github.com/balau/JTAGduino
 */ 


#include "config.h"
#include "bitbang.h"
#include "jtag/interface.h"

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

static const struct command_registration jtagduino_command_handlers[] = {
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
	LOG_DEBUG("init jtagduino\n");
	LOG_OUTPUT("output log\n\n");

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
	LOG_DEBUG("jtagduino: khz\n");
	return ERROR_OK;
}
static int jtagduino_speed_div(int speed, int *khz)
{
	LOG_DEBUG("jtagduino: speed_div\n");
	return ERROR_OK;
}
static int jtagduino_speed(int speed)
{
	LOG_DEBUG("jtagduino: speed\n");
	return ERROR_OK;
}
