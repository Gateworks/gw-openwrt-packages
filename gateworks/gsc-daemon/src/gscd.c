// vi: set ai ts=2 sts=2 sw=2 et cin:
/** 
 * @file gscd.c
 * @brief Gateworks System Controller deamon
 * switches and scripts
 * @author Harold Hall <hhall@gateworks.com>
 * @author Tim Harvey <tharvey@gateworks.com>
 * @date 9/12/2012
 * @version 1.0
 *
 * This daemon monitors the Gateworks System Controller (GSC) host interrupt
 * (using poll(2) which waits on an interrupt vs polling) and emits hotplug
 * events.
 *
 * The default hotplug subsystem used is 'button' (but can be specified with
 * the '--subsystem' param.  The same env variables that are passed via the
 * OpenWrt button-hotplug kernel driver (see
 *  https://dev.openwrt.org/browser/trunk/package/button-hotplug) are used:
 *    ACTION=%s - event type
 *        pressed|released|held - button state
 *        high|low|still_high|still_low - GPIO state
 *    SEEN=%d   - number of seconds passed since last event of this type
 *    BUTTON=%s - name of button/signal/event
 *
 * In addition the following env vars are added:
 *    COUNT=%d  - number of events of this type occuring without 1s passing
 *                (ie ACTION=released SEEN=0 COUNT=3 BUTTON=USER_PB means
 *                 that the USER_PB has been pressed and released 3 times
 *                 with less than 1s in between)
 *
 */

#define DEBUG

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <getopt.h>
#include <errno.h>

#include "i2c-dev.h"

#ifdef DEBUG
#define DPRINTF(fmt, ...) if (debug) { fprintf(stdout, "DEBUG: " fmt, ## __VA_ARGS__); }
#else
#define DPRINTF(lvl, fmt, ...)
#endif

/*
 * GSC Address/Register/Bit definitions (see http://trac.gateworks.com/wiki/gsc)
 */
#define PCA_I2C_ADDR      0x23 // PCA9555 7-bit address
#define GSC_I2C_ADDR      0x20 // GSC 7-bit address

#define GSC_INT_SOURCE    10   // interrupt source reg
#define GSC_INT_ENABLE    11   // interrupt enable reg

/* interrupt source/enable bits */
#define GSC_INT_PB        0    // pushbutton short press-release
#define GSC_INT_ERASE     1    // EEPROM erase complete
#define GSC_INT_GPIO      4    // GPIO
#define GSC_INT_TAMPER    5    // Tamper event
#define GSC_INT_PB_LONG   7    // pushbutton long hold event

/* PCA955x registers */
#define PCA_INP0          0    // PORT0 input
#define PCA_DIR0          6    // PORT0 config
#define PCA_INP1          1    // PORT1 input
#define PCA_DIR1          7    // PORT1 config

static int debug = 0;

/** get_gpio - get state of GSC GPIO Ports
 * returns Port0 input states in LSB and Port1 input states in MSB
 */
unsigned short get_gpio(int fd)
{
  unsigned short res = 0;

  if (ioctl(fd, I2C_SLAVE_FORCE, PCA_I2C_ADDR) < 0) {
      perror("I2C_SLAVE_FORCE");
      exit(-3);
  }
  res = i2c_smbus_read_byte_data(fd, PCA_INP0) & 
        i2c_smbus_read_byte_data(fd, PCA_DIR0);
  res |= (i2c_smbus_read_byte_data(fd, PCA_INP1) & 
          i2c_smbus_read_byte_data(fd, PCA_DIR1)) << 8;

  return res;
}

/** clear_interrupt - clear interrupt status
 */
int clear_interrupt(int fd, unsigned int mask) {
  if (ioctl(fd, I2C_SLAVE_FORCE, GSC_I2C_ADDR) < 0) {
    perror("I2C_SLAVE_FORCE");
    return -1;
  }
  if (i2c_smbus_write_byte_data(fd, GSC_INT_SOURCE, ~mask) == -1) {
    perror("i2c_smbus_write_byte_data failed");
    return -1;
  }
  return 0;
}

/** hotplug_event - call hotlpug subsystem
 */
static int
hotplug_event(char* type, char *envp[])
{
  pid_t pid;
  char *argv[] = { "/sbin/hotplug-call", type, (char *) 0};
  
  DPRINTF("%s: %s %s %s %s %s\n", __func__, type, envp[0], envp[1], envp[2], envp[3]);
  if ((pid = fork()) == 0) {
    int ret = execve(argv[0], argv, envp);
    DPRINTF("%d: launched call returned %d\n", getpid(), ret);
    if (ret < 0)
      perror("hotplug call failed");
    _exit(0);
  }
  return pid;
}

/** display usage and exit
 */
void usage(const char* app)
{
  fprintf(stderr, "usage: %s [OPTIONS] <i2cbus> <irq_gpio>\n"
      "\n"
      " Options:\n"
      "    --pb_gpio <n>    - the GPIO of the user pushbutton\n"
      "    --pb_name <str>  - the name to emit for user pushbutton\n"
      "    --nochange_mask  - a bitmask of GPIO values to emit nochange events for\n"
      "    -d, --debug      - enable debugging\n"
    , app);

  exit(1);
}


/** main
 */
int main(int argc, char** argv)
{
  int i2c_fd, gpio_fd;
  time_t event_time;          // track event time delta
  time_t irq_time[8];
  time_t gpio_time[16];
  int gpio_count[16];         // track gpio counts
  unsigned short last_gpio;   // track gpio states
  unsigned short gpio;
  struct pollfd fdset[1];     // for poll(2)
  char buf[256];              // general purpose buffer
  char *env[5];
  int i, c;

  // options
  int i2c_bus = 0;            // i2c bus device (typically /dev/i2c0)
  int gsc_irq = -1;           // host interrupt gpio
  int pb_gpio = -1;           // pushbutton gpio
  int nochange_mask = 0;      // bitmask of gpios we eimt 'nochange' events for
  char *pb_name = "USER_PB";  // pushbutton name
  char *subsystem= "button";  // hotplug subsystem;
  int timeout = 0;            // timeout in seconds (period of nochange events)

  // parse commandline options
  while (1) {
    int optind = 0;
    static struct option long_opts[] = {
      {"debug", 0, &debug, 'd'},
      {"pb_gpio", 1, 0, 0},
      {"pb_name", 1, 0, 0},
      {"subsystem", 1, 0, 0},
      {"nochange_mask", 1, 0, 0},
      {"nochange_period", 1, 0, 0},
      {0, 0, 0, 0}
    };
  
    c = getopt_long(argc, argv, "dh?", long_opts, &optind);
    if (c == -1)
      break;
  
    switch (c) {
    case 0:
      if (strcmp("pb_gpio", long_opts[optind].name) == 0) {
        pb_gpio = atoi(optarg);
      } else if (strcmp("pb_name", long_opts[optind].name) == 0) {
        pb_name = optarg; 
      } else if (strcmp("nochange_mask", long_opts[optind].name) == 0) {
        nochange_mask = strtoul(optarg, NULL, 0); 
      } else if (strcmp("nochange_period", long_opts[optind].name) == 0) {
        timeout = atoi(optarg); 
      } else if (strcmp("subsystem", long_opts[optind].name) == 0) {
        subsystem = optarg; 
      } else {
        fprintf(stderr, "unknown option\n");
        usage(argv[0]);
      }
      break;

    case 'd':
      debug++;
      break;

    default:
      usage(argv[0]);
      break;
    }
  }

  // process remaining required commandline params
  if (argc - optind >= 2) {
      i = argc - optind;
      i2c_bus = atoi(argv[optind++]);
      gsc_irq = atoi(argv[optind++]);
  }

  // validate options
  if (i2c_bus == -1 || gsc_irq == -1) {
    fprintf(stderr, "invalid configuration\n");
    usage(argv[0]);
  }

  //open main i2c kernel device so we can communicate with GSC over it
  snprintf(buf, sizeof(buf)-1, "/dev/i2c-%d", i2c_bus);
  if ((i2c_fd = open(buf, O_RDWR)) < 0) {
    fprintf(stderr, "Failed to open %s: %s\n", buf, strerror(errno));
    exit(-1);
  }

  // open /sys/class gpio file that we will be polling for interrupts
  snprintf(buf, sizeof(buf)-1, "/sys/class/gpio/gpio%d/edge",
      gsc_irq);
  if ((gpio_fd = open(buf, O_WRONLY)) < 0) {
    fprintf(stderr, "invalid gsc_irq:%d %s - %s\n", gsc_irq, buf,
        strerror(errno));
    exit(-2);
  }
  sprintf(buf, "falling");
  write(gpio_fd, buf, strlen(buf));
  close(gpio_fd);
  snprintf(buf, sizeof(buf)-1, "/sys/class/gpio/gpio%d/value",
      gsc_irq);
  if ((gpio_fd = open(buf, O_RDONLY | O_NONBLOCK)) < 0) {
    fprintf(stderr, "invalid gsc_irq:%d %s - %s\n", gsc_irq, buf,
        strerror(errno));
    exit(-2);
  }

  printf("%s: monitoring gpio%d for GSC interrupt\n", argv[0], gsc_irq);
  if (pb_gpio != -1) {
    printf(" and pushbutton on gpio%d (bit%d)\n", pb_gpio, pb_gpio-100);
    pb_gpio -= 100; // turn into bit
    nochange_mask |= 1<<pb_gpio;
  }
  if (timeout)
    printf(" and nochange events every %d seconds\n", timeout);

  //This read clears out the data from the value file so that we only get
  // interrupts
  read(gpio_fd, buf, sizeof(buf));

  //initialize gpio states and last state change time
  for (i = 0; i < 4; i++)
      env[i] = (char*) malloc(32);
  env[4] = 0;
  for (i = 0; i < 16; i++) {
    gpio_time[i] = time(NULL);
    gpio_count[i] = 0;
  }
  for (i = 0; i < 8; i++)
    irq_time[i] = time(NULL);
  last_gpio = get_gpio(i2c_fd);
  DPRINTF("GPIO:%04x\n", last_gpio);

  // clear GSC interrupts 
  clear_interrupt(i2c_fd, 0xff);

  while (1) {
    unsigned short changed;
    unsigned char source;

    /* wait for change in gpio value or timeout */
    memset((void*)fdset, 0, sizeof(fdset));
    fdset[0].fd = gpio_fd;
    fdset[0].events = POLLPRI;
    if ((c = poll(fdset, 1, timeout?(timeout*1000):10000)) < 0) {
      perror("poll() failed");
      break;
    }
    DPRINTF("poll() returned %d\n", c);

    /* grab event data */
    if (c > 0 || timeout) {
      event_time = time(NULL);
      gpio = get_gpio(i2c_fd);
      changed = gpio ^ last_gpio; //gpios that changed their states
      DPRINTF("GPIO:%04x/%04x/%04x\n", gpio, last_gpio, changed);
      last_gpio = gpio;
    }
/*
    if (fdset[0].revents & POLLNVAL) {
      read(fdset[0].fd, buf, sizeof(buf));
      fprintf(stderr,"\npoll() GPIO %d interrupt invalid\n", 1);
    }
*/
    if (fdset[0].revents & POLLPRI) {
      int rz;
      lseek(fdset[0].fd, 0, SEEK_SET);
      rz = read(fdset[0].fd, buf, sizeof(buf));
      buf[rz?rz-1:0] = 0;
      DPRINTF("read %d bytes: %s\n", rz, buf);
    }

    /* timeout occurred
     */
    if (c == 0) {
      if (!timeout)
        continue;
      DPRINTF("timeout\n");
      // emulate a GSC interrupt to handle 'nochange' event
      source = 1<<GSC_INT_GPIO;
    } // end timeout

    /* GSC Interrupt has occured - determine source and emit event
     */
    else {
      //set i2c bus contoller to send to the proper slave address
      if (ioctl(i2c_fd, I2C_SLAVE_FORCE, GSC_I2C_ADDR) < 0) {
        perror("I2C_SLAVE_FORCE");
        break;
      }

      source = i2c_smbus_read_byte_data(i2c_fd, GSC_INT_SOURCE);
      //source &= i2c_smbus_read_byte_data(i2c_fd, GSC_INT_ENABLE);
      DPRINTF("GSC interrupt: 0x%02x\n", source);

      // clear all interrupts 
      clear_interrupt(i2c_fd, 0xff);
    }

    // iterate over various IRQ bits
    for (i = 0; i < 8; i++) {
      if (source & (1<<i)) {
        sprintf(env[0], "SEEN=%d", (int) (event_time - irq_time[i]));
        sprintf(env[3], "COUNT=");
        irq_time[i] = event_time;
        switch (i) {
        // GPIO change
        case GSC_INT_GPIO: {
          int j;

          // iterate over all GSC GPIO bits
          for (j = 0; j < 16; j++) {
            // emit events for GPIO's that have changed
            if (changed & 1<<j) {
              int seen = (int) (event_time - gpio_time[j]);
              gpio_time[j] = event_time;
              if (seen < 2)
                gpio_count[j]++;
              else
                gpio_count[j] = 0;
              sprintf(env[0], "SEEN=%d", seen);
              sprintf(env[1], "COUNT=%d", gpio_count[j]);
              if (pb_gpio != -1 && j == pb_gpio) {
                sprintf(env[2], "BUTTON=%s", pb_name);
                sprintf(env[3], "ACTION=%s",
                    (gpio & 1<<j)?"released":"pressed");
              } else {
                sprintf(env[2], "BUTTON=GPIO%d", 100 + j);
                sprintf(env[3], "ACTION=%s", (gpio & 1<<j)?"high":"low");
              }
              hotplug_event(subsystem, env);
            }

            // emit events for GPIO's that have not changed if timeout/mask
            else if (nochange_mask & 1<<j && timeout) {
              int seen = (int) (event_time - gpio_time[j]);
              gpio_time[j] = event_time;
              if (seen < 2)
                gpio_count[j]++;
              else
                gpio_count[j] = 0;
              sprintf(env[0], "SEEN=%d", seen);
              sprintf(env[1], "COUNT=%d", gpio_count[j]);
              if (pb_gpio != -1 && j == pb_gpio) {
                if (gpio & 1<<j) {
                  // don't bother emitting 'unheld' events for pushbutton
                  continue;
                } else {
                  sprintf(env[2], "BUTTON=%s", pb_name);
                  sprintf(env[3], "ACTION=held");
                }
              } else {
                sprintf(env[2], "BUTTON=GPIO%d", 100 + j);
                sprintf(env[3], "ACTION=%s",
                    (gpio & 1<<j)?"still_high":"still_low");
              }
              hotplug_event(subsystem, env);
            }
          }
        } break;

        // Pushbutton quick press and release event
        case GSC_INT_PB:
          sprintf(env[1], "BUTTON=%s", pb_name);
          sprintf(env[2], "ACTION=pressed");
          hotplug_event(subsystem, env);
          break;

        // Pushbutton held > 700ms event
        case GSC_INT_PB_LONG:
          sprintf(env[1], "BUTTON=%s", pb_name);
          sprintf(env[2], "ACTION=held");
          hotplug_event(subsystem, env);
          break;

        // EEPROM Erase complete event
        case GSC_INT_ERASE:
          sprintf(env[1], "BUTTON=erase");
          sprintf(env[2], "ACTION=complete");
          hotplug_event(subsystem, env);
          break; 

        // Tamper event
        case GSC_INT_TAMPER:
          sprintf(env[1], "BUTTON=tamper");
          sprintf(env[2], "ACTION=asserted");
          hotplug_event(subsystem, env);
          break; 
        } 
      }
    } // for all interrupt status bits
  }

  close(i2c_fd);
  close(gpio_fd);
  return 1;
}
