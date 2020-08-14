#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <string.h>

#include "utils.h"
#include "config.h"


static char _helpstr[] =
"%s: Neoway module helper\n\n" \
"This software is used to handle requests to various Neoway module subsystems\n\n"
"Usage: %s [args], where args is:\n"
"--daemonize, -d\n"
"        Become a daemon, start in background. Default: run in foreground\n"
"--pidfile=<path>, -p <path>\n"
"        Specify path for the PID file. Default: %s\n"
"--config=<path>, -c <path>\n"
"        Specify path for the configuration file. Default: %s\n"
"--kud=<address>[:port]\n"
"        Set KUD server address and port\n"
"--uart=<devname>[:<baudrate>], -u <devname>[:<baudrate>]\n"
"        Set UART device name. Default: /dev/ttyHSL0:115200\n"
"--modem=<devname>[:<baudrate>], -m <devname>[:<baudrate>]\n"
"        Set UART device name for communicating with modem. Default: /dev/ttyHS0:115200\n"
"--help, -h\n"
"       Print this message\n"
"\n";


void
options_init(options_t *o)
{

    o->baud_rate = DEFAULT_UART_BAUD_RATE;
    o->modem_baud_rate = DEFAULT_MODEM_BAUD_RATE;
    o->pid_file = strdup(PID_FILE);
    o->kud_address = strdup(GATEWAY_ADDRESS);
    o->uart_tty = strdup(DEFAULT_UART_TTY);
    o->modem_tty = strdup(DEFAULT_MODEM_TTY);
    o->go_daemon = FALSE;
    o->broadcast_addr = strdup(BROADCAST_ADDRESS);
    o->broadcast_period = BROADCAST_PERIOD;
    o->modem_fd = -1;
    o->uart_fd = -1;
    
    o->gps_enabled = TRUE;
    o->gprs_enabled = TRUE;

    pthread_mutex_init(&o->mutex, NULL);

    /*--web interface--*/
    //o->rssi[10] = strdup(NULL);
    memset(o->rssi, 0, sizeof(o->rssi));
    memset(o->threed_fix, 0, sizeof(o->threed_fix));
    memset(o->imei, 0, sizeof(o->imei));
    memset(o->imsi, 0, sizeof(o->imsi));
    memset(o->reg_in_mesh, 0, sizeof(o->reg_in_mesh));
    memset(o->country_cod, 0, sizeof(o->country_cod));
    memset(o->operator_cod, 0, sizeof(o->operator_cod));
    memset(o->gps_cords, 0, sizeof(o->gps_cords));
    memset(o->mobile_data, 0, sizeof(o->mobile_data));
    memset(o->up_time_string, 0, sizeof(o->up_time_string));
    memset(o->power_type, 0, sizeof(o->power_type));
    memset(o->last_mileage, 0, sizeof(o->last_mileage));
    memset(o->carrige_mileage, 0, sizeof(o->carrige_mileage));
}


void
options_cleanup(options_t *o)
{
    free(o->pid_file);
    free(o->uart_tty);
    free(o->modem_tty);
}


int
write_pid(options_t *o, pid_t pid)
{
    FILE *pidf;

    pidf = fopen(o->pid_file, "w");
    if(NULL != pidf) {
        fprintf(pidf, "%d", pid);
        fclose(pidf);
        return 0;
    }

    syslog(LOG_ERR, "Failed to write PID file %s: %s", o->pid_file, strerror(errno));
    return -1;
}


static void
redirect_file_descriptors(options_t *opts)
{
    char *tty;
    struct rlimit rlim;
    int fd, maxfd;

    getrlimit(RLIMIT_NOFILE, &rlim);
    maxfd = rlim.rlim_max;
    if(maxfd == RLIM_INFINITY)
        maxfd = MAXFD;

    // Close all TTYs except our UART
    for(fd=0; fd<maxfd; ++fd) {
        tty = ttyname(fd);
        if(NULL != tty) {
            if(0 != strcmp(tty, opts->uart_tty)) {
                close(fd);
            }
        }
    }

    fd = open("/dev/null", O_RDWR);

    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDIN_FILENO);
    dup2(fd, STDERR_FILENO);
}


/* This is our variant of daemon() syscall. Unlike stdlib's daemon() function
 * it implements so-called "double fork()" techinque to avoid becoming
 * a session leader.
 */
int
daemonize(options_t *opts, int noClose)
{
    pid_t pid;

    // Fork once to go into the background.
    syslog(LOG_DEBUG, "Forking first child");
    pid = fork();
    if(0 != pid) {
        // Parent. Exit using _exit(), which doesn't fire any atexit
        // functions.
        _exit(EXIT_SUCCESS);
    }

    // First child. Create a new session. setsid() creates the session
    // and makes this (child) process the process group leader. The process
    // is guaranteed not to have a control terminal.
    syslog(LOG_DEBUG, "Creating new session");
    pid = setsid();

    // Fork a second child to ensure that the daemon never reacquires
    // a control terminal.
    syslog(LOG_DEBUG, "Forking second child");
    pid = fork();
    if(0 != pid) {
        // Original child. Exit.
        _exit(EXIT_SUCCESS);
    }

    write_pid(opts, pid);

    // This is the second child. Set the umask.
    syslog(LOG_DEBUG, "Setting umask");
    umask(UMASK);

    // Go to a neutral corner (i.e., the primary file system, so
    // the daemon doesn't prevent some other file system from being
    // unmounted).
    syslog(LOG_DEBUG, "Changing working directory to \"%s\"", WORKDIR);
    chdir(WORKDIR);

    // Unless noClose was specified, close all file descriptors.
    if(!noClose) {
        syslog(LOG_DEBUG, "Redirecting file descriptors");
        redirect_file_descriptors(opts);
        // TODO
    }

    syslog(LOG_INFO, "Switched to daemon mode");
    return 0;
}


void
help(options_t *opts)
{
    (void)opts;
    fprintf(stdout, _helpstr, APPNAME, APPNAME);
    _exit(EXIT_SUCCESS);
}


int
get_UUID(unsigned char *uuid)
{
    struct ifreq ifr;
    struct ifconf ifc;
    char buf[1024];
    int sock;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if(sock == -1) { /* handle error*/
        return -1;
    }

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) {
        close(sock);
        return -2;
    }

    struct ifreq* it = ifc.ifc_req;
    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

    for(; it != end; ++it) {
        strcpy(ifr.ifr_name, it->ifr_name);
        if(ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
            if(!(ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
                if(ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
                    memcpy(uuid, ifr.ifr_hwaddr.sa_data, 6);
                    break;
                }
            } // --
        } else {
            close(sock);
            return -3;
        }
    }

    close(sock);
    return (it == end)? -4: 0;
}

