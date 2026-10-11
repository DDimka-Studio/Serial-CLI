#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#define MAX_PATH_LEN 256
#define MAX_RESPONSE 2048

static const int COMMON_BAUD_RATES[] = { 9600, 115200, 57600, 38400, 19200, 4800, 230400 };
static const int COMMON_BAUD_COUNT = (int)(sizeof(COMMON_BAUD_RATES) / sizeof(COMMON_BAUD_RATES[0]));

static speed_t get_termios_baud(int baud) {
    switch (baud) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return (speed_t)0; /* invalid/unsupported */
    }
}

static int find_auto_device(char *out_path, size_t max_len) {
    DIR *dir = opendir("/dev");
    if (!dir) {
        fprintf(stderr, "Cannot open /dev: %s\n", strerror(errno));
        return 0;
    }

    struct dirent *entry;
    char acm_fallback[MAX_PATH_LEN] = {0};
    char usb_found[MAX_PATH_LEN] = {0};
    int usb_count = 0, acm_count = 0;

    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "ttyUSB", 6) == 0) {
            if (usb_count == 0)
                snprintf(usb_found, sizeof(usb_found), "/dev/%s", entry->d_name);
            usb_count++;
        } else if (strncmp(entry->d_name, "ttyACM", 6) == 0) {
            if (acm_count == 0)
                snprintf(acm_fallback, sizeof(acm_fallback), "/dev/%s", entry->d_name);
            acm_count++;
        }
    }
    closedir(dir);

    if (usb_count > 0) {
        if (usb_count > 1)
            fprintf(stderr, "Warning: %d ttyUSB devices, using %s\n", usb_count, usb_found);
        strncpy(out_path, usb_found, max_len - 1);
        out_path[max_len - 1] = '\0';
        return 1;
    }
    if (acm_count > 0) {
        if (acm_count > 1)
            fprintf(stderr, "Warning: %d ttyACM devices, using %s\n", acm_count, acm_fallback);
        strncpy(out_path, acm_fallback, max_len - 1);
        out_path[max_len - 1] = '\0';
        return 1;
    }
    return 0;
}

static ssize_t read_until_quiet(int fd, unsigned char *buf, size_t max_len,
                                double total_timeout_sec, int quiet_ms) {
    ssize_t total = 0;
    struct timeval deadline, now, remaining, quiet_tv;

    gettimeofday(&deadline, NULL);
    deadline.tv_sec  += (time_t)total_timeout_sec;
    deadline.tv_usec += (suseconds_t)((total_timeout_sec - (time_t)total_timeout_sec) * 1e6);
    if (deadline.tv_usec >= 1000000) {
        deadline.tv_sec++;
        deadline.tv_usec -= 1000000;
    }
    if (deadline.tv_usec < 0) {
        deadline.tv_sec--;
        deadline.tv_usec += 1000000;
    }

    quiet_tv.tv_sec  = quiet_ms / 1000;
    quiet_tv.tv_usec = (quiet_ms % 1000) * 1000;

    while (total < (ssize_t)(max_len - 1)) {
        gettimeofday(&now, NULL);
        if (now.tv_sec > deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec && now.tv_usec >= deadline.tv_usec))
            break;

        remaining.tv_sec  = deadline.tv_sec  - now.tv_sec;
        remaining.tv_usec = deadline.tv_usec - now.tv_usec;
        if (remaining.tv_usec < 0) {
            remaining.tv_sec--;
            remaining.tv_usec += 1000000;
        }

        struct timeval wait = remaining;
        if (total > 0) {
            if (quiet_tv.tv_sec < wait.tv_sec ||
                (quiet_tv.tv_sec == wait.tv_sec && quiet_tv.tv_usec < wait.tv_usec))
                wait = quiet_tv;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int sel = select(fd + 1, &rfds, NULL, NULL, &wait);
        if (sel < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (sel == 0) {
            if (total > 0) break;
            continue;
        }

        ssize_t n = read(fd, buf + total, max_len - 1 - (size_t)total);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            break;
        }
        if (n == 0) break;
        total += n;
    }

    if (total > 0) buf[total] = '\0';
    return total;
}

static int looks_like_text(const unsigned char *buf, ssize_t n) {
    if (n <= 0) return 0;
    ssize_t good = 0;
    for (ssize_t i = 0; i < n; i++) {
        unsigned char c = buf[i];
        if ((c >= 32 && c < 127) || c == '\r' || c == '\n' || c == '\t')
            good++;
    }
    return good * 10 >= n * 9;
}

static void drain_input(int fd, double timeout_sec) {
    unsigned char tmp[256];
    read_until_quiet(fd, tmp, sizeof(tmp), timeout_sec, 60);
    tcflush(fd, TCIFLUSH);
}

static int set_baud(int fd, int baud_rate) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    speed_t speed = get_termios_baud(baud_rate);
    if (speed == (speed_t)0) {
        errno = EINVAL;
        return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= (CS8 | CREAD | CLOCAL);
    tty.c_cflag &= ~HUPCL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | BRKINT | INPCK | ISTRIP);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    tcflush(fd, TCIOFLUSH);
    return 0;
}

static int open_port(const char *device_path, int keep_dtr, double reset_wait_sec, int verbose) {
    int fd = open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "open(%s): %s\n", device_path, strerror(errno));
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        fprintf(stderr, "fcntl(F_GETFL): %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
        fprintf(stderr, "fcntl(F_SETFL): %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    if (set_baud(fd, 9600) != 0) {
        fprintf(stderr, "set_baud(9600): %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    {
        int status = 0;
        if (ioctl(fd, TIOCMGET, &status) == 0) {
            if (keep_dtr)
                status |= (TIOCM_DTR | TIOCM_RTS);
            else
                status &= ~(TIOCM_DTR | TIOCM_RTS);
            if (ioctl(fd, TIOCMSET, &status) != 0 && verbose)
                fprintf(stderr, "ioctl(TIOCMSET): %s\n", strerror(errno));
        } else if (verbose) {
            fprintf(stderr, "ioctl(TIOCMGET): %s\n", strerror(errno));
        }
    }

    tcflush(fd, TCIOFLUSH);

    if (reset_wait_sec > 0.0) {
        if (verbose)
            fprintf(stderr, "Waiting %.1f s after open (Arduino reset)...\n", reset_wait_sec);
        usleep((useconds_t)(reset_wait_sec * 1e6));
        tcflush(fd, TCIFLUSH);
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        fprintf(stderr, "fcntl(F_SETFL): %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

static int try_command(int fd, int baud, const char *command, const char *line_ending,
                       double timeout_sec, unsigned char *out_buf, size_t max_out_len,
                       size_t *out_len, int verbose) {
    if (set_baud(fd, baud) != 0) {
        fprintf(stderr, "set_baud(%d): %s\n", baud, strerror(errno));
        return 0;
    }

    if (verbose)
        fprintf(stderr, "Trying %d baud...\n", baud);

    if (command && command[0]) {
        /* Sync: terminate any partial line left in the device buffer
           (garbage from earlier runs / wrong baud), discard its reply. */
        if (line_ending && line_ending[0]) {
            if (write(fd, line_ending, strlen(line_ending)) > 0)
                tcdrain(fd);
            drain_input(fd, 0.25);
        }

        size_t cmd_len = strlen(command);
        ssize_t written = write(fd, command, cmd_len);
        if (written < 0 || (size_t)written != cmd_len) {
            fprintf(stderr, "write: %s\n", strerror(errno));
            return 0;
        }
        if (line_ending && line_ending[0]) {
            size_t le = strlen(line_ending);
            written = write(fd, line_ending, le);
            if (written < 0 || (size_t)written != le) {
                fprintf(stderr, "write ending: %s\n", strerror(errno));
                return 0;
            }
        }
        tcdrain(fd);
    }

    usleep(20000);

    ssize_t n = read_until_quiet(fd, out_buf, max_out_len, timeout_sec, 60);
    if (n > 0 && looks_like_text(out_buf, n)) {
        if (verbose)
            fprintf(stderr, "OK @ %d baud (%zd bytes)\n", baud, n);
        if (out_len) *out_len = (size_t)n;
        return 1;
    }
    if (n > 0 && verbose)
        fprintf(stderr, "Garbage @ %d baud, ignoring\n", baud);
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options] [COMMAND]\n"
        "\n"
        "  -d DEVICE     /dev/ttyUSB0 or /dev/ttyACM0 (default: auto)\n"
        "  -b BAUD       fixed baud (skip probing)\n"
        "  -e ENDING     \\r\\n (default), \\r, \\n, none\n"
        "  -w SECONDS    wait after open for reset (default: 2.0)\n"
        "  --no-dtr      clear DTR/RTS after open\n"
        "  -v            verbose\n"
        "  -h            help\n"
        "\n"
        "Default command: \"WA D9 255\"\n"
        "Baud order: 9600 first, then 115200, ...\n"
        "Port is opened once; baud is changed without re-open (fast).\n",
        prog);
}

int main(int argc, char *argv[]) {
    int custom_baud = 0;
    const char *cmd = "WA D9 255";
    const char *device_arg = NULL;
    const char *ending = "\r\n";
    double reset_wait = 2.0;
    int keep_dtr = 1;
    int verbose = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            custom_baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            device_arg = argv[++i];
        } else if (strcmp(argv[i], "-e") == 0 && i + 1 < argc) {
            i++;
            if (strcmp(argv[i], "none") == 0)
                ending = "";
            else if (strcmp(argv[i], "\\r") == 0 || strcmp(argv[i], "\r") == 0)
                ending = "\r";
            else if (strcmp(argv[i], "\\n") == 0 || strcmp(argv[i], "\n") == 0)
                ending = "\n";
            else if (strcmp(argv[i], "\\r\\n") == 0 || strcmp(argv[i], "\r\n") == 0)
                ending = "\r\n";
            else
                ending = argv[i];
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            reset_wait = atof(argv[++i]);
            if (reset_wait < 0) reset_wait = 0;
        } else if (strcmp(argv[i], "--no-dtr") == 0) {
            keep_dtr = 0;
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = 1;
        } else {
            cmd = argv[i];
        }
    }

    char device[MAX_PATH_LEN];
    if (device_arg) {
        strncpy(device, device_arg, sizeof(device) - 1);
        device[sizeof(device) - 1] = '\0';
    } else if (!find_auto_device(device, sizeof(device))) {
        fprintf(stderr, "Error: no ttyUSB/ttyACM found\n");
        return 1;
    }

    if (verbose) {
        fprintf(stderr, "Device: %s\n", device);
        fprintf(stderr, "Command: \"%s\"\n", cmd);
    }

    int fd = open_port(device, keep_dtr, reset_wait, verbose);
    if (fd < 0)
        return 1;

    unsigned char response_buf[MAX_RESPONSE];

    int ok = 0;
    size_t resp_len = 0;
    if (custom_baud > 0) {
        for (int attempt = 0; attempt < 2 && !ok; attempt++) {
            memset(response_buf, 0, sizeof(response_buf));
            ok = try_command(fd, custom_baud, cmd, ending, 1.2,
                             response_buf, sizeof(response_buf), &resp_len, verbose);
        }
    } else {
        for (int i = 0; i < COMMON_BAUD_COUNT && !ok; i++) {
            /* first baud gets a retry: board may still be booting */
            int attempts = (i == 0) ? 2 : 1;
            for (int a = 0; a < attempts && !ok; a++) {
                memset(response_buf, 0, sizeof(response_buf));
                ok = try_command(fd, COMMON_BAUD_RATES[i], cmd, ending, 0.7,
                                 response_buf, sizeof(response_buf), &resp_len, verbose);
            }
        }
    }

    close(fd);

    if (ok) {
        fwrite(response_buf, 1, resp_len, stdout);
        if (resp_len > 0 && response_buf[resp_len - 1] != '\n')
            putchar('\n');
        return 0;
    }

    fprintf(stderr, "Error: no response from %s\n", device);
    fprintf(stderr, "Try: %s -b 9600 -v\n", argv[0]);
    fprintf(stderr, "  or: %s -w 2.0 -b 9600\n", argv[0]);
    return 1;
}

