#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>

#define GPS_PORT "/dev/serial0"
#define BUFFER_SIZE 1024

int configure_uart(int fd) {
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    return tcsetattr(fd, TCSANOW, &options);
}

void convert_utc_to_ist(const char *utc_time, char *ist_time) {
    int hours, minutes;
    double seconds;
    sscanf(utc_time, "%2d%2d%lf", &hours, &minutes, &seconds);

    minutes += 30;
    hours += 5;
    if (minutes >= 60) {
        minutes -= 60;
        hours += 1;
    }
    if (hours >= 24) {
        hours -= 24;
    }

    snprintf(ist_time, 11, "%02d:%02d:%02.0f", hours, minutes, seconds);
}

void parse_and_display_location(const char *buffer) {
    char *sentence = strstr(buffer, "$GPRMC");
    if (sentence) {
        char utc_time[11], latitude[15], ns[2], longitude[15], ew[2], ist_time[11];
        if (sscanf(sentence, "$GPRMC,%10[^,],A,%14[^,],%1[^,],%14[^,],%1[^,]", 
                   utc_time, latitude, ns, longitude, ew) == 5) {
            
            double lat = atof(latitude);
            double lon = atof(longitude);
            int lat_deg = (int)(lat / 100);
            int lon_deg = (int)(lon / 100);
            double lat_min = lat - (lat_deg * 100);
            double lon_min = lon - (lon_deg * 100);

            double lat_decimal = lat_deg + (lat_min / 60.0);
            double lon_decimal = lon_deg + (lon_min / 60.0);

            if (ns[0] == 'S') lat_decimal = -lat_decimal;
            if (ew[0] == 'W') lon_decimal = -lon_decimal;

            convert_utc_to_ist(utc_time, ist_time);

            printf("IST Time: %s\n", ist_time);
            printf("Latitude: %d° %.5f' %s\n", lat_deg, lat_min, ns);
            printf("Longitude: %d° %.5f' %s\n", lon_deg, lon_min, ew);
            printf("Decimal Degrees: %.5f, %.5f\n", lat_decimal, lon_decimal);
        }
    }
}

int main() {
    char buffer[BUFFER_SIZE];
    int fd = open(GPS_PORT, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        perror("Failed to open UART");
        return 1;
    }

    if (configure_uart(fd) != 0) {
        perror("Failed to configure UART");
        close(fd);
        return 1;
    }

    while (1) {
        int bytes_read = read(fd, buffer, BUFFER_SIZE - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            parse_and_display_location(buffer);
        }
        usleep(1000000);
    }

    close(fd);
    return 0;
}