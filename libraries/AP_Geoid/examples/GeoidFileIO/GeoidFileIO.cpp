/*
  example file io for AP_Geoid

  requires the geoid data file egm96-5.pgm in directory ./geoids

  if GeographicLib is installed this may be found at:
  /usr/local/share/GeographicLib/geoids
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Geoid/Geoid.h>
#include <AP_Math/AP_Math.h>

#include <stdio.h>
#include <limits>

// set according to the data size in file
#define MAX_LINE_SIZE 256

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

/*
  initialise systems
*/
static AP_BoardConfig board_config;

int file_count = 0;
const char *dir_path = nullptr;
static void read_geoid_file(const char *);

/*
  sketch setup and loop
*/
void setup()
{
    board_config.init();

    // directory for geoid files
    dir_path = "./geoids";
    hal.console->printf("Creating directory - %s\n", dir_path);
    int ret = AP::FS().mkdir(dir_path);
    if (ret == -1) {
        if (errno == EEXIST) {
            hal.console->printf("Geoid directory - %s already exist\n", dir_path);
        } else {
            hal.console->printf("Failed to create directory %s - %s\n", dir_path, strerror(errno));
        }
    }

    // geoid file_path
    char file_path[100];
    snprintf(file_path, sizeof(file_path), "%s/egm96-5.pgm", dir_path);
    hal.console->printf("Geoid file_path: %s\n", file_path);

    // read geoid file
    read_geoid_file(file_path);
}

void loop()
{
    hal.scheduler->delay(1000);
}

/*
  read the geoid file header
*/
static void read_geoid_file(const char *file_path)
{
    char line[MAX_LINE_SIZE];

    // open file in read mode
    const int fd = AP::FS().open(file_path, O_RDONLY);
    if (fd == -1) {
        hal.console->printf("Open %s failed\n", file_path);
        return;
    }
    hal.console->printf("Open %s\n", file_path);

    // read line
    int line_count = 0;

    // check format
    if (AP::FS().fgets(line, sizeof(line)-1, fd)) {
        if (strcmp(line, "P5") != 0) {
            hal.console->printf("File not in PGM format %s\n", file_path);
            AP::FS().close(fd);
            return;
        }
        hal.console->printf("Line %d: %s\n", line_count, line);
        line_count++;
    }

    float offset = std::numeric_limits<float>::max();
    float scale = 0.0f;
    float maxerror = -1.0f;
    float rmserror = -1.0f;
    int width;
    int height;

    while (AP::FS().fgets(line, sizeof(line)-1, fd)) {
        // tokenise
        char *saveptr = nullptr;
        const char *comment_id_str = strtok_r(line, " ", &saveptr);
        if (comment_id_str == nullptr || strlen(comment_id_str) == 0) {
            continue;
        }

        // comments
        if (strcmp(comment_id_str, "#") == 0) {
            const char *key_str = strtok_r(nullptr, " ", &saveptr);
            if (key_str == nullptr || strlen(key_str) == 0) {
                continue;
            }

            if (strcmp(key_str, "Description") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    hal.console->printf("Description: %s\n", value_str);
                }
            }
            if (strcmp(key_str, "DateTime") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    hal.console->printf("DateTime: %s\n", value_str);
                }
            }
            if (strcmp(key_str, "Offset") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    offset = atof(value_str);
                    hal.console->printf("Offset: %f\n", offset);
                }
            }
            if (strcmp(key_str, "Scale") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    scale = atof(value_str);
                    hal.console->printf("Scale: %f\n", scale);
                }
            }
            if (strcmp(key_str, "MaxCubicError") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    maxerror = atof(value_str);
                    hal.console->printf("MaxCubicError: %f\n", maxerror);
                }
            }
            if (strcmp(key_str, "MaxBilinearError") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    maxerror = atof(value_str);
                    hal.console->printf("MaxBilinearError: %f\n", maxerror);
                }
            }
            if (strcmp(key_str, "RMSCubicError") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    rmserror = atof(value_str);
                    hal.console->printf("RMSCubicError: %f\n", rmserror);
                }
            }
            if (strcmp(key_str, "RMSBilinearError") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    rmserror = atof(value_str);
                    hal.console->printf("RMSBilinearError: %f\n", rmserror);
                }
            }
        } else {
            // read raster size
            width = atoi(comment_id_str);
            hal.console->printf("width: %d\n", width);

            const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
            if (value_str != nullptr || strlen(value_str) > 0) {
                height = atoi(value_str);
                hal.console->printf("height: %d\n", height);
            }
            break;
        }
        line_count++;
    }

    // read maxval
    unsigned maxval;
    hal.console->printf("Reading maxval...\n");
    while (AP::FS().fgets(line, sizeof(line)-1, fd)) {
        if (strlen(line) != 0) {
            maxval = atol(line);
            hal.console->printf("maxval: %d\n", maxval);
            break;
        }
    }
    
    // set start of data block
    hal.console->printf("Setting start of data block...\n");
    int32_t datastart = AP::FS().lseek(fd, 0, SEEK_CUR);
    hal.console->printf("datastart: %d\n", int(datastart));

    // check file size
    //! @note this will always fail when using the egm96-5-header.pgm example.
    int32_t pixel_size_ = 2;
    int32_t swidth = width;
    int32_t sheight = height;
    int32_t file_end = AP::FS().lseek(fd, 0, SEEK_END);
    if (datastart + pixel_size_ * swidth * sheight != file_end) {
        hal.console->printf("File has the wrong length %s\n", file_path);
    }
    hal.console->printf("file length expected: %d\n", int(datastart + pixel_size_ * swidth * sheight));
    hal.console->printf("file length actual:   %d\n", int(file_end));

    // close file after reading
    AP::FS().close(fd);
}

AP_HAL_MAIN();
