/**
 * @file main.c
 *
 * @brief Main Entry point for the CY3240 i2c test
 *
 * Main Entry point for the CY3240 i2c test
 *
 * @ingroup CY3240
 *
 * @owner  Kevin Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin Kirkup (kevin.kirkup@gmail.com)
 */
//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h> /* for getopt() */
#include <time.h>
#include <hidapi/hidapi.h>

#include "cy3240.h"
#include "cy3240_util.h"
#include "i2c_demo.h"

#define SLEEP_BETWEEN_CMD   (250000)

void print_buffer(const uint8_t* const buffer, uint16_t length) {
    printf("\nBuffer: (Length=%i)", length);

    for (int count = 0; count < length; count ++) {

        if(count % 16 == 0) {
            printf("\n0%d: ", (int)(count / 16));
        }

        if(count % 4 == 0) {
            printf(" %02x",buffer[count]);

        } else {
            printf("%02x",buffer[count]);
        }
    }

    printf("\n");
}


typedef struct __attribute__((packed)) {
	uint16_t soilResistance;
    int16_t temperature;
    uint16_t humidity;
    uint16_t illuminance;        
} sensor_data;


int main(int argc, char *argv[]) {
    int iface_num = 0;
    int retry;

    void *handle = 0;

    uint16_t length = 0;

    sensor_data sensorData;

    // Initialize the device
    cy3240_factory(
            &handle,
            iface_num,
            50,
            CY3240_POWER_3_3V,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    // Open the device
    cy3240_open(handle);

	usleep(SLEEP_BETWEEN_CMD);

    // Configure the bridge controller using the default settings
    cy3240_reconfigure(
            handle,
            CY3240_POWER_3_3V,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz);

	usleep(SLEEP_BETWEEN_CMD);

    length = sizeof(sensorData);
	
	while (true) {
		cy3240_read(handle, 0x08, (uint8_t*) &sensorData, &length);

		// print_buffer(data, length);

		fprintf(stdout, "data: %d %.2f %.2f %d\n",
				sensorData.soilResistance, sensorData.temperature / 100.0,
				sensorData.humidity / 10.0, sensorData.illuminance);

		usleep(SLEEP_BETWEEN_CMD);
	}

    // Configure the bridge controller using the default settings
    cy3240_reconfigure(
            handle,
            CY3240_POWER_5V,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz);

    usleep(SLEEP_BETWEEN_CMD);

    // Close the device
    cy3240_close(handle);

    return 0;
}
