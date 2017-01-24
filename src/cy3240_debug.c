/**
 * @file cy3240_debug.c
 *
 * @brief CY3240 debug functions and macros
 *
 * CY3240 debug functions and macros
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
#include "cy3240.h"
#include "cy3240_packet.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{

/**
 * The number of bytes to print per line
 */
#define BYTES_PER_LINE      (16)

/**
 * The number of bytes to group when printing
 */
#define BYTE_SEGMENTS       (4)

//@} End of Defines

//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Method to print the USB packet
 *
 *  @param buffer[in] the receive buffer to print
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_debug_print_packet(
        const uint8_t* const buffer,
        uint16_t length
        )
{
    // Check the input parameters
    if ((buffer != NULL) &&
        (length != 0)) {

        int count;

        // Iterate through the entire receive buffer
        for (count = 0; count < length; count++) {

            // Add a line number
            if (count % BYTES_PER_LINE == 0)
                printf("\n0%d: ", (int)(count / BYTES_PER_LINE));

            // Add a space between every the segments
            if (count % BYTE_SEGMENTS == 0)
                printf(" %02x",buffer[count]);

            else
                printf("%02x",buffer[count]);
        }

        // Add a newline at the end
        printf("\n");
        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
/**
 *  Method to print out the sending packet to the USB device
 *
 *  @param buffer [in] The send buffer
 *  @param length [in] The length of the send buffer
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_debug_print_send_packet(
        const uint8_t* buffer,
        uint16_t length
        )
{
    if ((buffer != NULL) &&
        (length != 0)) {

    	buffer = buffer + 1;
        int count;

        printf("\nSending packet: (Length=%i)", length);

        /* Not worrying about code size right now...still in debug mode */
        if(buffer[0] & CONTROL_BYTE_I2C_READ)
            printf("\nReading ");
        else {
            printf("\nWriting ");

			if(buffer[0] & CONTROL_BYTE_START)
				printf("a start condition ");

			if(buffer[0] & CONTROL_BYTE_RESTART)
				printf("a restart condition ");

			if(buffer[0] & CONTROL_BYTE_STOP)
				printf("a stop condition ");

			if(buffer[0] & CONTROL_BYTE_REINIT)
				printf("a reinitialization command ");

			if(buffer[0] & CONTROL_BYTE_RECONFIG)
				printf("a reconfiguration command ");
        }

        // Check the current bus configuration
        switch(buffer[0] & CY3240_BUS_MASK) {

            case CY3240_BUS_I2C:
                printf("to the I2C bus ");
                break;

          case CY3240_BUS_SPI:
                printf("to the SPI bus(invalid?) ");
                break;

          case CY3240_BUS_UART:
                printf("to the UART(invalid?) ");
                break;

          case CY3240_BUS_LIN:
                printf("to the LIN bus(invalid?) ");
                break;
        }

        printf("with length %d at address 0x%02x\nRaw output:",
                buffer[1],
                buffer[2]);

        // Print the packet
        return cy3240_debug_print_packet(buffer, length);
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
/**
 *  Method to print to information in the received control packet from the
 *  CY3240
 *
 *  @param packet [in] the control packet
 *  @param length [in] the length of the control packet
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_debug_print_receive_control_packet(
        const uint8_t* const packet,
        uint16_t length
        )
{
    if ((packet != NULL) &&
        (length != 0)) {

        int count;

        printf("\nReceived packet: (Length=%i)", length);

        if (packet[0] & 0x04)
            printf("\nThe device is powered");

        if (packet[0] & 0x02)
            printf("\nAn interrupt was received");

        for (count = 0; count < length; count ++) {

            if(count % BYTES_PER_LINE == 0)
                printf("\n0%d: ", (int)(count / BYTES_PER_LINE));

            if(count % BYTE_SEGMENTS == 0)
                printf(" %02x",packet[count]);

            else
                printf("%02x",packet[count]);
        }

        printf("\n");
        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//@} End of Methods
