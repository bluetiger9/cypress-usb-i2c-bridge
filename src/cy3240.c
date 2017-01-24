/**
 * @file test_cypress.c
 *
 * @brief The Cypress CY3240 USB-I2C Bridge
 *
 * The Cypress CY3240 USB-I2C Bridge
 *
 * @ingroup USB-I2C
 *
 * @owner  Kevin Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin Kirkup (kevin.kirkup@gmail.com)
 */

/* COPYRIGHT --
 *
 * This file is part of libhid, a user-space HID access library.
 * libhid is (c) 2003-2005
 *   Martin F. Krafft <libhid@pobox.madduck.net>
 *   Charles Lepple <clepple@ghz.cc>
 *   Arnaud Quette <arnaud.quette@free.fr> && <arnaud.quette@mgeups.com>
 * and distributed under the terms of the GNU General Public License.
 * See the file ./COPYING in the source distribution for more information.
 *
 * THIS PACKAGE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES
 * OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <hidapi/hidapi.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include "config.h"
#include "cy3240.h"
#include "cy3240_types.h"
#include "cy3240_private_types.h"
#include "cy3240_debug.h"
#include "cy3240_packet.h"

//@} End of Includes


//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{

#define SEND_PACKET_LEN (65)
#define RECV_PACKET_LEN (64)

/* Result Macros */
#define HID_SUCCESS(s)  ((s == 0) ? TRUE : FALSE)
#define HID_FAILURE(s)  ((s != 0) ? TRUE : FALSE)

#define MIN(a,b) (((a)<(b))?(a):(b))

//@} End of Defines

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

// The sending buffer
char SEND_PACKET[SEND_PACKET_LEN] = { 0x00,
    0x03, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// The receive buffer
char RECV_PACKET[RECV_PACKET_LEN] = {
    0x03, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Mutex to lock concurrent access to the bridge chip
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//@} End of Data


//////////////////////////////////////////////////////////////////////
/// @name Private Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Method to Transmit and Receive a packet from the CY3240
 *
 *  @param pCy3240        [in] the Cypress 3240 status structure
 *  @param pSendData      [in] the data to send
 *  @param pSendLength    [in] the length of the send data
 *  @param pReceiveData   [out] the data received from the Cypress 3240
 *  @param pReceiveLength [out] the length of the received data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
transcieve(
        const Cy3240_t* const pCy3240,
        const uint8_t* const pSendData,
        const uint16_t* const pSendLength,
        uint8_t* const pReceiveData,
        uint16_t* const pReceiveLength
        )
{
    if ((pCy3240 != NULL) &&
        (pSendData != NULL) &&
        (pSendLength != NULL) &&
        (*pSendLength != 0) &&
        (pReceiveData != NULL) &&
        (pReceiveLength != NULL) &&
        (*pReceiveLength != 0)) {

        int error = 0;

        CY3240_DEBUG_PRINT_TX_PACKET(pSendData, *pSendLength);

        // Write the data to the USB HID device
        error = hid_write(
                pCy3240->pHid,
                pSendData,
                SEND_PACKET_LEN);

        if (error == -1) {
            fprintf(stderr, "hid_write failed with return code %d\n", error);
            return CY3240_ERROR_HID;
        }

        // Read the response data from the USB HID device
        error = hid_read_timeout(
                pCy3240->pHid,
                pReceiveData,
                RECV_PACKET_LEN,
                pCy3240->timeout);

        if (error == -1) {
            fprintf(stderr, "hid_get_input_report failed with return code %d\n", error);
            return CY3240_ERROR_HID;

        } else if (error == 0) {
            fprintf(stderr, "no bytes read\n");
            return CY3240_ERROR_HID;
        }

        CY3240_DEBUG_PRINT_RX_PACKET(pReceiveData, *pReceiveLength);

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_HID;
}


//-----------------------------------------------------------------------------
/**
 *  Method to pack the packet to change the power mode for the bridge controller
 *
 *  @param pCy3240 [in] the CY3240 state
 *  @param pLength [out] the length of the reconfigure data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
pack_reconfigure_power(
        Cy3240_Power_t power,
        uint16_t* const pLength
        )
{
    // Check the parameters
    if (pLength != NULL) {
		memset(SEND_PACKET, 0u, 64);

        // Initialize the byte index
        uint8_t byteIndex = 1;

        SEND_PACKET[byteIndex++] = CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_START;
        SEND_PACKET[byteIndex++] = 0;

        // Set the I2C address of the CY3240 control register
        SEND_PACKET[byteIndex++] = CONTROL_I2C_ADDRESS;

        // Set the power mode to use
        SEND_PACKET[byteIndex] = power;

        // Set the length
        *pLength = byteIndex + 1;

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function pack_reconfigure_power  ----- */

//-----------------------------------------------------------------------------
/**
 *  Method to pack the reconfigure the clock speed for the bridge controller
 *
 *  @param pCy3240 [in] the CY3240 state
 *  @param pLength [out] the length of the reconfigure data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
pack_reconfigure_clock(
        Cy3240_I2C_ClockSpeed_t clock,
        uint16_t* const pLength
        )
{
    // Check the parameters
    if (pLength != NULL) {
		memset(SEND_PACKET, 0u, 64);

        // Initialize the byte index
        uint8_t byteIndex = 1;

        SEND_PACKET[byteIndex++] = CONTROL_BYTE_RECONFIG | clock;
        SEND_PACKET[byteIndex++] = LENGTH_BYTE_LAST_PACKET;

        // Set the I2C address of the CY3240 control register
        SEND_PACKET[byteIndex++] = CONTROL_I2C_ADDRESS;

        // Set the length
        *pLength = byteIndex + 1;

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function pack_reconfigure_clock ----- */


//-----------------------------------------------------------------------------
/**
 *  Method to pack the restart the bridge controller
 *
 *  @param pLength [out] the length of the restart data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
pack_restart(
        uint16_t* const pLength
        )
{
    if (pLength != NULL) {
		memset(SEND_PACKET, 0u, 64);

        // Initialize the byte index
        uint8_t byteIndex = 1;

        SEND_PACKET[byteIndex++] = CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_RESTART;
        SEND_PACKET[byteIndex++] = LENGTH_BYTE_LAST_PACKET;

        // Set the control address
        SEND_PACKET[byteIndex++] = CONTROL_I2C_ADDRESS;

        // Set the length
        *pLength = byteIndex + 1;

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function pack_restart  ----- */

//-----------------------------------------------------------------------------
/**
 *  Method to pack the re-initialize the bridge controller
 *
 *  @param pLength [out] the length of the reinit data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
pack_reinit(
        uint16_t* const pLength
        )
{
    if (pLength != NULL) {
		memset(SEND_PACKET, 0u, 64);

        // Initialize the byte index
        uint8_t byteIndex = 1;

        SEND_PACKET[byteIndex++] = CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_REINIT;
        SEND_PACKET[byteIndex++] = LENGTH_BYTE_LAST_PACKET;

        // Set the control address
        SEND_PACKET[byteIndex++] = CONTROL_I2C_ADDRESS;

        // Set the length
        *pLength = byteIndex + 1;

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function pack_reinit  ----- */


//-----------------------------------------------------------------------------
/**
 *  Method to pack a data write input packet
 *
 *  @param address     [in] the I2C address of the target
 *  @param pSendData   [in] the data to send
 *  @param pSendLength [in] the length of the data to send
 *  @param first       [in] is this the first packet
 *  @param more        [in] are there more packets
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
pack_write_input(
        uint8_t address,
        const uint8_t* const pSendData,
        uint16_t* const pSendLength,
        bool first,
        bool more
        )
{
    // Check parameters
    if ((pSendData != NULL) &&
        (pSendLength != NULL) &&
        (*pSendLength != 0)) {
				memset(SEND_PACKET, 0u, 64);
        // Initialize the byte index
        uint8_t byteIndex = 1;

        SEND_PACKET[byteIndex++] = CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_START;
        SEND_PACKET[byteIndex++] = (uint8_t)*pSendLength;

        // Check to see if this is the last packet
        if (more)
             SEND_PACKET[INPUT_PACKET_INDEX_LENGTH] |= LENGTH_BYTE_MORE_PACKETS;

        else
             SEND_PACKET[INPUT_PACKET_INDEX_CMD] |= CONTROL_BYTE_STOP;

        // If this is the first packet, we need to send the address
        if (first)
             SEND_PACKET[byteIndex++] = address;

        // Copy the data in to the send buffer
        memcpy(&SEND_PACKET[byteIndex], pSendData, *pSendLength);

        // Update the length to include the header bytes
        *pSendLength += byteIndex;

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function pack_write  ----- */

//-----------------------------------------------------------------------------
/**
 *  Method to decode the write output packet
 *
 *  @param pWriteLength [in] the number of bytes written
 *  @param pReadLength  [in] the number of bytes read
 *  @param pBytesLeft   [out] the number of bytes left to send
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
unpack_write_output(
        const uint16_t* const pWriteLength,
        const uint16_t* const pReadLength,
        uint16_t* const pBytesLeft
        )
{
    // Check the parameters
    if ((pWriteLength != NULL) &&
        (*pWriteLength != 0) &&
        (pReadLength != NULL) &&
        (*pReadLength != 0) &&
        (RECV_PACKET[OUTPUT_PACKET_INDEX_STATUS] != 0x00)) {

        int x = 0;

        // Loop through the pack acknowledgments
        for (x = OUTPUT_PACKET_INDEX_STATUS + 1; x < *pReadLength; x++) {

            DBG(printf("RECV_PACKET[%i]=%02x\n", x, RECV_PACKET[x]);)

            // Check for ack
            if (RECV_PACKET[x] == TX_ACK) {

                // Decrement the number of remaining bytes
                if (pBytesLeft != NULL) {
                     *pBytesLeft = *pBytesLeft - 1;
                }

            // Nack
            } else {
                printf(" Nack: %i\n", x);
                return CY3240_ERROR_TX;
            }
        }

        // Finished
        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}


//-----------------------------------------------------------------------------
/**
 *  Method to pack the read input packet
 *
 *  @param address     [in] the I2C address of the device to read
 *  @param pReadLength [in] the length of bytes to read
 *  @param first       [in] is this the first read
 *  @param more        [in] is there more data to read
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
pack_read_input (
        uint8_t address,
        uint16_t* const pReadLength
        )
{
    // Check the parameters
    if ((pReadLength != NULL) &&
        (*pReadLength != 0)) {
				memset(SEND_PACKET, 0u, 64);
        uint8_t byteIndex = 1;

        SEND_PACKET[byteIndex++] = CONTROL_BYTE_I2C_READ | CONTROL_BYTE_START | CONTROL_BYTE_STOP;
        SEND_PACKET[byteIndex++] = (uint8_t)*pReadLength;

        // We need to send the address
        SEND_PACKET[byteIndex++] = address;

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function pack_read_packet  ----- */


//-----------------------------------------------------------------------------
/**
 *  Method to unpack the read output packet data
 *
 *  @param pData   [out] the buffer to read the data into
 *  @param pLength [in] the amount of data to read
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
unpack_read_output (
        uint8_t* const pData,
        const uint16_t* const pLength
        )
{
    // Check the parameters
    if ((pData != NULL) &&
        (pLength != NULL) &&
        (*pLength != 0) &&
        (RECV_PACKET[OUTPUT_PACKET_INDEX_STATUS] != 0x00)) {

        // Copy the data from the receive buffer
        memcpy(pData, &RECV_PACKET[OUTPUT_PACKET_INDEX_DATA], *pLength);

        return CY3240_ERROR_OK;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}   /* -----  end of static function unpack_read_output  ----- */

//-----------------------------------------------------------------------------
/**
 * Method to reconfigure the power mode for the CY3240 bridge chip
 *
 * @param pCy3240 [in] the bridge state inforamtion
 * @param power   [in] the power mode to set
 * @return Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
reconfigure_power(
        Cy3240_t* const pCy3240,
        Cy3240_Power_t power
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    uint16_t writeLength = 0;
    uint16_t readLength = 0;

    result = pack_reconfigure_power(
            power,
            &writeLength);

    if CY3240_FAILURE(result)
        printf("Failed to pack send data in write input packet: %i\n", result);

    if (CY3240_SUCCESS(result)) {

        readLength = writeLength + CY3240_STATUS_CODE_SIZE;

        // Write the data to the buffer
        // Note! The received data is ignored
        result = transcieve(
                pCy3240,
                SEND_PACKET,
                &writeLength,
                RECV_PACKET,
                &readLength);

        if CY3240_FAILURE(result)
            printf("Failed to transmit write packet\n");
    }

    // Set the power mode
    if CY3240_SUCCESS(result)
        pCy3240->power = power;

    return result;
}

//-----------------------------------------------------------------------------
/**
 * Method to set the clock rate for the CY3240 bridge chip
 *
 * @param pCy3240 [in] the bridge state inforamtion
 * @param clock   [in] the clock rate to set
 * @return Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
static Cy3240_Error_t
reconfigure_clock(
        Cy3240_t* const pCy3240,
        Cy3240_I2C_ClockSpeed_t clock
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    uint16_t writeLength = 0;
    uint16_t readLength = 0;

    result = pack_reconfigure_clock(
            clock,
            &writeLength);

    if CY3240_FAILURE(result)
        printf("Failed to pack send data in write input packet: %i\n", result);

    if (CY3240_SUCCESS(result)) {

        readLength = writeLength + CY3240_STATUS_CODE_SIZE;

        // Write the data to the buffer
        // Note! The received data is ignored
        result = transcieve(
                pCy3240,
                SEND_PACKET,
                &writeLength,
                RECV_PACKET,
                &readLength);

        if CY3240_FAILURE(result)
            printf("Failed to transmit packet\n");
    }

    // Set the clock rate
    if CY3240_SUCCESS(result)
        pCy3240->clock = clock;

    return result;
}

//@} End of Private Methods


//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_restart(
        void *handle
        )
{
    // The handle is the pointer to the state structure
    const Cy3240_t* pCy3240 = (const Cy3240_t*)handle;

    // Check the parameters
    if (pCy3240 != NULL) {

        Cy3240_Error_t result = CY3240_ERROR_OK;

        uint16_t writeLength = 0;
        uint16_t readLength = 0;

        pthread_mutex_lock(&mutex);

        // TODO: Check the state machine
        // Construct the message
        if CY3240_SUCCESS(result) {

            result = pack_restart(
                    &writeLength);

            if CY3240_FAILURE(result)
                printf("Failed to pack restart data in write input packet: %i\n", result);
        }

        // Send the message
        if (CY3240_SUCCESS(result)) {

            readLength = writeLength + CY3240_STATUS_CODE_SIZE;

            // Write the data to the buffer
            // Note! The received data is ignored
            result = transcieve(
                    pCy3240,
                    SEND_PACKET,
                    &writeLength,
                    RECV_PACKET,
                    &readLength);

            if CY3240_FAILURE(result)
                printf("Failed to transmit restart packet\n");
        }

        // Decode the response
        if (CY3240_SUCCESS(result)) {

            result = unpack_write_output(
                    &writeLength,
                    &readLength,
                    NULL);

            if CY3240_FAILURE(result)
                printf("Slave failed to Ack restart\n");
        }

        pthread_mutex_unlock(&mutex);

        return result;

    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_reinit(
        void *handle
        )
{
    // The handle is the pointer to the state structure
    const Cy3240_t* pCy3240 = (const Cy3240_t*)handle;

    // Check the parameters
    if (pCy3240 != NULL) {

        Cy3240_Error_t result = CY3240_ERROR_OK;

        uint16_t writeLength = 0;
        uint16_t readLength = 0;

        pthread_mutex_lock(&mutex);

        // Construct the packet
        if CY3240_SUCCESS(result) {

            result = pack_reinit(
                    &writeLength);

            if CY3240_FAILURE(result)
                printf("Failed to pack reinit data in write input packet: %i\n", result);
        }

        // Write the packet
        if (CY3240_SUCCESS(result)) {

            readLength = writeLength + CY3240_STATUS_CODE_SIZE;

            // Write the data to the buffer
            // Note! The received data is ignored
            result = transcieve(
                    pCy3240,
                    SEND_PACKET,
                    &writeLength,
                    RECV_PACKET,
                    &readLength);

            if CY3240_FAILURE(result)
                printf("Failed to transmit reinit packet\n");
        }

        pthread_mutex_lock(&mutex);

        return result;

    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_reconfigure(
        void *handle,
        Cy3240_Power_t power,
        Cy3240_Bus_t bus,
        Cy3240_I2C_ClockSpeed_t clock
        )
{
    // The handle is the pointer to the state structure
    Cy3240_t* const pCy3240 = (Cy3240_t* const)handle;

    // Check the parameters
    if (pCy3240 != NULL) {

        Cy3240_Error_t result = CY3240_ERROR_OK;

        pthread_mutex_lock(&mutex);

        // Set the clock mode
           if CY3240_SUCCESS(result) {

                result = reconfigure_clock(
                        pCy3240,
                        clock);

                if CY3240_FAILURE(result)
                    printf("Failed to set the requested clock mode: %02x\n", clock);

           }

        // Change the power mode
        if CY3240_SUCCESS(result) {

            result = reconfigure_power(
                    pCy3240,
                    power);

            if CY3240_FAILURE(result)
                printf("Failed to set the requested power mode: %02x\n", power);
        }



        // TODO: Changing bus not supported
        if CY3240_SUCCESS(result) {

            pCy3240->bus = bus;
        }

        pthread_mutex_unlock(&mutex);

        return result;

    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}


//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_write(
        void *handle,
        uint8_t address,
        const uint8_t* const pData,
        uint16_t* const pLength
        )
{

    // The handle is the pointer to the state structure
    Cy3240_t* pCy3240 = (Cy3240_t*)handle;

    if ((pCy3240 != NULL) &&
        (pData != NULL) &&
        (pLength != NULL) &&
        (*pLength != 0)) {

        Cy3240_Error_t result = CY3240_ERROR_OK;

        // TODO: Move the WriteStart point after each write
        uint16_t writeLength;
        uint16_t readLength;
        uint8_t* pWriteStart = pData;
        uint16_t bytesLeft = *pLength;

        bool first = true;

        pthread_mutex_lock(&mutex);

        while (CY3240_SUCCESS(result) && (bytesLeft > 0)) {

            // Are there going to be more segments
            bool more;

            // Check if there will be more segments
            if (bytesLeft > CY3240_MAX_WRITE_BYTES)
                 more = true;
            else
                 more = false;

            // Set the write and read length to transfer one packet at a time
            writeLength = MIN(bytesLeft, CY3240_MAX_WRITE_BYTES);
            readLength = writeLength + CY3240_STATUS_CODE_SIZE;

            // TODO:
            // Need to be able to retransmit data if Nack is received
            if CY3240_SUCCESS(result) {

                // Pack the data in to the send packet
                result = pack_write_input(
                        address,
                        pWriteStart,
                        &writeLength,
                        first,
                        more);

                if CY3240_FAILURE(result)
                    printf("Failed to pack send data in write input packet: %i\n", result);
            }

            if (CY3240_SUCCESS(result)) {

                // Write the data to the buffer
                result = transcieve(
                        pCy3240,
                        SEND_PACKET,
                        &writeLength,
                        RECV_PACKET,
                        &readLength);

                if CY3240_FAILURE(result)
                    printf("Failed to transmit write packet\n");
            }

            if (CY3240_SUCCESS(result)) {

                // Decode the response
                result = unpack_write_output(
                        &writeLength,
                        &readLength,
                        &bytesLeft);

                if CY3240_FAILURE(result)
                    printf("Failed to transmit all data\n");
            }

            // No longer the first time
            first = false;
        }

        pthread_mutex_unlock(&mutex);

        return result;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_read(
        void *handle,
        uint8_t address,
        uint8_t* const pData,
        uint16_t* const pLength
        )
{
    // The handle is the pointer to the state structure
    const Cy3240_t* pCy3240 = (const Cy3240_t*)handle;

    if ((pCy3240 != NULL) &&
        (pData != NULL) &&
        (pLength != NULL) &&
        (*pLength != 0)) {

        Cy3240_Error_t result = CY3240_ERROR_OK;

        uint16_t readLength;
        uint8_t* pReadStart = pData;
        const uint16_t writeLength = READ_INPUT_PACKET_SIZE;
        uint16_t bytesLeft = *pLength;

        pthread_mutex_lock(&mutex);

        // Loop while there is still data to read
        while (CY3240_SUCCESS(result) &&
               (bytesLeft > 0)) {

            // Calculate the number of packets to read
            readLength = MIN(bytesLeft, CY3240_MAX_READ_BYTES);

            // Create the read input packet
            if (CY3240_SUCCESS(result)) {

                result = pack_read_input(
                        address,
                        &readLength);

                if CY3240_FAILURE(result)
                    printf("Failed to pack read input packet: %i\n", result);
            }

            // Send the data
            if (CY3240_SUCCESS(result)) {

                // Write the data to the buffer
                result = transcieve(
                        pCy3240,
                        SEND_PACKET,
                        &writeLength,
                        RECV_PACKET,
                        &readLength);

                if CY3240_FAILURE(result)
                    printf("Failed to transmit read packet\n");
            }

			printf("read lengt: %d\n", readLength);

            // unpack the result
            if (CY3240_SUCCESS(result)) {

                // Decode the response
                result = unpack_read_output(
                        pReadStart,
                        &readLength);

                if CY3240_FAILURE(result) {
                    printf("Failed to read data\n");

                } else {
                    pReadStart += readLength;
                    bytesLeft -= readLength;
                }
            }
        }

        pthread_mutex_unlock(&mutex);

        return result;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}


static void writeInternal(const Cy3240_t* pCy3240, const uint8_t address) {

	memset(SEND_PACKET, 0u, 64);

    // Initialize the byte index
    SEND_PACKET[1] = 0x02;
    SEND_PACKET[2] = 0x00;
    SEND_PACKET[3] = address;
    SEND_PACKET[4] = 0x00;

	uint16_t writeLength = 5;

	uint16_t readLength = writeLength + CY3240_STATUS_CODE_SIZE;

	int result = transcieve(
			pCy3240, SEND_PACKET, &writeLength, RECV_PACKET, &readLength);

	// Send the data
    if CY3240_FAILURE(result)
        printf("Failed to transmit internal write packet\n");

}

static void write81(const Cy3240_t* pCy3240) {
    printf("Writing to 0x81...\n");
	writeInternal(pCy3240, 0x81);
	printf("...done\n.");
}

static void write8f(const Cy3240_t* pCy3240) {
	printf("Writing to 0x8f...\n");
	writeInternal(pCy3240, 0x8f);
	printf("...done\n.");
}

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_open(
        void *handle
        )
{
    // The handle is the pointer to the state structure
    Cy3240_t* pCy3240 = (Cy3240_t*)handle;

    if (pCy3240 != NULL) {

        int error = 0;
        Cy3240_Error_t result = CY3240_ERROR_OK;

        //hid_deviceMatcher matcher = {pCy3240->vendor_id, pCy3240->product_id, NULL, NULL, 0};

        pthread_mutex_lock(&mutex);

        // Initialize the device
        if CY3240_SUCCESS(result) {
            error = pCy3240->w.init();

            if (HID_FAILURE(error)) {
                 fprintf(stderr, "hid_init failed with return code %d\n", error);
                 result = CY3240_ERROR_HID;
            }
        }

        // For open the usb device
        if CY3240_SUCCESS(result) {

            fprintf(stdout, "hid_open %d %d %d\n", pCy3240->vendor_id, pCy3240->product_id, NULL);
            //pCy3240->pHid = hid_open_path("/dev/usb/hiddev2");

            pCy3240->pHid = pCy3240->w.open(pCy3240->vendor_id, pCy3240->product_id, NULL);
            fprintf(stdout, "hid device address: %d\n", pCy3240->pHid);


       		if (pCy3240->pHid == NULL) {
                fprintf(stderr, "hid_force_open failed with return code %d\n", error);
                result = CY3240_ERROR_HID;
            }

			#define MAX_STR 255
			wchar_t wstr[MAX_STR];

			// Read the Manufacturer String
			hid_get_manufacturer_string(pCy3240->pHid, wstr, MAX_STR);
			printf("Manufacturer String: %ls\n", wstr);

			// Read the Product String
			hid_get_product_string(pCy3240->pHid, wstr, MAX_STR);
			printf("Product String: %ls\n", wstr);

			// Read the Serial Number String
			hid_get_serial_number_string(pCy3240->pHid, wstr, MAX_STR);
			printf("Serial Number String: %ls", wstr);
			printf("\n");

			//printf("Numbered reorts: %d\n", pCy3240->pHid->uses_numbered_reports);

     
        }
		
        // TODO: figure out why these commands are needed
        write81(pCy3240);
        write8f(pCy3240);
        write81(pCy3240);
				
        pthread_mutex_unlock(&mutex);

        return result;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_close(
        void *handle
        )
{
    // The handle is the pointer to the state structure
    Cy3240_t* pCy3240 = (Cy3240_t*)handle;

    if (pCy3240 != NULL) {

        Cy3240_Error_t result = CY3240_ERROR_OK;
        int error = 0;

        pthread_mutex_lock(&mutex);

        // Close the connection
        if (CY3240_SUCCESS(result)) {
            pCy3240->w.close(pCy3240->pHid);
        }

        // Free unused resources
        if CY3240_SUCCESS(result)
            free(pCy3240);

        pthread_mutex_lock(&mutex);

        return result;
    }

    return CY3240_ERROR_INVALID_PARAMETERS;
}

//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_factory (
        void **pHandle,
        int iface_number,
        int timeout,
        Cy3240_Power_t power,
        Cy3240_Bus_t bus,
        Cy3240_I2C_ClockSpeed_t clock
        )
{

     // The handle is the pointer to the state structure
     Cy3240_t* pCy3240 = (Cy3240_t*)malloc(sizeof(Cy3240_t));

     // Check the parameters
     if (pCy3240 != NULL) {

          // Initialize the Cy3240 data structure
          pCy3240->vendor_id = CY3240_VID;
          pCy3240->product_id = CY3240_PID;
          pCy3240->iface_number = iface_number;
          pCy3240->timeout = timeout;
          pCy3240->power = power;
          pCy3240->bus = bus;
          pCy3240->clock = clock;
          pCy3240->w.init = hid_init;
          pCy3240->w.close = hid_close;
          pCy3240->w.write = hid_write;
          pCy3240->w.read = hid_read_timeout;
          pCy3240->w.open = hid_open;

          *pHandle = pCy3240;

          return CY3240_ERROR_OK;
     }

     return CY3240_ERROR_INVALID_PARAMETERS;

}   /* -----  end of function cy3240_util_factory  ----- */

//@} End of Methods

