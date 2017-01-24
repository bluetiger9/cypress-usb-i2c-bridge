/**
 * @file writeTest.c
 *
 * @brief Write test for the CY3240 code
 *
 * Write test for the CY3240 code
 *
 * @ingroup Write
 *
 * @owner  Kevin Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin Kirkup (kevin.kirkup@gmail.com)
 */

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include "string.h"
#include "unittest.h"
#include "writeTest.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

// The location where data should be written in the send buffer
uint8_t* pWrite;

//@} End of Data


//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID write
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
static int
myWrite(
        hid_device* const hidif,
        unsigned int const ep,
        const char* bytes,
        unsigned int const size,
        unsigned int const timeout
        )
{
    DBG(printf("HID Write\n");)

    // Write the data to the send buffer
    memcpy(pWrite, bytes, size);

    // Move the write pointers
    pWrite += size;

    return 0;
}

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID read
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
static int
myRead(
        hid_device* const hidif,
        unsigned int const ep,
        char* const bytes,
        unsigned int const size,
        unsigned int const timeout
        )
{
    DBG(printf("HID Read\n");)

    // Copy the acknowledgments in the return buffer
    memcpy(bytes, RECEIVE_BUFFER, size);

    // Set the status byte to something unique
    bytes[0] = 0x07;

    return 0;
}

//-----------------------------------------------------------------------------
/**
 *  Setup before each test case
 */
//-----------------------------------------------------------------------------
A_Before void
testWriteSetup(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // Initialize the send buffer
    memset(SEND_BUFFER, 0x00, sizeof(SEND_BUFFER));

    // Initialize the write location
    pWrite = SEND_BUFFER;

    // Fill the receive buffer with ack bytes
    memset(RECEIVE_BUFFER, TX_ACK, sizeof(RECEIVE_BUFFER));

    // Initialize the state
    result = cy3240_factory(
            &handle,
            0,
            1000,
            CY3240_POWER_5V,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    assertTrue("The usb device should be successfully created",
            CY3240_SUCCESS(result)
            );

    pMyData = (Cy3240_t*)handle;

    // Modify the read and write interfaces to point to our functions
    pMyData->w.init = testGenericInit;
    pMyData->w.close = testGenericClose;
    pMyData->w.write = myWrite;
    pMyData->w.read = myRead;
    pMyData->w.open = testOpen;

    // Open the device
    result = cy3240_open(handle);
}


//-----------------------------------------------------------------------------
/**
 *  Cleanup after each test case
 */
//-----------------------------------------------------------------------------
A_After void
testWriteCleanup(
        void
        )
{
    int handle = (int)pMyData;

    // Close the device handle
    cy3240_close(handle);
}

//-----------------------------------------------------------------------------
/**
 *  Error Test Case
 */
//-----------------------------------------------------------------------------
A_Test void
testWriteError(
        void
        )
{
    uint8_t data[8] = {0};
    uint16_t length = 8;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // NULL Handle
    result = cy3240_write(
            handle,
            MY_ADDRESS,
            data,
            &length);

    assertEquals("Pass write with NULL handle should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );

    // Initialize the handle
    handle = (int)pMyData;

    // NULL Data buffer
    result = cy3240_write(
            handle,
            MY_ADDRESS,
            NULL,
            &length
            );

    assertEquals("Pass write with NULL data buffer should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );

    // NULL length parameter
    cy3240_write(
            handle,
            MY_ADDRESS,
            data,
            NULL
            );

    assertEquals("Passing write with NULL length should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );

    // 0 length
    length = 0;
    cy3240_write(
            handle,
            MY_ADDRESS,
            data,
            &length
            );

    assertEquals("Pass zero for the length should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for writing a small amount of data i.e. < packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testWriteSmall(
        void
        )
{
    uint8_t data[8] = {0};
    uint16_t length = 8;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Fill the data buffer with a test pattern
    memset(data, 0xAC, sizeof(data));

    // Write a small packet to the device
    result = cy3240_write(
            handle,
            MY_ADDRESS,
            data,
            &length
            );

    assertTrue("The write should complete successfully",
            CY3240_SUCCESS(result)
            );

    assertEquals("The control byte should show start, stop, write and I2C: 0x0A",
            0x0A,
            SEND_BUFFER[INPUT_PACKET_INDEX_CMD]
            );

    assertEquals("The length byte should show length 8 and no more packets: 0x08",
            0x08,
            SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]
            );

    assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
            MY_ADDRESS,
            SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]
            );

    assertEquals("The data portion of the send buffer should equal the data buffer",
            0,
            memcmp(data, &SEND_BUFFER[WRITE_INPUT_PACKET_INDEX_DATA], length)
            );
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for writing a medium amount of data i.e. = packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testWriteMedium(
        void
        )
{
    uint8_t data[61] = {0};
    uint16_t length = 61;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Fill the data buffer with a test pattern
    memset(data, 0xAC, sizeof(data));

    // Write a small packet to the device
    result = cy3240_write(
            handle,
            MY_ADDRESS,
            data,
            &length
            );

    assertTrue("The write should complete successfully",
            CY3240_SUCCESS(result)
            );

    assertEquals("The control byte should show start, stop, write and I2C: 0x0A",
            0x0A,
            SEND_BUFFER[INPUT_PACKET_INDEX_CMD]
            );

    assertEquals("The length byte should show length 61 and no more packets: 0x3D",
            0x3D,
            SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]
            );

    assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
            MY_ADDRESS,
            SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]
            );

    assertEquals("The data portion of the send buffer should equal the data buffer",
            0,
            memcmp(data, &SEND_BUFFER[WRITE_INPUT_PACKET_INDEX_DATA], length)
            );
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for writing a large amount of data i.e. > packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testWriteLarge(
        void
        )
{
    uint8_t data[69] = {0};
    uint16_t length = 69;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Fill the data buffer with a test pattern
    memset(data, 0xAC, sizeof(data));

    // Write a small packet to the device
    result = cy3240_write(
            handle,
            MY_ADDRESS,
            data,
            &length);

    assertTrue("The write should complete successfully",
            CY3240_SUCCESS(result)
            );

    assertEquals("The control byte should show start, write and I2C: 0x02",
            0x02,
            SEND_BUFFER[INPUT_PACKET_INDEX_CMD]
            );

    assertEquals("The length byte should show length 61 and more packets: 0xBD",
            0xBD,
            SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]
            );

    assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
            MY_ADDRESS,
            SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]
            );

    assertEquals("The data portion of the send buffer should equal the data buffer",
            0,
            memcmp(data, &SEND_BUFFER[WRITE_INPUT_PACKET_INDEX_DATA], CY3240_MAX_WRITE_BYTES)
            );

    // Shift the offset of the buffer for the second packet
    assertEquals("The control byte should show start, stop, write and I2C: 0x0A",
            0x0A,
            SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_CMD]
            );

    assertEquals("The length byte should show length 8 and no more packets: 0x08",
            0x08,
            SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_LENGTH]
            );

    assertEquals("The data portion of the send buffer should equal the data buffer",
            0,
            memcmp(data, &SEND_BUFFER[CY3240_MAX_SIZE_PACKET + WRITE_INPUT_PACKET_INDEX_DATA], 8)
            );

}

//-----------------------------------------------------------------------------
/**
 *  Test Case to test Nack during transmit
 */
//-----------------------------------------------------------------------------
A_Test void
testWriteNack(
        void
        )
{
}

//@} End of Methods

