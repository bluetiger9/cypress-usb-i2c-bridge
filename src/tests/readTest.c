/**
 * @file readTest
 *
 * @brief CY3240 Read tests
 *
 * CY3240 Read tests
 *
 * @ingroup Read
 *
 * @owner  Kevin S Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin S Kirkup (kevin.kirkup@gmail.com)
 */
//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <string.h>
#include "unittest.h"
#include "readTest.h"
//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

// The location where data should be written in the send buffer
uint8_t* pRead;
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

    // Increment the write pointer
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
    memcpy(bytes, pRead, size);

    // Move the read buffer pointer
    pRead += size;

    return 0;
}

//-----------------------------------------------------------------------------
/**
 *  Setup before each test case
 */
//-----------------------------------------------------------------------------
A_Before void
testReadSetup(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // Initialize the send buffer
    memset(SEND_BUFFER, 0x00, sizeof(SEND_BUFFER));

    // Initialize the read/write buffer pointers
    pRead = RECEIVE_BUFFER;
    pWrite = SEND_BUFFER;

    // Fill the receive buffer with the data for the first packet
    memset(RECEIVE_BUFFER, 0xAC, CY3240_MAX_SIZE_PACKET);

    // Set the status byte to something unique
    RECEIVE_BUFFER[0] = 0x07;

    // Fill the receive buffer with the data for the second packet
    memset(RECEIVE_BUFFER + CY3240_MAX_SIZE_PACKET, 0xFA, CY3240_MAX_SIZE_PACKET);

    // Set the status byte to something unique
    RECEIVE_BUFFER[CY3240_MAX_SIZE_PACKET] = 0x07;

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
testReadCleanup(
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
testReadError(
        void
        )
{
    uint8_t data[8] = {0};
    uint16_t length = 8;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // NULL handle
    result = cy3240_read(
            handle,
            MY_ADDRESS,
            data,
            &length
            );

    assertEquals("Pass read with NULL handle should indicate invalid parameter",
              CY3240_ERROR_INVALID_PARAMETERS,
              result
              );

    // Initialize the handle
    handle = (int)pMyData;

    // NULL Data buffer
    result = cy3240_read(
            handle,
            MY_ADDRESS,
            NULL,
            &length
            );

    assertEquals("Pass read with NULL data buffer should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );

    // NULL length parameter
    result = cy3240_read(
            handle,
            MY_ADDRESS,
            data,
            NULL
            );

    assertEquals("Passing read with NULL length should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );

    // 0 length
    length = 0;
    result = cy3240_read(
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
 *  Test Case for reading a small amount of data i.e. < packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testReadSmall(
        void
        )
{
    uint8_t data[8] = {0};
    uint16_t length = 8;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Write a small packet to the device
    result = cy3240_read(
            handle,
            MY_ADDRESS,
            data,
            &length
            );

    assertTrue("The read should complete successfully",
            CY3240_SUCCESS(result)
            );

    assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
            0x0B,
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

    assertEquals("The read data should match the read buffer: 0x0AC",
            0,
            memcmp(data, &RECEIVE_BUFFER[OUTPUT_PACKET_INDEX_DATA], length)
            );
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for reading a medium amount of data i.e. = packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testReadMedium (
        void
        )
{
    uint8_t data[CY3240_MAX_READ_BYTES] = {0};
    uint16_t length = CY3240_MAX_READ_BYTES;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Write a small packet to the device
    result = cy3240_read(
            handle,
            MY_ADDRESS,
            data,
            &length);

    assertTrue("The read should complete successfully",
            CY3240_SUCCESS(result)
            );

    assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
            0x0B,
            SEND_BUFFER[INPUT_PACKET_INDEX_CMD]
            );

    assertEquals("The length byte should show length 61 and no more packets: 0x3F",
            0x3D,
            SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]
            );

    assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
            MY_ADDRESS,
            SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]
            );

    assertEquals("The read data should match the read buffer: 0x0AC",
            0,
            memcmp(data, &RECEIVE_BUFFER[OUTPUT_PACKET_INDEX_DATA], length)
            );
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for reading a large amount of data i.e. > packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testReadLarge(
        void
        )
{
    uint8_t data[69] = {0};
    uint16_t length = 69;
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Write a small packet to the device
    result = cy3240_read(
            handle,
            MY_ADDRESS,
            data,
            &length
            );

    assertTrue("The read should complete successfully",
            CY3240_SUCCESS(result)
            );

    assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
            0x0B,
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

    assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
            0x0B,
            SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_CMD]
            );

    assertEquals("The length byte should show length 8 and no more packets: 0x08",
            0x08,
            SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_LENGTH]
            );

    assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
            MY_ADDRESS,
            SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_ADDRESS]
            );

    // Shift the offset of the buffer for the second packet
    assertEquals("The first packet of data should match 0xAC",
            0,
            memcmp(data, &RECEIVE_BUFFER[1], CY3240_MAX_READ_BYTES)
            );

    assertEquals("The remaining packet data should match 0xFA",
            0,
            memcmp(&data[CY3240_MAX_READ_BYTES], &RECEIVE_BUFFER[CY3240_MAX_SIZE_PACKET + 1], 8)
            );
}

//@} End of Methods
