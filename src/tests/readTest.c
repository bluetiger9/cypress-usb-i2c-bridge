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

#include "string.h"
#include "unittest.h"
#include "readTest.h"
//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

// The location where data should be written in the send buffer
uint8* pRead;
uint8* pWrite;

//@} End of Data

//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID init
 *
 *  @see hid.h
 *  @returns hid_return
 */
//-----------------------------------------------------------------------------
hid_return
testReadInit(
          void
          )
{
     DBG(printf("HID Init\n");)

     return HID_RET_SUCCESS;
}   /* -----  end of static function init_test  ----- */

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID write
 *
 *  @see hid.h
 *  @returns hid_return
 */
//-----------------------------------------------------------------------------
hid_return
testReadWrite(
          HIDInterface* const hidif,
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

     return HID_RET_SUCCESS;
}

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID read
 *
 *  @see hid.h
 *  @returns hid_return
 */
//-----------------------------------------------------------------------------
hid_return
testReadRead(
          HIDInterface* const hidif,
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

     return HID_RET_SUCCESS;
}

//-----------------------------------------------------------------------------
/**
 *  Setup before each test case
 */
//-----------------------------------------------------------------------------
A_Before void
testReadSetup (
          void
          )
{
     Cy3240_Error_t result = CY3240_ERROR_OK;

     // Initialize the state
     memset(&myData, 0x00, sizeof(Cy3240_t));

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
     result = cy3240_util_factory(
               &myData,
               0,
               1000,
               CY3240_POWER_5V,
               CY3240_BUS_I2C,
               CY3240_100kHz);

     assertTrue("The usb device should be successfully created",
               CY3240_SUCCESS(result));

     // Modify the read and write interfaces to point to our functions
     myData.init = testReadInit;
     myData.write = testReadWrite;
     myData.read = testReadRead;
}

//-----------------------------------------------------------------------------
/**
 *  Cleanup after each test case
 */
//-----------------------------------------------------------------------------
A_After void
testReadCleanup (
          void
          )
{
     // Initialize the state
     memset(&myData, 0x00, sizeof(Cy3240_t));
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
     uint8 data[8] = {0};
     uint16 length = 8;
     Cy3240_Error_t result = CY3240_ERROR_OK;

     // NULL Data buffer
     result = cy3240_read(
               &myData,
               MY_ADDRESS,
               NULL,
               &length);

     assertEquals("Pass read with NULL data buffer should indicate invalid parameter",
               CY3240_ERROR_INVALID_PARAMETERS,
               result);

     // NULL length parameter
     cy3240_read(
               &myData,
               MY_ADDRESS,
               data,
               NULL);

     assertEquals("Passing read with NULL length should indicate invalid parameter",
               CY3240_ERROR_INVALID_PARAMETERS,
               result);

     // 0 length
     length = 0;
     cy3240_read(
               &myData,
               MY_ADDRESS,
               data,
               &length);

     assertEquals("Pass zero for the length should indicate invalid parameter",
               CY3240_ERROR_INVALID_PARAMETERS,
               result);
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for reading a small amount of data i.e. < packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testReadSmall (
          void
          )
{
     uint8 data[8] = {0};
     uint16 length = 8;
     Cy3240_Error_t result = CY3240_ERROR_OK;

     // Write a small packet to the device
     result = cy3240_read(
               &myData,
               MY_ADDRESS,
               data,
               &length);

     assertTrue("The read should complete successfully",
               CY3240_SUCCESS(result));

     assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
               0x0B,
               SEND_BUFFER[INPUT_PACKET_INDEX_CMD]);

     assertEquals("The length byte should show length 8 and no more packets: 0x08",
               0x08,
               SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]);

     assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
               MY_ADDRESS,
               SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]);

     assertEquals("The read data should match the read buffer: 0x0AC",
               0,
               memcmp(data, &RECEIVE_BUFFER[OUTPUT_PACKET_INDEX_DATA], length));
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
     uint8 data[CY3240_MAX_READ_BYTES] = {0};
     uint16 length = CY3240_MAX_READ_BYTES;
     Cy3240_Error_t result = CY3240_ERROR_OK;

     // Write a small packet to the device
     result = cy3240_read(
               &myData,
               MY_ADDRESS,
               data,
               &length);

     assertTrue("The read should complete successfully",
               CY3240_SUCCESS(result));

     assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
               0x0B,
               SEND_BUFFER[INPUT_PACKET_INDEX_CMD]);

     assertEquals("The length byte should show length 61 and no more packets: 0x3F",
               0x3D,
               SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]);

     assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
               MY_ADDRESS,
               SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]);

     assertEquals("The read data should match the read buffer: 0x0AC",
               0,
               memcmp(data, &RECEIVE_BUFFER[OUTPUT_PACKET_INDEX_DATA], length));
}

//-----------------------------------------------------------------------------
/**
 *  Test Case for reading a large amount of data i.e. > packet size
 */
//-----------------------------------------------------------------------------
A_Test void
testReadLarge (
          void
          )
{
     uint8 data[69] = {0};
     uint16 length = 69;
     Cy3240_Error_t result = CY3240_ERROR_OK;

     // Write a small packet to the device
     result = cy3240_read(
               &myData,
               MY_ADDRESS,
               data,
               &length);

     assertTrue("The read should complete successfully",
               CY3240_SUCCESS(result));

     assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
               0x0B,
               SEND_BUFFER[INPUT_PACKET_INDEX_CMD]);

     assertEquals("The length byte should show length 61 and no more packets: 0x3D",
               0x3D,
               SEND_BUFFER[INPUT_PACKET_INDEX_LENGTH]);

     assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
               MY_ADDRESS,
               SEND_BUFFER[INPUT_PACKET_INDEX_ADDRESS]);

     assertEquals("The control byte should show start, stop, read and I2C: 0x0B",
               0x0B,
               SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_CMD]);

     assertEquals("The length byte should show length 8 and no more packets: 0x08",
               0x08,
               SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_LENGTH]);

     assertEquals("The address portion of the send buffer should equal MY_ADDRESS",
               MY_ADDRESS,
               SEND_BUFFER[CY3240_MAX_SIZE_PACKET + INPUT_PACKET_INDEX_ADDRESS]);

     // Shift the offset of the buffer for the second packet
     assertEquals("The first packet of data should match 0xAC",
               0,
               memcmp(data, &RECEIVE_BUFFER[1], CY3240_MAX_READ_BYTES));

     assertEquals("The remaining packet data should match 0xFA",
               0,
               memcmp(&data[CY3240_MAX_READ_BYTES], &RECEIVE_BUFFER[CY3240_MAX_SIZE_PACKET + 1], 8));
}

//@} End of Methods
