/**
 * @file reconfigTest
 *
 * @brief Unit test for the reconfigure command
 *
 * Unit test for the reconfigure command
 *
 * @ingroup 
 *
 * @owner  Kevin S Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin S Kirkup (kevin.kirkup@gmail.com)
 */

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <string.h>
#include "reconfigTest.h"
#include "unittest.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

// The location where data should be written in the send buffer
uint8_t* pWrite;

//@} End of Data

//////////////////////////////////////////////////////////////////////
/// @name Private Methods
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

    // Move the write pointer
    pWrite += size;

    return 0;
}

//-----------------------------------------------------------------------------
/**
 * Method to test the clock packet
 *
 * @param pData   [in] the start of the data
 * @param control [in] the control byte
 * @param length  [in] the length byte
 * @param address [in] the address byte
 */
//-----------------------------------------------------------------------------
static bool
testClockPacket(
        const uint8_t* const pData,
        uint8_t control,
        uint8_t length,
        uint8_t address
        )
{
    // Check the control byte
    if (control != pData[INPUT_PACKET_INDEX_CMD]) {
        printf("The control byte should indicate Reconfigure, 0x%02x but was 0x%02x\n",
                control,
                pData[INPUT_PACKET_INDEX_CMD]
                );

        return false;
    }

    // Check the length
    if (length != pData[INPUT_PACKET_INDEX_LENGTH]) {
        printf("The length byte should be 0x%02x but was 0x%02X\n",
                length,
                pData[INPUT_PACKET_INDEX_LENGTH]
                );

        return false;
    }

    // Check the address
    if (address != pData[INPUT_PACKET_INDEX_ADDRESS]) {
        printf("The address byte should show the control register, 0x%02x but was 0x%02x\n",
                address,
                pData[INPUT_PACKET_INDEX_ADDRESS]
                );

        return false;
    }

    // Everything was OK
    return true;
}

//-----------------------------------------------------------------------------
/**
 * Method to set the power packet
 *
 * @param pData   [in] the start of the data
 * @param control [in] the control byte
 * @param length  [in] the length byte
 * @param address [in] the address byte
 * @param power   [in] the power mode byte
 * @return
 */
//-----------------------------------------------------------------------------
static bool
testPowerPacket(
        const uint8_t* const pData,
        uint8_t control,
        uint8_t length,
        uint8_t address,
        uint8_t power
        )
{
    // Check the control byte
    if (control != pData[INPUT_PACKET_INDEX_CMD]) {
        printf("The control byte should indicate Write and Start: 0x%02x but was 0x%02x\n",
                control,
                pData[INPUT_PACKET_INDEX_CMD]
                );

        return false;
    }

    // Check the length
    if (length != pData[INPUT_PACKET_INDEX_LENGTH]) {
        printf("The length byte should be 0x%02x but was 0x%02x\n",
                length,
                pData[INPUT_PACKET_INDEX_LENGTH]
                );

        return false;
    }

    // Check the address
    if (address != pData[INPUT_PACKET_INDEX_ADDRESS]) {
        printf("The address byte should show the control register, 0x%02x but was 0x%02x\n",
                address,
                pData[INPUT_PACKET_INDEX_ADDRESS]
                );

        return false;
    }

    // Check the power
    if (power != pData[INPUT_PACKET_INDEX_ADDRESS + 1]) {
        printf("The power mode byte should be 0x%02x but was 0x%02x\n",
                power,
                pData[INPUT_PACKET_INDEX_ADDRESS + 1]
                );

        return false;
    }

    return true;
}

//@} End of Private Methods

//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Setup before each test case
 */
//-----------------------------------------------------------------------------
A_Before void
testReconfigSetup(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // Initialize the send buffer
    memset(SEND_BUFFER, 0x00, sizeof(SEND_BUFFER));

    // Fill the receive buffer with the data for the first packet
    memset(RECEIVE_BUFFER, TX_ACK, RECEIVE_BUFFER_SIZE);

    // Set the status byte to something unique
    RECEIVE_BUFFER[0] = 0x07;

    // Initialize the write buffer pointer
    pWrite = SEND_BUFFER;

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
    pMyData->w.read = testGenericRead;
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
testReconfigCleanup (
        void
        )
{
    int handle = (int)pMyData;

    // Initialize the write buffer pointer
    pWrite = SEND_BUFFER;

    // Close the device handle
    cy3240_close(handle);
}

//-----------------------------------------------------------------------------
/**
 *  Error Test Case
 */
//-----------------------------------------------------------------------------
A_Test void
testReconfigError(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // NULL Data buffer
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    assertEquals("Pass reconfigure with NULL handle should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result);
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigPowerExternal(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set external power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    assertEquals("The operation should result in an OK",
            CY3240_ERROR_OK,
            result
            );

    assertTrue("Power External",
            testPowerPacket(
            SEND_BUFFER,
            CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_START | CY3240_CLOCK__100kHz | CY3240_BUS_I2C,
            0x01,
            CONTROL_I2C_ADDRESS,
            CY3240_POWER_EXTERNAL
            ));
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigPower5V(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set 5V power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_5V,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    assertEquals("The operation should result in an OK",
            CY3240_ERROR_OK,
            result
            );

    assertTrue("Power 5V",
            testPowerPacket(
            SEND_BUFFER,
            CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_START | CY3240_CLOCK__100kHz | CY3240_BUS_I2C,
            0x01,
            CONTROL_I2C_ADDRESS,
            CY3240_POWER_5V
            ));
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigPower3_3V(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set 3.3V power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_3_3V,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    assertEquals("The operation should result in an OK",
            CY3240_ERROR_OK,
            result
            );

    assertTrue("Power 3.3V",
            testPowerPacket(
            SEND_BUFFER,
            CONTROL_BYTE_I2C_WRITE | CONTROL_BYTE_START | CY3240_CLOCK__100kHz | CY3240_BUS_I2C,
            0x01,
            CONTROL_I2C_ADDRESS,
            CY3240_POWER_3_3V
            ));
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigClock100kHz(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );

    assertEquals("The operation should result in an OK",
            CY3240_ERROR_OK,
            result
            );

    assertTrue("Clock 100KHz",
            testClockPacket(
            SEND_BUFFER + CY3240_MAX_SIZE_PACKET,
            CONTROL_BYTE_RECONFIG | CY3240_CLOCK__100kHz | CY3240_BUS_I2C,
            0x00,
            CONTROL_I2C_ADDRESS
            ));
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigClock400kHz(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__400kHz
            );

    assertEquals("The operation should result in an OK",
            CY3240_ERROR_OK,
            result
            );

    assertTrue("Clock 400KHz",
            testClockPacket(
            SEND_BUFFER + CY3240_MAX_SIZE_PACKET,
            CONTROL_BYTE_RECONFIG | CY3240_CLOCK__400kHz | CY3240_BUS_I2C,
            0x00,
            CONTROL_I2C_ADDRESS
            ));
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigClock50kHz(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__50kHz
            );

    assertEquals("The operation should result in an OK",
            CY3240_ERROR_OK,
            result
            );

    assertTrue("Clock 50KHz",
            testClockPacket(
            SEND_BUFFER + CY3240_MAX_SIZE_PACKET,
            CONTROL_BYTE_RECONFIG | CY3240_CLOCK__50kHz | CY3240_BUS_I2C,
            0x00,
            CONTROL_I2C_ADDRESS
            ));
}

//-----------------------------------------------------------------------------
A_Test void
testReconfigClockReserved(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__Reserved
            );

    assertTrue("Clock Reserved",
            testClockPacket(
            SEND_BUFFER + CY3240_MAX_SIZE_PACKET,
            CONTROL_BYTE_RECONFIG | CY3240_CLOCK__Reserved | CY3240_BUS_I2C,
            0x00,
            CONTROL_I2C_ADDRESS
            ));
}

//-----------------------------------------------------------------------------
// TODO: changing the bus mode is not currently supported
A_Test void
testReconfigBusI2C(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_I2C,
            CY3240_CLOCK__100kHz
            );
}

//-----------------------------------------------------------------------------
// TODO: changing the bus mode is not currently supported
A_Test void
testReconfigBusSPI(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_SPI,
            CY3240_CLOCK__100kHz
            );
}

//-----------------------------------------------------------------------------
// TODO: changing the bus mode is not currently supported
A_Test void
testReconfigBusUART(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_UART,
            CY3240_CLOCK__100kHz
            );
}

//-----------------------------------------------------------------------------
// TODO: changing the bus mode is not currently supported
A_Test void
testReconfigBusLIN(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = (int)pMyData;

    // Set the power mode
    result = cy3240_reconfigure(
            handle,
            CY3240_POWER_EXTERNAL,
            CY3240_BUS_LIN,
            CY3240_CLOCK__100kHz
            );
}

//-----------------------------------------------------------------------------
A_Test void
testReinitError(
        void
        )
{
    Cy3240_Error_t result = CY3240_ERROR_OK;
    int handle = 0;

    // NULL Data buffer
    result = cy3240_reinit(handle);

    assertEquals("Pass reinit with NULL handle should indicate invalid parameter",
            CY3240_ERROR_INVALID_PARAMETERS,
            result
            );
}

//@} End of Methods
