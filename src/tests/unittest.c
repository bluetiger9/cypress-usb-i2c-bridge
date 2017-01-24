/**
 * @file unittest
 *
 * @brief Unit Test common data
 *
 * Unit Test common data
 *
 * @ingroup UnitTest
 *
 * @owner  Kevin S Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin S Kirkup (kevin.kirkup@gmail.com)
 */

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <string.h>
#include "unittest.h"
#include "cy3240_private_types.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

/**
 * The component that will be used for the unit tests.
 */
Cy3240_t* pMyData;

// The sending buffer
uint8_t SEND_BUFFER[SEND_BUFFER_SIZE] = {0};
uint8_t RECEIVE_BUFFER[RECEIVE_BUFFER_SIZE] = {0};

//@} End of Data


//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID init
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericInit(
        void
        )
{
    DBG(printf("Generic HID Init\n");)

    return 0;
}   /* -----  end of static function init_test  ----- */

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID close
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericClose(
        hid_device *const hidif
        )
{
    DBG(printf("Generic HID Close\n");)

    return 0;
}   /* -----  end of static function init_test  ----- */

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID write
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericWrite(
        hid_device* const hidif,
        unsigned int const ep,
        const char* bytes,
        unsigned int const size,
        unsigned int const timeout
        )
{
    DBG(printf("Generic HID Write\n");)

    // Write the data to the send buffer
    memcpy(SEND_BUFFER, bytes, size);

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
int
testGenericRead(
        hid_device* const hidif,
        unsigned int const ep,
        char* const bytes,
        unsigned int const size,
        unsigned int const timeout
        )
{
    DBG(printf("Generic HID Read\n");)

    // Copy the acknowledgments in the return buffer
    memcpy(bytes, RECEIVE_BUFFER, size);

    // Set the status byte to something unique
    bytes[0] = 0x07;

    return 0;
}

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID clean
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericCleanup(
        void
        )
{
    DBG(printf("Generic HID Cleanup\n");)

    return 0;
}

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID delete interface
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
void
testGenericDeleteIf(
        hid_device **const hidif
        )
{
    DBG(printf("Generic HID DeleteIf\n");)

    return;
}

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID Force Open
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
hid_device *
testOpen(
        unsigned short vendor_id, 
		unsigned short product_id, 
		const wchar_t *serial_number
        )
{
    DBG(printf("Generic HID Force Open\n");)

    return 0;
}

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the new HID Interface
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
hid_device *
testGenericNewHidInterface(
        void
        )
{
    DBG(printf("Generic HID new Interface\n");)

    // Give a constant for the interface type to avoid memory leak
    return (hid_device*)0x01;
}

//@} End of Methods
