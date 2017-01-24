/**
 * @file cy3240_util.c
 *
 * @brief Utility functions for the CY3240
 *
 * Utility functions for the CY3240
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
#include <stdbool.h>
#include <stdlib.h>
#include <wchar.h>
#include "hidapi/hidapi.h"
#include "cy3240_types.h"
#include "cy3240_util.h"

//@} End of Includes


//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
bool
cy3240_util_match_serial_number(
        hid_device* device,
		const wchar_t* custom,
        size_t len
        )
{
    // Allocate a buffer to read the current usb device's serial number
    wchar_t* buffer = (wchar_t*) malloc (len * sizeof(wchar_t));

    // Get the serial number of the specfied device
    hid_get_serial_number_string(device, buffer, len);

    // Compare the current serial number with the one we are looking for
    bool ret = wcsncmp(buffer, custom, len) == 0;

    // Free the temporary buffer
    free(buffer);

    // Return the result of the compare
    return ret;
}

//@} End of Methods
