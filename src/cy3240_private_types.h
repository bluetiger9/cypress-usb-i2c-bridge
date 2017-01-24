/**
 * @file cy3240_private_types
 *
 * @brief Private type definitions for the CY3240 bridge controller
 *
 * Private type definitions for the CY3240 bridge controller
 *
 * @ingroup CY3240
 *
 * @owner  Kevin S Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin S Kirkup (kevin.kirkup@gmail.com)
 */
#ifndef INCLUSION_GUARD_CY3240_PRIVATE_TYPES_H
#define INCLUSION_GUARD_CY3240_PRIVATE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <hidapi/hidapi.h>
#include "cy3240_types.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Types
//@{

/**
 * Function pointer for the HID write function
 */
typedef int
(*hid_write_fpt)(
        hid_device *device, const unsigned char *data, size_t length
        );

/**
 * Function pointer for the HID read function
 */
typedef int
(*hid_read_fpt)(
        hid_device *dev, unsigned char *data, size_t length, int milliseconds
        );


/**
 * Function pointer for the HID init function
 */
typedef int
(*hid_init_fpt)(
        void
        );

/**
 * Function pointer for the HID close function
 */
typedef void
(*hid_close_fpt)(
        hid_device *const hidif
        );

/**
 * Function pointer for the HID cleanup function
 
typedef int
(*hid_cleanup_fpt)(
        void
        );
*/
/**
 * Function pointer for the HID delete interface function
 
typedef void
(*hid_delete_hid_device_fpt)(
        hid_device **const hidif
        );
*/

/**
 * Function pointer for the HID force open function
 */
typedef hid_device *
(*hid_open_fpt)(
        unsigned short vendor_id, 
		unsigned short product_id, 
		const wchar_t *serial_number
        );

/**
 * Function pointer for the HID new Interface function
 
typedef hid_device *
(*hid_new_hid_device_fpt)(
        void
        );
*/

/**
 * Structure to wrap the hid interface
 */
typedef struct {
    hid_init_fpt init;                         ///< Pointer to the hid init function
    hid_close_fpt close;                       ///< Pointer to the hid close function
    hid_write_fpt write;                       ///< Pointer to the hid write function
    hid_read_fpt read;                         ///< Pointer to the hid read function
    hid_open_fpt open;      				       ///< Pointer to the hid force open function
} hid_wrapper_t;

/**
 * CY3240 device state structure
 */
typedef struct {
    uint16_t vendor_id;                        ///< Vendor ID
    uint16_t product_id;                       ///< Product ID
    int iface_number;                          ///< The interface number
    int timeout;                               ///< USB Transfer timeout
    Cy3240_I2C_ClockSpeed_t clock;             ///< The clock speed
    Cy3240_Bus_t bus;                          ///< The bus configuration
    Cy3240_Power_t power;                      ///< The power configuration
    hid_device *pHid;                          ///< HID Interface
    hid_wrapper_t w;                           ///< HID interface wrapper
} Cy3240_t;

//@} End of Types

#ifdef __cplusplus
}
#endif

#endif // INCLUSION_GUARD_CY3240_PRIVATE_TYPES_H
