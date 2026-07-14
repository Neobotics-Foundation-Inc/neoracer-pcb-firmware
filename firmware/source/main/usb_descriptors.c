#include "usb_descriptors.h"
#include "sdkconfig.h"

const char *oscore_usb_string_descriptor[] = {
    (char[]){0x09, 0x04},
    CONFIG_OSCORE_USB_MANUFACTURER_STRING,
    CONFIG_OSCORE_USB_PRODUCT_STRING,
    CONFIG_OSCORE_USB_SERIAL_STRING,
};

const size_t oscore_usb_string_descriptor_count =
    sizeof(oscore_usb_string_descriptor) / sizeof(oscore_usb_string_descriptor[0]);
