#include <stdio.h>
#include <fmt/format.h>
#include <libusb-1.0/libusb.h>

using namespace fmt;

#define VENDOR_ID 0x045e
#define PRODUCT_ID 0x02d1


int main(int argc, char **argv)
{
  int status;
  libusb_device_handle *device = NULL;

  status = libusb_init(NULL); // for default loggin information 
  if (status != 0)
  {
    print("ERROR: Failed to init libusb, libusb_init()\n");
    return -status;
  }

  device = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
  if (device == NULL)
  {
    print("ERROR: Failed to find USB device with idVendor={:#06x}, idProduct={:#06x}\n", VENDOR_ID, PRODUCT_ID);
    print("You may not have root priviledges...\n");
    libusb_exit(NULL);
    return -1;
  }

  



  libusb_close(device);

  libusb_exit(NULL);
  return 0;
}