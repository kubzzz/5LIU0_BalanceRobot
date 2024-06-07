#include <stdio.h>
#include <fmt/format.h>
#include <libusb-1.0/libusb.h>

using namespace fmt;

#define VENDOR_ID 0x045e
#define PRODUCT_ID 0x02d1 // for Xbox one controller


int main(int argc, char **argv)
{
  int status;
  libusb_context *context = NULL;

  status = libusb_init(&context); // pass NULL for default context  
  if (status != 0)
  {
    print("ERROR: Failed to init libusb with status=, {}\n", libusb_strerror(status));
    return EXIT_FAILURE;
  }

  libusb_device_handle *device_handle = NULL;

  device_handle = libusb_open_device_with_vid_pid(context, VENDOR_ID, PRODUCT_ID);
  if (device_handle == NULL)
  {
    print("ERROR: Failed to find USB device with idVendor={:#06x}, idProduct={:#06x}\n", VENDOR_ID, PRODUCT_ID);
    print("You may not have root priviledges...\n");
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  libusb_device *device = NULL;

  device = libusb_get_device(device_handle);
  if (device == NULL)
  {
    print("ERROR: Failed to get device\n");
    libusb_close(device_handle);
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  libusb_device_descriptor device_desc;

  status = libusb_get_device_descriptor(device, &device_desc);
  if (status != 0)
  {
    print("ERROR: Failed to get device descriptor, {}\n", libusb_strerror(status));
    libusb_close(device_handle);
    libusb_exit(context);
    return EXIT_FAILURE;
  }
  print("NumConfigurations={} \n", device_desc.bNumConfigurations);

  libusb_config_descriptor *config_desc;
  status = libusb_get_config_descriptor(device, 0, &config_desc);
  if (status != 0)
  {
    print("ERROR: Failed to get configuration descriptor, {}\n", libusb_strerror(status));
    libusb_close(device_handle);
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  print("NumInterfaces={}\n", config_desc->bNumInterfaces);
  print("NumEndpoints={}\n", config_desc->interface->altsetting->bInterfaceNumber);

  uint8_t interfaceNum;

  for (uint8_t InterfaceNum = 0; InterfaceNum < config_desc->bNumInterfaces; InterfaceNum++)
  {
    status = libusb_kernel_driver_active(device_handle, InterfaceNum);
    if (status != 0)
    {
      if (status == 1)
      {
        print("ERROR: A kernel driver is active on interface({})\n",InterfaceNum);
        
        status = libusb_detach_kernel_driver(device_handle, InterfaceNum);
        if (status == 0)
        {
          interfaceNum = InterfaceNum;
        }
        if (status != 0)
        {
          print("ERROR: Failed to detach kernel driver on interface({}), {}\n", InterfaceNum, libusb_strerror(status));
        }
        status = libusb_kernel_driver_active(device_handle, InterfaceNum);
        if (status == 0)
        {
          print("Interface({}) is free!\n", InterfaceNum);
        }
      }
      else
      {
        print("ERROR: {}\n", libusb_strerror(status));
        libusb_free_config_descriptor(config_desc);
        libusb_close(device_handle);
        libusb_exit(context);
        return EXIT_FAILURE;
      }
    }
  }

  status = libusb_claim_interface(device_handle, interfaceNum);
  if (status == 0)
  {
    print("Succesful claimed interface({})!\n", interfaceNum);
  }
  else
  {
    print("ERROR: Failed to claim interface({}), {}\n", interfaceNum, libusb_strerror(status));
  }


  libusb_free_config_descriptor(config_desc);


  /* Release interface number */
  status = libusb_release_interface(device_handle, interfaceNum);
  if (status != 0)
  {
    print("ERROR: Failed to release interface({}), {}\n", interfaceNum, libusb_strerror(status));
  }

  /* re attach the kernel driver again before closing */
  status = libusb_attach_kernel_driver(device_handle, interfaceNum);
  if (status != 0)
  {
    print("ERROR: Failed to attach kernel driver on interface({}), {}\n", interfaceNum, libusb_strerror(status));
  }

  libusb_close(device_handle);

  libusb_exit(context);
  return EXIT_SUCCESS;
}