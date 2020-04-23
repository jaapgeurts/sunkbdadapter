#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

#define MAX_BUF 255

char buf[MAX_BUF];

int main()
{
	printf("Sun Type 6 UART Keyboard tool\n");

	struct libusb_context* ctxt;

    if (libusb_init(&ctxt)) {
		perror("Can't open libusb");
		exit(1);
	}

	struct libusb_device **devices;
	int numDevices = libusb_get_device_list(ctxt, &devices);
	if (numDevices < 0) {
		perror("Can't get devices\n");
		exit(1);
	}
	for (int i=0;i<numDevices;i++)
	{
		struct libusb_device_descriptor desc;
		libusb_get_device_descriptor(devices[i],&desc);
		struct libusb_device_handle *devHandle;
		if (libusb_open(devices[i],&devHandle)) {
			perror("Can't open device");
			exit(1);
		}
		unsigned char manu[255] = {0};
		unsigned char prod[255] = {0};
		int ret = libusb_get_string_descriptor_ascii(devHandle,desc.iManufacturer,manu,255);
		ret = libusb_get_string_descriptor_ascii(devHandle,desc.iProduct,prod,255);
		libusb_close(devHandle);
		printf("%.4x:%.4x (%s,%s)\n",desc.idVendor, desc.idProduct,manu,prod);
	}

	libusb_free_device_list(devices, 1);

    libusb_exit(ctxt);

	return (0);
}