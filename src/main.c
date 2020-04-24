#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define MAX_BUF 255

unsigned char buf[MAX_BUF];

typedef struct
{
  uint8_t  reportId;                                 // Report ID = 0x01 (1)
  uint8_t  VEN_VendorDefined0001[64];                // FF00 0001  Value = 0 to 255
} inputReport01_t;

typedef struct
{
  uint8_t  reportId;                                 // Report ID = 0x02 (2)
  uint8_t  VEN_VendorDefined0001[64];                // FF00 0001  Value = 0 to 255
} outputReport02_t;

int main()
{
	printf("Sun Type 6 UART Keyboard tool\n");

	struct libusb_context* ctxt;

    if (libusb_init(&ctxt)) {
		perror("Can't open libusb");
		exit(1);
	}

	struct libusb_device_handle* handle;

	handle = libusb_open_device_with_vid_pid(ctxt, 0x0430,0005);
	if (handle == NULL)
	{
		perror("Can't open device");
		exit(1);
	}
	
	/*if (libusb_reset_device(handle))
	{
		perror("Can't reset device");
		exit(1);
	}
	*/
	struct libusb_device* device = libusb_get_device(handle);
	struct libusb_config_descriptor* config;
	libusb_get_active_config_descriptor(device,&config);
	libusb_get_string_descriptor_ascii(handle, config->iConfiguration,buf,MAX_BUF);
	printf("Configuration: %s\n", buf);
	for (int i=0;i<config->bNumInterfaces; i++)
	{
		
		for(int k=0;k<config->interface[i].num_altsetting;k++)
		{
			libusb_get_string_descriptor_ascii(handle, config->interface[i].altsetting[k].iInterface,buf,MAX_BUF);
			printf("Interface[%d][%d]: %s\n",i,k,buf);
			printf("Extra: %d\n",config->interface[i].altsetting[k].extra_length); 
			// for (int l=0;l<config->interface[i].altsetting[k].extra_length;l++)
			// {
			// 	printf("Extra: %d",config->interface[i].altsetting[k].extra
			// }
			for (int l=0;l<config->interface[i].altsetting[k].bNumEndpoints;l++)
			{
				printf("%d\n",config->interface[i].altsetting[k].endpoint[l].bEndpointAddress);
			}
		}
		
	}

	if (libusb_detach_kernel_driver(handle,0))
	{
		perror("Can't detatch");
		exit(1);
	}
	if (libusb_claim_interface(handle,0))
	{
		perror("Can't claim");
		exit(1);
	}

	//libusb_set_configuration(handle,0);

    uint8_t data[] = {0x0f};

#define HID_SET_REPORT 0x09	
int reportId = 0;
	int outsize;
	//if (libusb_interrupt_transfer(handle,LIBUSB_ENDPOINT_OUT, &data,sizeof(data),&outsize, 5000)) {
	outsize = libusb_control_transfer(handle,LIBUSB_ENDPOINT_OUT|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_REQUEST_TYPE_CLASS,HID_SET_REPORT, reportId,0,data,sizeof(data), 5000);
	if (outsize < 1) {
		perror("Can't send");
		libusb_release_interface(handle,0);
		libusb_attach_kernel_driver(handle,0);
		exit(1);
	}
	printf("Sent %d bytes\n",outsize);

	sleep(5);

	libusb_release_interface(handle,0);
	libusb_attach_kernel_driver(handle,0);
	libusb_close(handle);
	// send report

    libusb_exit(ctxt);

	return (0);
}

void show_all(struct libusb_context* ctxt)
{
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
		libusb_get_string_descriptor_ascii(devHandle,desc.iManufacturer,manu,255);
		libusb_get_string_descriptor_ascii(devHandle,desc.iProduct,prod,255);
		libusb_close(devHandle);
		printf("%.4x:%.4x (%s,%s)\n",desc.idVendor, desc.idProduct,manu,prod);
	}
	libusb_free_device_list(devices, 1);
}