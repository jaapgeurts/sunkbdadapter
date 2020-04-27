/* Linux */
#include <linux/hidraw.h>
#include <libudev.h>

/* Unix */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* C */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define SUN_TYPE6_KBD_VID 0x0430
#define SUN_TYPE6_KBD_PID 0x0005

struct udev_device *find_keyboard(struct udev *udev, uint16_t vid, uint16_t pid);

int main(int argc, char *argv[])
{
    struct udev *udev;

    struct udev_device *dev;

    /* Create the udev object */
    udev = udev_new();
    if (!udev)
    {
        fprintf(stderr, "Can't create udev\n");
        exit(1);
    }

    dev = find_keyboard(udev, SUN_TYPE6_KBD_VID, SUN_TYPE6_KBD_PID);
    if (dev == NULL)
    { 
        fprintf(stderr,"Can't find attached keyboard\n");
        udev_unref(udev);
        exit(1);
    }
    // open the device

    const char *devNodePath = udev_device_get_devnode(dev);
    printf("path: %s\n",devNodePath);
    int fd = open(devNodePath,O_WRONLY | O_APPEND);
    if (fd == -1)
    {
        perror("Can't open keyboard hidraw device\n");
        udev_device_unref(dev);
        udev_unref(udev);
        exit(1);
    }

    udev_unref(udev);
    return 0;
}

struct udev_device *find_keyboard(struct udev *udev, uint16_t vid, uint16_t pid)
{
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;

    enumerate = udev_enumerate_new(udev);
    // search all hidraw devices
    udev_enumerate_add_match_subsystem(enumerate, "hidraw");
    udev_enumerate_scan_devices(enumerate);

    devices = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(dev_list_entry, devices)
    {
        dev = NULL;
        const char *path;

        /* Get the filename of the /sys entry for the device
        and create a udev_device object (dev) representing it */
        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);
        printf("Device Node Path: %s\n\t(%s)\n", udev_device_get_devnode(dev), path);

        
        // find the first usb_device. This will return the vid & pid to match.
        struct udev_device* parent_dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if (parent_dev)
        {
            /* From here, we can call get_sysattr_value() for each file
                in the device's /sys entry. The strings passed into these
                functions (idProduct, idVendor, serial, etc.) correspond
                directly to the files in the directory which represents
                the USB device. Note that USB strings are Unicode, UCS2
                encoded, but the strings returned from
                udev_device_get_sysattr_value() are UTF-8 encoded. */
            const char *vidStr = udev_device_get_sysattr_value(parent_dev, "idVendor");
            const char *pidStr = udev_device_get_sysattr_value(parent_dev, "idProduct");
            uint16_t lVid = strtoull(vidStr, NULL, 16);
            uint16_t lPid = strtoull(pidStr, NULL, 16);
            if (vid == lVid && pid == lPid)
            {
                printf("Found: VID/PID: %.4x %.4x\n", vid, pid);
                printf("\t%s\t%s\n",
                    udev_device_get_sysattr_value(parent_dev,"manufacturer"),
                    udev_device_get_sysattr_value(parent_dev,"product"));
                printf("\tserial: %s\n", udev_device_get_sysattr_value(parent_dev, "serial"));
                break;
            }
        }
    }
    /* Free the enumerator object */
    udev_enumerate_unref(enumerate);

    return dev;
}