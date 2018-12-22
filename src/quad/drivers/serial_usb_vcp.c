
#include "io.h"
#include "serial_usb_vcp.h"
#include "usbd_desc.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "utils.h"
#include "system.h"
//#include "usbd_cdc_vcp.h"

#define USB_TIMEOUT			50

extern uint8_t usbIsConfigured(void);  // HJI
extern uint8_t usbIsConnected(void);   // HJI
extern uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength);
extern uint32_t CDC_Receive_BytesAvailable(void);
extern uint32_t CDC_Send_FreeBytes(void);
extern uint32_t CDC_Receive_DATA(uint8_t* recvBuf, uint32_t len);       // HJI
extern uint32_t CDC_BaudRate(void);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

static vcpPort_t vcpPort;

static bool usbVcpFlush(vcpPort_t *port)
{
    uint32_t count = port->txAt;
    port->txAt = 0;

    if (count == 0) {
        return true;
    }

    if (!usbIsConnected() || !usbIsConfigured()) {
        return false;
    }

    uint32_t start = millis();
    uint8_t *p = port->txBuf;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
    return count == 0;
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

    return CDC_Receive_BytesAvailable();
}

uint32_t usbTxBytesFree(const serialPort_t *instance)
{
	UNUSED(instance);
    return CDC_Send_FreeBytes();
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t buf[1];

    while (true) {
        if (CDC_Receive_DATA(buf, 1))
            return buf[0];
    }
}

static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);

    // TODO implement
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static void usbVcpSetMode(serialPort_t *instance, portMode_t mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    uint32_t start = millis();
    const uint8_t *p = data;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};

serialPort_t *usbVcpOpen(void)
{
	vcpPort_t *s;
	
	IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);		// IO_TAG(PA11) = 0x1B
	IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);		// IO_TAG(PA12) = 0x1C
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
	
	s = &vcpPort;
	s->port.vTable = usbVTable;
	
	return (serialPort_t *)s;
}
