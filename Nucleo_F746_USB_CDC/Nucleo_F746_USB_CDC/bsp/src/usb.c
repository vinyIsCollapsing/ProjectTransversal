/*
 * usb.c
 *
 *  Created on: Nov 23, 2023
 *      Author: Laurent
 */

#include "usb.h"



#define USB_OTG_FS_INEP(i)    ((USB_OTG_INEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USB_OTG_FS_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))

/*
 * Local static functions
 */

static void 	USB_Reset_Event_Handler		(void);
static void 	USB_Setup_Packet_Handler	(void);

static void 	USB_FIFO_Read				(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt);
static void 	USB_FIFO_Write				(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt);



/*
 * Global variables
 */

USB_OTG_GlobalTypeDef * const USB_OTG_FS_GLOBAL  = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
USB_OTG_DeviceTypeDef * const USB_OTG_FS_DEVICE  = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);

usb_packet_t		g_usb_packet;			// Incoming packet info
usb_setup_packet_t 	g_usb_setup_packet;		// Setup packet info

usb_in_ctrl_t 		g_usb_in_ctrl;			// IN  control Endpoint
usb_out_ctrl_t 		g_usb_out_ctrl;			// OUT control Endpoint

usb_cdc_t 			g_usb;					// USB CDC data structure



/*
 * USB CDC (VIRTUAL COM PORT) DESCRIPTORS
 */

#define BCD_DEVICE  					0x0200
#define VENDOR_ID   					0x0483			// ST Microelectronics
#define PRODUCT_ID  					0x5740			// ST Virtual COM port

#define USB_DESCTYPE_DEVICE				0x01
#define	USB_DESCTYPE_CONFIGURATION		0x02
#define USB_DESCTYPE_INTERFACE			0x04
#define USB_DESCTYPE_ENDPOINT			0x05


#define USB_CLASS_COMMUNICATIONS		0x02
#define USB_CDC_ACM_SUBCLASS			0x02
#define USB_CDC_AT_COMMAND_PROTOCOL 	0x01

#define USB_DESCTYPE_CS_INTERFACE		0x24

#define CDC_NOTIFICATION_EP_NUM			0x02
#define USB_CLASS_CDC_DATA				0x0A

#define CDC_DATA_RX_EP_NUM				0x01
#define CDC_DATA_TX_EP_NUM				0x01

#define USBD_VID     					1155
#define USBD_LANGID_STRING     			1033
#define USBD_MANUFACTURER_STRING     	"STMicroelectronics"
#define USBD_PID_FS     				22336
#define USBD_PRODUCT_STRING_FS     		"STM32 Virtual ComPort"
#define USBD_SERIALNUMBER_STRING_FS     "00000000001A"
#define USBD_CONFIGURATION_STRING_FS    "CDC Config"
#define USBD_INTERFACE_STRING_FS     	"CDC Interface"



/* USB CDC Device Descriptor - 18 bytes */

uint8_t devDesc[] =
{
		0x12,                  		/* 0  bLength 									*/
		USB_DESCTYPE_DEVICE,   		/* 1  bdescriptorType - Device 					*/
		0x00,                  		/* 2  bcdUSB version 							*/
		0x02,                  		/* 3  bcdUSB version 							*/
		USB_CLASS_COMMUNICATIONS,	/* 4  bDeviceClass - USB CDC Class 				*/
		0x02,                  		/* 5  bDeviceSubClass  - Specified by interface */
		0x00,                  		/* 6  bDeviceProtocol  - Specified by interface */
		0x40,                  		/* 7  bMaxPacketSize for EP0 - max = 64 		*/
		(VENDOR_ID & 0xFF),    		/* 8  idVendor 									*/
		(VENDOR_ID >> 8),      		/* 9  idVendor 									*/
		(PRODUCT_ID & 0xFF),   		/* 10 idProduct 								*/
		(PRODUCT_ID >> 8),     		/* 11 idProduct 								*/
		(BCD_DEVICE & 0xFF),   		/* 12 bcdDevice 								*/
		(BCD_DEVICE >> 8),     		/* 13 bcdDevice 								*/
		0x01,                  		/* 14 iManufacturer - index of string 			*/
		0x02,                  		/* 15 iProduct  - index of string 				*/
		0x03,                  		/* 16 iSerialNumber  - index of string 			*/
		0x01                   		/* 17 bNumConfigurations 						*/
};


/* USB CDC String Descriptors */

// Index 0 -> Supported languages (always)
uint8_t strDesc0[]=
{
		0x04,                       		/* 0  bLength = 4 */
		0x03, 								/* 1  bDescriptortype - String */
		0x09, 0x04							/* 2  wLANGID 0x0409 = English US */
};

// Index 1 -> iManufacturer
// "STMicroelectronics"
// 530054004d006900630072006f0065006c0065006300740072006f006e00690063007300
uint8_t strDesc1[] =
{
		0x26,                       		/* 0  bLength = 38 */
		0x03, 								/* 1  bDescriptortype - String */
		0x53, 0x00, 0x54, 0x00,				/* "STMicroelectronics" */
		0x4d, 0x00, 0x69, 0x00,
		0x63, 0x00, 0x72, 0x00,
		0x6f, 0x00, 0x65, 0x00,
		0x6c, 0x00, 0x65, 0x00,
		0x63, 0x00, 0x74, 0x00,
		0x72, 0x00, 0x6f, 0x00,
		0x6e, 0x00, 0x69, 0x00,
		0x63, 0x00, 0x73, 0x00
};

// Index 2 -> iProduct
// "STM32 Virtual ComPort"
// 530054004d003300320020005600690072007400750061006c00200043006f006d0050006f0072007400
uint8_t strDesc2[] =
{
		0x2C,                       		/* 0  bLength = 44 */
		0x03, 								/* 1  bDescriptortype - String */
		0x53, 0x00, 0x54, 0x00,				/* "STM32 Virtual ComPort" */
		0x4d, 0x00, 0x33, 0x00,
		0x32, 0x00, 0x20, 0x00,
		0x56, 0x00, 0x69, 0x00,
		0x72, 0x00, 0x74, 0x00,
		0x75, 0x00, 0x61, 0x00,
		0x6c, 0x00, 0x20, 0x00,
		0x43, 0x00, 0x6f, 0x00,
		0x6d, 0x00, 0x50, 0x00,
		0x6f, 0x00, 0x72, 0x00,
		0x74, 0x00
};


// Index 3 -> iSerial
// 00000000001A
// 300030003000300030003000300030003000300031004100
uint8_t strDesc3[] =
{
		0x1A,                       		/* 0  bLength = 26 */
		0x03, 								/* 1  bDescriptortype - String */
		0x30, 0x00, 0x30, 0x00,				/* "00000000001A" */
		0x30, 0x00, 0x30, 0x00,
		0x30, 0x00, 0x30, 0x00,
		0x30, 0x00, 0x30, 0x00,
		0x30, 0x00, 0x30, 0x00,
		0x31, 0x00, 0x41, 0x00
};


/*	USB CDC Configuration Descriptor - 67 bytes */

uint8_t cfgDesc[] =
{
		0x09,                       		/* 0  bLength */
		USB_DESCTYPE_CONFIGURATION, 		/* 1  bDescriptortype - Configuration */
		0x43, 0x00,                 		/* 2  wTotalLength	*/
		0x02,                       		/* 4  bNumInterfaces */
		0x01,                       		/* 5  bConfigurationValue */
		0x00,                       		/* 6  iConfiguration - index of string */
		0xC0,                       		/* 7  bmAttributes - Self powered */
		0x32,                       		/* 8  bMaxPower - 100mA */

		/* CDC Communication interface */
		0x09,                       		/* 0  bLength */
		USB_DESCTYPE_INTERFACE,     		/* 1  bDescriptorType - Interface */
		0x00,                       		/* 2  bInterfaceNumber - Interface 0 */
		0x00,                       		/* 3  bAlternateSetting */
		0x01,                       		/* 4  bNumEndpoints */
		USB_CLASS_COMMUNICATIONS,   		/* 5  bInterfaceClass */
		USB_CDC_ACM_SUBCLASS,       		/* 6  bInterfaceSubClass - Abstract Control Model */
		USB_CDC_AT_COMMAND_PROTOCOL,		/* 7  bInterfaceProtocol - AT Command V.250 protocol */
		0x00,                       		/* 8  iInterface - No string descriptor */

		/* Header Functional descriptor */
		0x05,                      			/* 0  bLength */
		USB_DESCTYPE_CS_INTERFACE, 			/* 1  bDescriptortype, CS_INTERFACE */
		0x00,                      			/* 2  bDescriptorsubtype, HEADER */
		0x10, 0x01,                			/* 3  bcdCDC */

		/* Call Management Functional descriptor */
		0x05,                     			/* 0  bLength */
		USB_DESCTYPE_CS_INTERFACE,			/* 1  bDescriptortype, CS_INTERFACE */
		0x01,                     			/* 2  bDescriptorsubtype, CALL MANAGEMENT */
		0x00,                     			/* 3  bmCapabilities */
		0x01,                     			/* 4  bDataInterface */

		/* ACM Functional descriptor */
		0x04,                      			/* 0  bLength */
		USB_DESCTYPE_CS_INTERFACE, 			/* 1  bDescriptortype, CS_INTERFACE */
		0x02,                      			/* 2  bDescriptorsubtype, ABSTRACT CONTROL MANAGEMENT */
		0x02,        						/* 3  bmCapabilities: Supports subset of ACM commands */

		/* Union Functional descriptor */
		0x05,                     			/* 0  bLength */
		USB_DESCTYPE_CS_INTERFACE,			/* 1  bDescriptortype, CS_INTERFACE */
		0x06,                     			/* 2  bDescriptorsubtype, UNION */
		0x00,                     			/* 3  bControlInterface - Interface 0 */
		0x01,                    			/* 4  bSubordinateInterface0 - Interface 1 */

		/* Notification Endpoint descriptor */
		0x07,                         		/* 0  bLength */
		USB_DESCTYPE_ENDPOINT,        		/* 1  bDescriptorType */
		(CDC_NOTIFICATION_EP_NUM | 0x80),	/* 2  bEndpointAddress */
		0x03,                         		/* 3  bmAttributes */
		0x08,                         		/* 4  wMaxPacketSize - Low */
		0x00,                         		/* 5  wMaxPacketSize - High */
		0x10,                         		/* 6  bInterval */

		/* CDC Data interface */
		0x09,                     			/* 0  bLength */
		USB_DESCTYPE_INTERFACE,   			/* 1  bDescriptorType */
		0x01,                     			/* 2  bInterfacecNumber */
		0x00,                     			/* 3  bAlternateSetting */
		0x02,                     			/* 4  bNumEndpoints */
		USB_CLASS_CDC_DATA,       			/* 5  bInterfaceClass */
		0x00,                     			/* 6  bInterfaceSubClass */
		0x00,                     			/* 7  bInterfaceProtocol */
		0x00,                     			/* 8  iInterface - No string descriptor */

		/* Data OUT Endpoint descriptor */
		0x07,                     			/* 0  bLength */
		USB_DESCTYPE_ENDPOINT,    			/* 1  bDescriptorType */
		CDC_DATA_RX_EP_NUM,       			/* 2  bEndpointAddress */
		0x02,                     			/* 3  bmAttributes */
		0x40,                     			/* 4  wMaxPacketSize - Low */
		0x00,                     			/* 5  wMaxPacketSize - High */
		0x00,                     			/* 6  bInterval */

		/* Data IN Endpoint descriptor */
		0x07,                     			/* 0  bLength */
		USB_DESCTYPE_ENDPOINT,    			/* 1  bDescriptorType */
		(CDC_DATA_TX_EP_NUM | 0x80),		/* 2  bEndpointAddress */
		0x02,                     			/* 3  bmAttributes */
		0x40,                     			/* 4  wMaxPacketSize - Low byte */
		0x00,                     			/* 5  wMaxPacketSize - High byte */
		0x00                      			/* 6  bInterval */
};


/* USB Device Qualifier Descriptor */

uint8_t devQualDesc[] =
{
		0x0A,								/* 0 bLength */
		0x06,								/* 1 bDescriptorType */
		0x00, 0x02,							/* 2-3 bcdUSB USB Specification Release Number in BCD (2.00) */
		0x00,								/* 4 bDeviceClass */
		0x00,								/* 5 bDeviceSubClass */
		0x00,								/* 6 bDeviceProtocol */
		0x40,								/* 7 bMaxPacketSize0 (64 bytes) */
		0x01,								/* 8 bNumConfigurations */
		0x00								/* 9 bReserved */
};


/*	USB CDC Line Coding */

uint8_t usb_cdc_line_coding[] =
{
		0x00, 0x96, 0x00, 0x00,				/* 00009600 -> 38400 bauds  */
		0x00,								/* 1 Stop bit				*/
		0x00,								/* No parity				*/
		0x08								/* 8-bit					*/
};


/*
 * USB Send function
 */

void BSP_USB_Send(uint8_t *msg, uint8_t length)
{
	uint8_t i;

	// Copy message into g_usb TX buffer
	for (i=0; i<length; i++)
	{
		g_usb.tx_buffer[i] = msg[i];
	}

	// Set number of bytes to transmit
	g_usb.tx_nbytes = length;

	// Setup IN EP1 for transmission
    USB_OTG_FS_INEP(1)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
    USB_OTG_FS_INEP(1)->DIEPTSIZ |= 1U <<19U;

    USB_OTG_FS_INEP(1)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// nbytes
    USB_OTG_FS_INEP(1)->DIEPTSIZ |= USB_OTG_DIEPTSIZ_XFRSIZ & g_usb.tx_nbytes;

    // Enable device IN EP FIFO empty (TXFE) interrupt -> This is done to wait for available space in the TX FIFO
    USB_OTG_FS_DEVICE->DIEPEMPMSK |= 1U <<1U;	// Enable IN EP1 TXFE interrupts

    // Enable IN EP1 (automatically disabled after previous transfer completed)
    USB_OTG_FS_INEP(1)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
}



/*
 * USB Core initialization for Device Mode
 */
void BSP_USB_Core_Init()
{
	// USB_OTG_FS GPIO Configuration
	// PA8		-> USB_OTG_FS_SOF	(AF10)
	// PA10     -> USB_OTG_FS_ID	(AF10)
	// PA11     -> USB_OTG_FS_DM	(AF10)
	// PA12     -> USB_OTG_FS_DP 	(AF10)
	// PA9		-> USB_OTG_FS_VBUS

	uint32_t	i;

	// Enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// Configure PA8, PA10, PA11, PA12 as AF mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
	GPIOA->MODER |=  (0x02 <<16U) | (0x02 <<20U) | (0x02 <<22U) | (0x02 <<24U);

	// Set to push-pull outputs
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT8_Msk | GPIO_OTYPER_OT10_Msk | GPIO_OTYPER_OT11_Msk | GPIO_OTYPER_OT12_Msk);

	// Set to very high speed
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);
	GPIOA->OSPEEDR |=  (0x03 <<16U) | (0x03 <<20U) | (0x03 <<22U) | (0x03 <<24U);

	// No pull resistors
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR10_Msk | GPIO_PUPDR_PUPDR11_Msk | GPIO_PUPDR_PUPDR12_Msk);

	// Connect to USB_OTG_FS (AF10)
	GPIOA->AFR[1] &= ~(0x000FFF0F);
	GPIOA->AFR[1] |=   0x000AAA0A;

	// Configure PA9 as input
	GPIOA->MODER &= ~(GPIO_MODER_MODER9);
	GPIOA->MODER |=  (0x00 <<18U);


	// USB Global Core Configuration

	// Start USB_OTG_FS clock
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

	// Global AHB USB Configuration
	// - Global USB interrupts are disabled
	// - TXFE signals that TX FIFO is completely empty
	USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	USB_OTG_FS->GAHBCFG |=  USB_OTG_GAHBCFG_TXFELVL;

	// NVIC enable USB_OTG_FS interrupts
	NVIC_SetPriority(OTG_FS_IRQn, 1);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	// Select internal PHY interface layer (actually always set)
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

	// Wait for AHB master state machine to be in IDLE state
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

	// Core soft reset
	USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

	// Wait (again) for AHB master state machine to be in IDLE state
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

	// Deactivate power down (i.e. transceiver becomes active)
	USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN;

	// Force DEVICE mode (no matter what the ID input pin is)
	USB_OTG_FS->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

	delay_ms(50);

	// Deactivate VBUS Sensing B
    // USB_OTG_FS->GCCFG &= ~ USB_OTG_GCCFG_VBUSBSEN;

    // Activate VBUS Sensing B
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBDEN;

    // Restart the Phy Clock
    *(__IO uint32_t *)((uint32_t)USB_OTG_FS + USB_OTG_PCGCCTL_BASE) = 0U;



	// USB DEVICE mode configuration

	// Set periodic frame interval to 80%
	USB_OTG_FS_DEVICE->DCFG &= ~(USB_OTG_DCFG_PFIVL_Msk);
	USB_OTG_FS_DEVICE->DCFG |= 0x00 <<USB_OTG_DCFG_PFIVL_Pos;

	// Set to full speed
	USB_OTG_FS_DEVICE->DCFG |= 0x03 <<USB_OTG_DCFG_DSPD_Pos;

	// Flush all TX FIFOs
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x10 <<USB_OTG_GRSTCTL_TXFNUM_Pos);
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	// Flush RX FIFO
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

	// Mask all IN EP interrupts
	USB_OTG_FS_DEVICE->DIEPMSK 	= 0x00000000U;

	// Mask all OUT EP interrupts
	USB_OTG_FS_DEVICE->DOEPMSK 	= 0x00000000U;

	// Mask all EP interrupts
	USB_OTG_FS_DEVICE->DAINTMSK = 0x00000000U;


	// For all IN EP (0, 1..5)
	for (i=0; i<6; i++)
	{
		// If the EP is currently enabled
	    if ((USB_OTG_FS_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
	    {
	    	// Disable the EP and set NAK
	    	USB_OTG_FS_INEP(i)->DIEPCTL = (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
	    }
	    else
	    {
	    	// Reset EP control register
	    	USB_OTG_FS_INEP(i)->DIEPCTL = 0x00000000U;
	    }

	    // Set IN EP size to zero
	    USB_OTG_FS_INEP(i)->DIEPTSIZ = 0x00000000U;

	    // Clear ALL IN EP interrupts flags
	    USB_OTG_FS_INEP(i)->DIEPINT |=  0x0000287BU;
	}

	// For all OUT EP (0, 1..5)
	for (i=0; i<6; i++)
	{
		// If the EP is currently enabled
		if ((USB_OTG_FS_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)	// Passe dans le else 4 fois
		{
			// Disable the EP and set NAK
			USB_OTG_FS_OUTEP(i)->DOEPCTL = (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
	    }
	    else
	    {
	    	// Reset EP control register
	    	USB_OTG_FS_OUTEP(i)->DOEPCTL = 0x00000000U;
	    }

		// Set OUT EP size to zero
		USB_OTG_FS_OUTEP(i)->DOEPTSIZ = 0x00000000U;

		 // Clear ALL OUT EP interrupt flags
		USB_OTG_FS_OUTEP(i)->DOEPINT  |= 0x0000313BU;
	}

	// Disable all interrupts
	USB_OTG_FS->GINTMSK = 0x00000000U;

	// Clear ALL pending interrupts
	USB_OTG_FS->GINTSTS |= 0xF030FC0AU;

	// Enable the common interrupts
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

	// Enable interrupts matching to the Device mode ONLY
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM;				// USB SUSPEND event
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST;					// USB RESET event
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ENUMDNEM;				// USB ENUMERATION done event
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_IEPINT;					// IN EP event

	// Soft-Disconnect USB device by disabling pull-up/pull-down
	USB_OTG_FS_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
	delay_ms(10);

	// Set RX FIFO size to 0x80 = 128 words = 512 bytes
	USB_OTG_FS->GRXFSIZ = (uint32_t)0x80;

	// Set TX FIFO for IN EP0 to 0x40 = 64 words = 256 bytes
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ( (uint32_t)0x40 <<16U | 0x80);

	// Set TX FIFO for IN EP1 to 0x80 = 128 words = 512 bytes				--> Total = 512 + 256 + 512 = 1280 bytes = 1,25kB
	USB_OTG_FS->DIEPTXF[0] = ((uint32_t)0x80 <<16U | (0x80 + 0x40) );

	// Soft-Connect USB device by enabling pull-up/pull-down
	USB_OTG_FS_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS ;
	delay_ms(10);

	// Enable global interrupts
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

	// Perform Reset initializations
	USB_Reset_Event_Handler();
}




/**********************************************************************
 *                                                                    *
 * USB_OTG_FS Interrupt Handler (this is where everything is done !)  *
 *                                                                    *
 **********************************************************************
 */

void OTG_FS_IRQHandler(void)
{
	uint32_t		temp;				// Temporary register
	uint32_t 		*p_fifo;
	uint16_t		i, k;


	// Deal with USB RESET event
	if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) == USB_OTG_GINTSTS_USBRST)
	{
		// Reset USB core
		USB_Reset_Event_Handler();

	    // Clear USBRST interrupt flag
	    USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST;

	    my_printf("RESET\r\n");
	}


	// Deal with USB SUSPEND event
	if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBSUSP) == USB_OTG_GINTSTS_USBSUSP)
	{
		// Clear USBSUSP flag
		USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;

		// Disable the USB SUSPEND event interrupt (to avoid multiple SUSPEND interrupts)
		USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_USBSUSPM;

		my_printf("SUSPEND\r\n");
	}


	// Deal with USB ENUMERATION DONE event
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    {
       	// Clear ENUMDNE flag
    	USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

    	// Re-enable the USB SUSPEND event interrupt
    	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM;

    	my_printf("ENUM DONE\r\n");
    }


	// Deal with data RX event
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == USB_OTG_GINTSTS_RXFLVL)
	{
		// Read RX status
		temp = USB_OTG_FS->GRXSTSR;

		// Retrieve the number of received bytes
		g_usb_packet.bcnt 	= (uint16_t)( (temp & USB_OTG_GRXSTSP_BCNT_Msk) >>USB_OTG_GRXSTSP_BCNT_Pos);
		g_usb_packet.status = (int8_t)  ( (temp & USB_OTG_GRXSTSP_PKTSTS_Msk) >>USB_OTG_GRXSTSP_PKTSTS_Pos);
		g_usb_packet.epnum	= (int8_t)  ( (temp & USB_OTG_GRXSTSP_EPNUM_Msk) >>USB_OTG_GRXSTSP_EPNUM_Pos);

		// pop FIFO
		USB_OTG_FS->GRXSTSP;

		my_printf("PK %d on EP%d of %d bytes : ", g_usb_packet.status, g_usb_packet.epnum, g_usb_packet.bcnt);

		// If this is a received SETUP packet (8 bytes received on EP0)
		if (g_usb_packet.status == USB_PKT_STATUS_SETUP)
		{
			my_printf("[SETUP]");

			// Handle SETUP packet
			USB_Setup_Packet_Handler();

			// Clear NAK bit and set EPENA to start transmission on EP0 (this terminates the SETUP transaction)
			USB_OTG_FS_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
		}


		// If this is a SETUP transaction completed
		if (g_usb_packet.status == USB_PKT_STATUS_SETUP_COMPLETE)
		{
			my_printf("[SETUP_COMPLETE]");

			// Clear NAK bit and set EPENA to start transmission on EP0
			USB_OTG_FS_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
		}


		// If this is an OUT transaction
		if (g_usb_packet.status == USB_PKT_STATUS_OUT)
		{
			my_printf("[OUT]");

			switch (g_usb_packet.epnum)
			{
				case 0 :	// EP0
				{
					my_printf("[EP0]");

					k = (g_usb_packet.bcnt >>2U);
					if ( (g_usb_packet.bcnt - (k <<2U)) > 0) k++;
					my_printf(" -> RECEIVING %d bytes in %d words ", g_usb_packet.bcnt, k);


					g_usb_out_ctrl.process = 1;

					// Retrieve DATA from OUT message
					p_fifo = (uint32_t*)0x50001000;
					USB_FIFO_Read(p_fifo, g_usb.rx_buffer, g_usb_packet.bcnt);

					// Print result
					for (i=0; i<g_usb_packet.bcnt; i++)
					{
						my_printf("[%02x] ", g_usb.rx_buffer[i]);
					}

					break;
				}

				case 1:		// EP1
				{
					my_printf("[EP1]");

					// Retrieve DATA from OUT message
					p_fifo = (uint32_t*)0x50002000;
					USB_FIFO_Read(p_fifo, g_usb.rx_buffer, g_usb_packet.bcnt);

					my_printf("\r\ndone\r\n");

					// Send HEX to console
					for (i=0; i<g_usb_packet.bcnt; i++)
					{
						my_printf("%02x ", g_usb.rx_buffer[i]);

						if ( (i>0) & ((i%32) == 0) ) my_printf("\r\n");
					}

					g_usb.rx_nbytes = g_usb_packet.bcnt;

					break;
				}

			}
		}

		// If this is an OUT transaction completed
		if (g_usb_packet.status == USB_PKT_STATUS_OUT_COMPLETE)
		{
			my_printf("[OUT_COMPLETE]");

			// Switch depending on end-point number
			switch(g_usb_packet.epnum)
			{
				case 0:			// EP0
				{
					my_printf("[EP0]");

					if (g_usb_out_ctrl.process == 1)
					{
						g_usb_out_ctrl.process = 0;

						// Disable device IN EP FIFO empty interrupt
						USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);
					}
					else
					{
						// Do nothing
						my_printf(" Ignored...");
					}

					break;
				}

				case 1:			// EP1
				{
					my_printf("[EP1]");
					my_printf("Re-enabling OUT EP1 for next reception");


					// Re-enable OUT EP1
					USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;

					// Clear OUT EP1 interrupt flags
					USB_OTG_FS_OUTEP(1)->DOEPINT |= 0xFF;

					// Clear NACK bit so that ACK is ready for next reception
					USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;

					break;
				}
			}
		}


		// Error catching...
		if (g_usb_packet.status == 0)
		{
			my_printf(" Weird...");

			// Flush RX FIFO
			USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
			while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
		}

		my_printf("\r\n");
	}


	// Deal with IN EP event
    if ( (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT) == USB_OTG_GINTSTS_IEPINT )
	{
		// If this is for EP0
		if ( (USB_OTG_FS_DEVICE->DAINT & 0x00000001) == 0x00000001)
		{
			my_printf("[IEP0]");

			if ( (USB_OTG_FS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_XFRC_Msk)   == USB_OTG_DIEPINT_XFRC)   my_printf("[XFRC]");
			if ( (USB_OTG_FS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_EPDISD_Msk) == USB_OTG_DIEPINT_EPDISD) my_printf("[EPDISD]");
			if ( (USB_OTG_FS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_TOC_Msk)    == USB_OTG_DIEPINT_TOC)    my_printf("[TOC]");
			if ( (USB_OTG_FS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_INEPNE_Msk) == USB_OTG_DIEPINT_INEPNE) my_printf("[INEPNE]");

			// if ( (USB_OTG_FS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_TXFE_Msk) == USB_OTG_DIEPINT_TXFE)

			if ( (USB_OTG_FS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_ITTXFE_Msk) == USB_OTG_DIEPINT_ITTXFE )
			{
				// my_printf("[TXFE]");
				my_printf("[ITTXFE]");

				// If there's at least one packet to send
				if (g_usb_in_ctrl.pcnt > 0)
				{
					my_printf(" -> Sending %d byte in %d packet", g_usb_in_ctrl.bcnt, g_usb_in_ctrl.pcnt);

					if (g_usb_in_ctrl.bcnt <= 64)
					{
						USB_FIFO_Write((uint32_t*)0x50001000, (uint8_t*)&g_usb_in_ctrl.buffer[g_usb_in_ctrl.pindex * 64], g_usb_in_ctrl.bcnt);
					}

					else
					{
						USB_FIFO_Write((uint32_t*)0x50001000, (uint8_t*)&g_usb_in_ctrl.buffer[g_usb_in_ctrl.pindex * 64], 64);
					}

					// Decrement packet counter
					g_usb_in_ctrl.pcnt--;

					// Prepare for next packet (if necessary)
					if (g_usb_in_ctrl.pcnt > 0)
					{
						// Increment packet index
						g_usb_in_ctrl.pindex++;

						// Update byte count for next transfer
						g_usb_in_ctrl.bcnt -= 64;

						// If there is only one packet, set number of bytes to transmit
						if (g_usb_in_ctrl.pcnt == 1)
						{
							USB_OTG_FS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;
						}

						// Otherwise, set byte count to maximum for first packet
						else
						{
							USB_OTG_FS_INEP(0)->DIEPTSIZ |= 64;
						}

						// Enable all IN EP interrupts
						USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_IEPINT;

						// Enable device IN EP FIFO empty interrupt
						USB_OTG_FS_DEVICE->DIEPEMPMSK |= 1U <<0U;

						// Enable IN EP0
						USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
					}
				}

				// If there's nothing to send
				else
				{
					my_printf(" -> Nothing to send");
				}


			}

			// Clear all EP0 IN interrupt flags
			USB_OTG_FS_INEP(0)->DIEPINT |= 0x0000287B;

			my_printf("\r\n");
		}


		// If this is for EP1
		if ( (USB_OTG_FS_DEVICE->DAINT & 0x00000002) == 0x00000002)
		{
			// my_printf("[IEP1]");
			// my_printf("[%08x]", USB_OTG_FS->GINTSTS);


	    	while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
	    	USART3->TDR = '1';

			if ( (USB_OTG_FS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_XFRC_Msk) == USB_OTG_DIEPINT_XFRC)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'X';

				USB_OTG_FS_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_XFRCM;

				// Transmission is done
				// The EP1 has been automatically disabled
			}

			if ( (USB_OTG_FS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_EPDISD_Msk) == USB_OTG_DIEPINT_EPDISD )
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'D';
			}

			if ( (USB_OTG_FS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_TOC_Msk) == USB_OTG_DIEPINT_TOC)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'T';
			}

			if ( (USB_OTG_FS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_INEPNE_Msk) == USB_OTG_DIEPINT_INEPNE)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'N';
			}

			if ( (USB_OTG_FS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_ITTXFE_Msk) == USB_OTG_DIEPINT_ITTXFE)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'I';
			}

			if ( (USB_OTG_FS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_TXFE_Msk) == USB_OTG_DIEPINT_TXFE)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'E';

				// Writing to the FIFO triggers the data transmission upon next IN token
				if (g_usb.tx_nbytes > 0)
				{
					// Copy data to send into EP1 TX FIFO
					p_fifo = (uint32_t*)0x50002000;
					USB_FIFO_Write(p_fifo, g_usb.tx_buffer, g_usb.tx_nbytes);

					// No more data to send
					g_usb.tx_nbytes = 0;

					USB_OTG_FS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;
				}

				// Disable IN EP1 FIFO empty (TXFE) interrupt
				USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<1U);
			}

			// Clear all EP1 IN interrupt flags
			USB_OTG_FS_INEP(1)->DIEPINT |= 0x0000287B;
		}
	}
}


/*
 * USB RESET event handler
 */
static void USB_Reset_Event_Handler(void)
{
	uint8_t		i;

	// Reset Remote Wake-up Signaling flag (don't know why...)
	USB_OTG_FS_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

	// Flush EP0 TX FIFO
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x00 <<USB_OTG_GRSTCTL_TXFNUM_Pos);
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	// Clear all EP Interrupt flags
    for (i=0; i<6; i++)
    {
    	USB_OTG_FS_INEP(i)->DIEPINT  |= 0x0000287BU;
    	USB_OTG_FS_OUTEP(i)->DOEPINT |= 0x0000313BU;
    }

    // Un-mask Interrupt for EP0 (IN/OUT)
    USB_OTG_FS_DEVICE->DAINTMSK |= 0x00010001U;

	// Un-mask Interrupt for OUT EP : SETUP | Transfet Compete | EP disable
    USB_OTG_FS_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);

	// Set Default Address to 0
    USB_OTG_FS_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

	// Setup EP0 to receive SETUP messages
    USB_OTG_FS_OUTEP(0)->DOEPTSIZ = 0U;
    USB_OTG_FS_OUTEP(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19U)) ;
    USB_OTG_FS_OUTEP(0)->DOEPTSIZ |= (3U * 8U);										// 3 packets of 8 bytes
    USB_OTG_FS_OUTEP(0)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;  					// 3 packets
}


/*
 * USB SETUP packet handler
 */
static void USB_Setup_Packet_Handler(void)
{
	uint8_t		rx_data[64];		// Buffer to store incoming packet data

	// Retrieve DATA from SETUP message
	USB_FIFO_Read((uint32_t *)0x50001000, rx_data, g_usb_packet.bcnt);

	// Parse SETUP packet
	g_usb_setup_packet.bmRequestType = rx_data[0];
	g_usb_setup_packet.bRequest		 = rx_data[1];
	g_usb_setup_packet.wValue		 = rx_data[3] <<8U | rx_data[2];
	g_usb_setup_packet.wIndex		 = rx_data[5] <<8U | rx_data[4];
	g_usb_setup_packet.wLength		 = rx_data[7] <<8U | rx_data[6];


	/* STANDARD REQUESTS */

	if ((g_usb_setup_packet.bmRequestType == 0x00) || (g_usb_setup_packet.bmRequestType == 0x80))
	{

		// Switch depending on request
		switch(g_usb_setup_packet.bRequest)
		{
			// 														GET DESCRIPTOR
			case USB_REQUEST_GET_DESCRIPTOR:
			{
				my_printf("[GET DESCRIPTOR]");

				// Switch depending on requested descriptor type (MSB of wValue)
				switch((g_usb_setup_packet.wValue & 0xFF00) >>8U)
				{
					//														DEVICE
					case USB_DESCRIPTOR_DEVICE:
					{
						my_printf("[DEVICE]");

						// Setup IN control structure for Device Descriptor sending
						g_usb_in_ctrl.epnum  = 0;
						g_usb_in_ctrl.pcnt   = 1;
						g_usb_in_ctrl.bcnt   = 18;
						g_usb_in_ctrl.pindex = 0;
						g_usb_in_ctrl.buffer = (uint8_t*)devDesc;


						break;
					}

					//													CONFIGURATION
					case USB_DESCRIPTOR_CONFIGURATION:
					{
						my_printf("[CONFIGURATION %d]", g_usb_setup_packet.wLength);

						// Setup IN control structure for Configuration Descriptor sending
						if (g_usb_setup_packet.wLength <64)
						{
							g_usb_in_ctrl.epnum  = 0;
							g_usb_in_ctrl.pcnt   = 1;
							g_usb_in_ctrl.bcnt   = g_usb_setup_packet.wLength;
							g_usb_in_ctrl.pindex = 0;
							g_usb_in_ctrl.buffer = (uint8_t*)cfgDesc;
						}
						else
						{
							g_usb_in_ctrl.epnum  = 0;
							g_usb_in_ctrl.pcnt   = 2;
							g_usb_in_ctrl.bcnt   = 67;
							g_usb_in_ctrl.pindex = 0;
							g_usb_in_ctrl.buffer = (uint8_t*)cfgDesc;
						}

						break;
					}

					//														STRING
					case USB_DESCRIPTOR_STRING:
					{
						my_printf("[STRING %d]", (uint8_t)(g_usb_setup_packet.wValue & 0x00FF));

						// Setup IN control structure for Configuration Descriptor sending
						g_usb_in_ctrl.epnum  = 0;
						g_usb_in_ctrl.pcnt   = 1;
						g_usb_in_ctrl.pindex = 0;

						// Switch depending on STRING index
						switch ((uint8_t)(g_usb_setup_packet.wValue & 0x00FF))
						{
							case 0:						// STRING DESCRIPTOR [0]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc0))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc0);		// 4 bytes
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc0;

								break;
							}

							case 1:						// STRING DESCRIPTOR [1]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc1))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc1);		// 38 bytes
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc1;

								break;
							}

							case 2:						// STRING DESCRIPTOR [2]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc2))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc2);		// 44 bytes
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc2;

								break;
							}

							case 3:						// STRING DESCRIPTOR [3]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc3))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc3);		// 26 bytes
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc3;

								break;
							}
						}

						break;
					}

					//													DEVICE QUALIFIER
					case USB_DESCRIPTOR_QUALIFIER:
					{
						my_printf(" [DEVICE QUALIFIER]");

						// Setup IN control structure for Device Qualifier Descriptor sending
						g_usb_in_ctrl.epnum  = 0;
						g_usb_in_ctrl.pcnt   = 1;
						g_usb_in_ctrl.bcnt   = sizeof(devQualDesc);					// 10 bytes
						g_usb_in_ctrl.pindex = 0;
						g_usb_in_ctrl.buffer = (uint8_t*)devQualDesc;

						break;
					}
				}

				// Prepare IN EP0 for transmission
				USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// packets number
				USB_OTG_FS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt <<19U;

				USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// bytes number

				// If there is only one packet, set number of bytes to transmit
				if (g_usb_in_ctrl.pcnt == 1)
				{
					USB_OTG_FS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;
				}

				// Otherwise, set byte count to maximum for first packet
				else
				{
					USB_OTG_FS_INEP(0)->DIEPTSIZ |= 64;
				}

				// Enable device IN EP FIFO empty interrupt
				USB_OTG_FS_DEVICE->DIEPEMPMSK |= 1U <<0U;

				// Enable IN EP0
				USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

				break;
			}

			//													SET ADDRESS
			case USB_REQUEST_SET_ADDRESS:
			{
				// Address is LSB of wValue
				my_printf("[SET ADRESS %d]", (uint8_t)(g_usb_setup_packet.wValue & 0x00FF));

				// Set Device address
				USB_OTG_FS_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD_Msk;
				USB_OTG_FS_DEVICE->DCFG |= (uint8_t)(g_usb_setup_packet.wValue & 0x00FF) <<USB_OTG_DCFG_DAD_Pos;

				g_usb_in_ctrl.pcnt   = 1;
				g_usb_in_ctrl.bcnt   = 0;

				// Setup IN EP0 for transmission
				USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
				USB_OTG_FS_INEP(0)->DIEPTSIZ |= 1U <<19U;

				USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// 0 bytes
				USB_OTG_FS_INEP(0)->DIEPTSIZ |= 0;

				// Disable device IN EP FIFO empty interrupt
				USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

				// Enable IN EP0
				USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

				break;
			}


			// 													SET CONFIGURATION
			case USB_REQUEST_SET_CONFIGURATION:
			{
				my_printf("[SET CONFIGURATION]");

				/*
				 *  Configure IN EP1
				 */

				// Clear all IN EP(1) interrupts flags
				USB_OTG_FS_INEP(1)->DIEPINT |=  0x0000287BU;

				// Enable all IN EP Transfer Complete (XFRC) interrupt (this one only)
				USB_OTG_FS_DEVICE->DIEPMSK  &= ~0x0000207BU;
				// USB_OTG_FS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;

			    // Un-mask general interrupts from IN EP1
			    USB_OTG_FS_DEVICE->DAINTMSK |= 0x00000002U;

			    // Setup IN EP1 in BULK mode (without enabling it at that time)
				USB_OTG_FS_INEP(1)->DIEPCTL = 0x00000000U;
				USB_OTG_FS_INEP(1)->DIEPCTL |= 2 <<USB_OTG_DIEPCTL_EPTYP_Pos;			// Bulk
				USB_OTG_FS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;					// USB Active
				USB_OTG_FS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;			// Set DATA0 PID
				USB_OTG_FS_INEP(1)->DIEPCTL |= 1U  << USB_OTG_DIEPCTL_TXFNUM_Pos;		// FIFO Number
				USB_OTG_FS_INEP(1)->DIEPCTL |= 64U << USB_OTG_DIEPCTL_MPSIZ_Pos;		// Max packet size = 64B
				USB_OTG_FS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;					// Clear NAK

				/*
				 * Configure OUT EP1
				 */

				// Clear all OUT EP(1) interrupts flags
				USB_OTG_FS_OUTEP(1)->DOEPINT |=  0x0000313BU;

				// Setup OUT EP1 in BULK mode
				USB_OTG_FS_OUTEP(1)->DOEPCTL = 0x00000000U;
				USB_OTG_FS_OUTEP(1)->DOEPCTL |= 2 <<USB_OTG_DOEPCTL_EPTYP_Pos;			// Bulk
				USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;					// USB Active
				USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;			// Set DATA0 PID
				USB_OTG_FS_OUTEP(1)->DOEPCTL |= 64U <<USB_OTG_DOEPCTL_MPSIZ_Pos;		// Max packet size = 64B
				USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;					// Clear NAK


				/*
				 *  Send a status IN packet on EP0
				 */

				g_usb_in_ctrl.pcnt   = 1;
				g_usb_in_ctrl.bcnt   = 0;

				// Setup IN EP0 for transmission
				USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
				USB_OTG_FS_INEP(0)->DIEPTSIZ |= 1U <<19U;

				USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// 0 bytes
				USB_OTG_FS_INEP(0)->DIEPTSIZ |= 0;

				// Disable device IN EP FIFO empty interrupt
				USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

				// Enable IN EP0
				USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

			    break;
			}

			default:
			{
				my_printf("[UNKNOWN SETUP MESSAGE]");
			}
		}
	}


	/* CLASS SPECIFIC REQUESTS */

	if ((g_usb_setup_packet.bmRequestType == 0xA1) && (g_usb_setup_packet.bRequest == 0x21))
	{
		// 																GET LINE CODING
		// Setup IN control structure for USB CDC Line Coding
		g_usb_in_ctrl.epnum  = 0;
		g_usb_in_ctrl.pcnt   = 1;
		g_usb_in_ctrl.bcnt   = 7;
		g_usb_in_ctrl.pindex = 0;
		g_usb_in_ctrl.buffer = (uint8_t*)usb_cdc_line_coding;

		// Prepare IN EP0 for transmission
		USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// packets number
		USB_OTG_FS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt <<19U;

		USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// bytes number
		USB_OTG_FS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;

		// Enable device IN EP FIFO empty interrupt
		USB_OTG_FS_DEVICE->DIEPEMPMSK |= 1U <<0U;

		// Enable IN EP0
		USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	}


	if ((g_usb_setup_packet.bmRequestType == 0x21) && (g_usb_setup_packet.bRequest == 0x22))
	{

		// 																SET CONTROL LINE STATE
		g_usb_in_ctrl.pcnt   = 1;
		g_usb_in_ctrl.bcnt   = 0;

		// Setup IN EP0 for transmission
		USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
		USB_OTG_FS_INEP(0)->DIEPTSIZ |= 1U <<19U;

		USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// 0 bytes
		USB_OTG_FS_INEP(0)->DIEPTSIZ |= 0;


		// Disable device IN EP FIFO empty interrupt
		USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

		// Enable IN EP0
		USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);


		// Activate IN EP1 in bulk mode with 128-bytes max packet size
		USB_OTG_FS_INEP(1)->DIEPCTL |= 0x02 <<USB_OTG_DIEPCTL_EPTYP_Pos;
		USB_OTG_FS_INEP(1)->DIEPCTL |= 128  <<USB_OTG_DIEPCTL_MPSIZ_Pos;
		USB_OTG_FS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;

		// Disable device IN EP1 FIFO empty interrupt
		USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<1U);

		// Clear all IN EP1 interrupt flags
		USB_OTG_FS_INEP(1)->DIEPINT = 0xFFU;

		// Unmask IN EP1 interrupts
		USB_OTG_FS_DEVICE->DAINTMSK |= 0x00000002;

		// Activate OUT EP1 in bulk mode with 128-bytes max packet size
		USB_OTG_FS_OUTEP(1)->DOEPCTL |= 0x02 <<USB_OTG_DOEPCTL_EPTYP_Pos;
		USB_OTG_FS_OUTEP(1)->DOEPCTL |= 128  <<USB_OTG_DOEPCTL_MPSIZ_Pos;
		USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;
		USB_OTG_FS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;

		// Activate IN EP2 in Interrupt mode with 8-bytes max packet size
		USB_OTG_FS_INEP(2)->DIEPCTL |= 0x03 <<USB_OTG_DIEPCTL_EPTYP_Pos;
		USB_OTG_FS_INEP(2)->DIEPCTL |= 8  <<USB_OTG_DIEPCTL_MPSIZ_Pos;
		USB_OTG_FS_INEP(2)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;

		// Disable device IN EP2 FIFO empty interrupt
		USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<2U);

		// Clear all IN EP1 interrupt flags
		USB_OTG_FS_INEP(2)->DIEPINT = 0xFFU;

		// Unmask IN EP2 interrupts
		USB_OTG_FS_DEVICE->DAINTMSK |= 0x00000004;

		// Check the status of the control line
		if (g_usb_setup_packet.wValue==3)
		{
			if (g_usb.cdc_ready == 0)
			{
				g_usb.cdc_ready = 1;
			}
		}

		else
		{
			g_usb.cdc_ready = 0;
		}
	}


	if ((g_usb_setup_packet.bmRequestType == 0x21) && (g_usb_setup_packet.bRequest == 0x20))
	{
		// 																SET LINE CODING
		g_usb_in_ctrl.pcnt   = 1;
		g_usb_in_ctrl.bcnt   = 0;

		// Setup IN EP0 for transmission
		USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
		USB_OTG_FS_INEP(0)->DIEPTSIZ |= 1U <<19U;

		USB_OTG_FS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// 0 bytes
		USB_OTG_FS_INEP(0)->DIEPTSIZ |= 0;

		// Disable device IN EP FIFO empty interrupt
		USB_OTG_FS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

		// Enable IN EP0
		USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	}
}


/*
 * Read a number bytes from RX FIFO and store into buffer
 */
static void USB_FIFO_Read(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt)
{
	uint16_t	i,j,k;

	// Read FIFO
	if (bcnt > 0)
	{
		// Compute k, the number of 32-bit words to read
		k = (bcnt >>2U);
		if ((bcnt - (k <<2U)) > 0) k++;

		// Read words from DFIFO
		// (The USB core manages DFIFO addressing, so only the access register address is needed)
		j = 0;
		for(i=0; i<k; i++)
		{
			*(uint32_t*)&buffer[j] = *fifo;
			j += 4;
		}
	}
}


/*
 * Write a number of bytes from buffer into TX FIFO
 */
static void USB_FIFO_Write(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt)
{
	uint16_t	i;

	// Write FIFO
	for (i=0; i<bcnt; i+=4)
	{
		*fifo = *(uint32_t*)&buffer[i];
	}
}

