/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "CDCDSerialDriverDescriptors.h"
#include <board.h>
#include <usb/common/core/USBGenericDescriptor.h>
#include <usb/common/core/USBConfigurationDescriptor.h>
#include <usb/common/core/USBEndpointDescriptor.h>
#include <usb/common/core/USBStringDescriptor.h>
#include <usb/common/core/USBGenericRequest.h>
#include <usb/common/cdc/CDCGenericDescriptor.h>
#include <usb/common/cdc/CDCDeviceDescriptor.h>
#include <usb/common/cdc/CDCCommunicationInterfaceDescriptor.h>
#include <usb/common/cdc/CDCDataInterfaceDescriptor.h>
#include <usb/common/cdc/CDCHeaderDescriptor.h>
#include <usb/common/cdc/CDCCallManagementDescriptor.h>
#include <usb/common/cdc/CDCAbstractControlManagementDescriptor.h>
#include <usb/common/cdc/CDCUnionDescriptor.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \page "CDC Serial Device IDs"
/// This page lists the IDs used in the CDC Serial Device Descriptor.
///
/// !IDs
/// - CDCDSerialDriverDescriptors_PRODUCTID
/// - CDCDSerialDriverDescriptors_VENDORID
/// - CDCDSerialDriverDescriptors_RELEASE

/// Device product ID.
#define CDCDSerialDriverDescriptors_PRODUCTID       0x9700
/// Device vendor ID (Atmel).
#define CDCDSerialDriverDescriptors_VENDORID        0x03EB
/// Device release number.
#define CDCDSerialDriverDescriptors_RELEASE         0x0100
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Macros
//------------------------------------------------------------------------------

/// Returns the minimum between two values.
#define MIN(a, b)       ((a < b) ? a : b)

//------------------------------------------------------------------------------
//         Internal structures
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Configuration descriptor list for a device implementing a CDC serial driver.
//------------------------------------------------------------------------------
typedef struct {

    /// Standard configuration descriptor.
    USBConfigurationDescriptor configuration;
#if defined(BOARD_USB_OTGHS)
    // OTG descriptor
    USBOtgDescriptor otgDescriptor;
#endif
    /// Communication interface descriptor.
    //USBInterfaceDescriptor  communication;
    /// CDC header functional descriptor.
    //CDCHeaderDescriptor header;
    /// CDC call management functional descriptor.
    //CDCCallManagementDescriptor callManagement;
    /// CDC abstract control management functional descriptor.
    //CDCAbstractControlManagementDescriptor abstractControlManagement;
    /// CDC union functional descriptor (with one slave interface).
    //CDCUnionDescriptor union1;
    /// Notification endpoint descriptor.
    //USBEndpointDescriptor notification;
    /// Data interface descriptor.
    USBInterfaceDescriptor data;
    /// Data OUT endpoint descriptor.
    USBEndpointDescriptor dataOut;
    /// Data IN endpoint descriptor.
    USBEndpointDescriptor dataIn;

} __attribute__ ((packed)) CDCDSerialDriverConfigurationDescriptors;

//------------------------------------------------------------------------------
//         Exported variables
//------------------------------------------------------------------------------

/// Standard USB device descriptor for the CDC serial driver
const USBDeviceDescriptor deviceDescriptor = {

    sizeof(USBDeviceDescriptor),
    USBGenericDescriptor_DEVICE,
    USBDeviceDescriptor_USB2_00,
    CDCDeviceDescriptor_CLASS,
    CDCDeviceDescriptor_SUBCLASS,
    CDCDeviceDescriptor_PROTOCOL,
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(0),
    CDCDSerialDriverDescriptors_VENDORID,
    CDCDSerialDriverDescriptors_PRODUCTID,
    CDCDSerialDriverDescriptors_RELEASE,
    1, // No string descriptor for manufacturer
    2, // Index of product string descriptor is #1
    3, // No string descriptor for serial number
    1 // Device has 1 possible configuration
};

#if defined(BOARD_USB_UDPHS) || defined(BOARD_USB_OTGHS)

/// USB device qualifier descriptor.
const USBDeviceQualifierDescriptor qualifierDescriptor = {

    sizeof(USBDeviceQualifierDescriptor),
    USBGenericDescriptor_DEVICEQUALIFIER,
    USBDeviceDescriptor_USB2_00,
    CDCDeviceDescriptor_CLASS,
    CDCDeviceDescriptor_SUBCLASS,
    CDCDeviceDescriptor_PROTOCOL,
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(0),
    1, // Device has one possible configuration
    0 // Reserved
};

#endif

/// Standard USB configuration descriptor for the CDC serial driver
const CDCDSerialDriverConfigurationDescriptors configurationDescriptorsFS = {

    // Standard configuration descriptor
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_CONFIGURATION,
        sizeof(CDCDSerialDriverConfigurationDescriptors),
        1,//2, // There are two interfaces in this configuration
        1, // This is configuration #1
        0, // No string descriptor for this configuration
        BOARD_USB_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
#if defined(BOARD_USB_OTGHS)
    // OTG descriptor
    {
        sizeof(USBOtgDescriptor),
        USBGenericDescriptor_OTG,
        USBOTGDescriptor_HNP_SRP
    },
#endif
   
    // Data class interface standard descriptor
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        0, //1, // This is interface #1
        0, // This is alternate setting #0 for this interface
        2, // This interface uses 2 endpoints
        0xFF, //CDCDataInterfaceDescriptor_CLASS,
        0xFF, //CDCDataInterfaceDescriptor_SUBCLASS,
        0xFF, //CDCDataInterfaceDescriptor_NOPROTOCOL,
        0  // No string descriptor for this interface
    },
    // Bulk-OUT endpoint standard descriptor
    {
        sizeof(USBEndpointDescriptor), 
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
                                      CDCDSerialDriverDescriptors_DATAOUT),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAOUT),
            USBEndpointDescriptor_MAXBULKSIZE_FS),
        0 // Must be 0 for full-speed bulk endpoints
    },
    // Bulk-IN endpoint descriptor
    {
        sizeof(USBEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
                                      CDCDSerialDriverDescriptors_DATAIN),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN),
            USBEndpointDescriptor_MAXBULKSIZE_FS),
        0 // Must be 0 for full-speed bulk endpoints
    },
};

/// Language ID string descriptor
const unsigned char languageIdStringDescriptor[] = {

    USBStringDescriptor_LENGTH(1),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_ENGLISH_US
};

#if defined(BOARD_USB_UDPHS) || defined(BOARD_USB_OTGHS)
/// Other-speed configuration descriptor (when in full-speed).
const CDCDSerialDriverConfigurationDescriptors otherSpeedDescriptorsFS = {

    // Standard configuration descriptor
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_OTHERSPEEDCONFIGURATION,
        sizeof(CDCDSerialDriverConfigurationDescriptors),
        1, //2, // There are two interfaces in this configuration
        1, // This is configuration #1
        0, // No string descriptor for this configuration
        BOARD_USB_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
#if defined(BOARD_USB_OTGHS)
    // OTG descriptor
    {
        sizeof(USBOtgDescriptor),
        USBGenericDescriptor_OTG,
        USBOTGDescriptor_HNP_SRP
    },
#endif
    
    // Data class interface standard descriptor
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        0, //1, // This is interface #1
        0, // This is alternate setting #0 for this interface
        2, // This interface uses 2 endpoints
        0xFF, //CDCDataInterfaceDescriptor_CLASS,
        0xFF, //CDCDataInterfaceDescriptor_SUBCLASS,
        0xFF, //CDCDataInterfaceDescriptor_NOPROTOCOL,
        0  // No string descriptor for this interface
    },
    // Bulk-OUT endpoint standard descriptor
    {
        sizeof(USBEndpointDescriptor), 
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
                                      CDCDSerialDriverDescriptors_DATAOUT),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAOUT),
            USBEndpointDescriptor_MAXBULKSIZE_HS),
        0 // Must be 0 for full-speed bulk endpoints
    },
    // Bulk-IN endpoint descriptor
    {
        sizeof(USBEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
                                      CDCDSerialDriverDescriptors_DATAIN),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN),
            USBEndpointDescriptor_MAXBULKSIZE_HS),
        0 // Must be 0 for full-speed bulk endpoints
    },
};


/// Configuration descriptor (when in high-speed).
const CDCDSerialDriverConfigurationDescriptors configurationDescriptorsHS = {

    // Standard configuration descriptor
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_CONFIGURATION,
        sizeof(CDCDSerialDriverConfigurationDescriptors),
        1, //2, // There are two interfaces in this configuration
        1, // This is configuration #1
        0, // No string descriptor for this configuration
        BOARD_USB_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
#if defined(BOARD_USB_OTGHS)
    // OTG descriptor
    {
        sizeof(USBOtgDescriptor),
        USBGenericDescriptor_OTG,
        USBOTGDescriptor_HNP_SRP
    },
#endif
   
    // Data class interface standard descriptor
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        0, //1, // This is interface #1
        0, // This is alternate setting #0 for this interface
        2, // This interface uses 2 endpoints
        0xFF, //CDCDataInterfaceDescriptor_CLASS,
        0xFF, //CDCDataInterfaceDescriptor_SUBCLASS,
        0xFF, //CDCDataInterfaceDescriptor_NOPROTOCOL,
        0  // No string descriptor for this interface
    },
    // Bulk-OUT endpoint standard descriptor
    {
        sizeof(USBEndpointDescriptor), 
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
                                      CDCDSerialDriverDescriptors_DATAOUT),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAOUT),
            USBEndpointDescriptor_MAXBULKSIZE_HS),
        0 // Must be 0 for full-speed bulk endpoints
    },
    // Bulk-IN endpoint descriptor
    {
        sizeof(USBEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
                                      CDCDSerialDriverDescriptors_DATAIN),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN),
            USBEndpointDescriptor_MAXBULKSIZE_HS),
        0 // Must be 0 for full-speed bulk endpoints
    },
};

/// Other-speed configuration descriptor (when in high-speed).
const CDCDSerialDriverConfigurationDescriptors otherSpeedDescriptorsHS = {

    // Standard configuration descriptor
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_OTHERSPEEDCONFIGURATION,
        sizeof(CDCDSerialDriverConfigurationDescriptors),
        1, //2, // There are two interfaces in this configuration
        1, // This is configuration #1
        0, // No string descriptor for this configuration
        BOARD_USB_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
#if defined(BOARD_USB_OTGHS)
    // OTG descriptor
    {
        sizeof(USBOtgDescriptor),
        USBGenericDescriptor_OTG,
        USBOTGDescriptor_HNP_SRP
    },
#endif
    
    // Data class interface standard descriptor
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        0, //1, // This is interface #1
        0, // This is alternate setting #0 for this interface
        2, // This interface uses 2 endpoints
        0xFF, //CDCDataInterfaceDescriptor_CLASS,
        0xFF, //CDCDataInterfaceDescriptor_SUBCLASS,
        0xFF, //CDCDataInterfaceDescriptor_NOPROTOCOL,
        0  // No string descriptor for this interface
    },
    // Bulk-OUT endpoint standard descriptor
    {
        sizeof(USBEndpointDescriptor), 
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
                                      CDCDSerialDriverDescriptors_DATAOUT),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAOUT),
            USBEndpointDescriptor_MAXBULKSIZE_FS),
        0 // Must be 0 for full-speed bulk endpoints
    },
    // Bulk-IN endpoint descriptor
    {
        sizeof(USBEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
                                      CDCDSerialDriverDescriptors_DATAIN),
        USBEndpointDescriptor_BULK,
        MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN),
            USBEndpointDescriptor_MAXBULKSIZE_FS),
        0 // Must be 0 for full-speed bulk endpoints
    },
};
#endif




/// Manufacturer name.
const unsigned char manufacturerDescriptor[] = {

    USBStringDescriptor_LENGTH(15),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('F'),
    USBStringDescriptor_UNICODE('o'),
    USBStringDescriptor_UNICODE('r'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('m'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('d'),
    USBStringDescriptor_UNICODE('i'),
    USBStringDescriptor_UNICODE('a'),
    USBStringDescriptor_UNICODE(' '),  
    USBStringDescriptor_UNICODE('I'),
    USBStringDescriptor_UNICODE('n'),
    USBStringDescriptor_UNICODE('c'),
    USBStringDescriptor_UNICODE('.')
};
/// Product string descriptor
const unsigned char productStringDescriptor[] = {

 USBStringDescriptor_LENGTH(15),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('i'),
    USBStringDescriptor_UNICODE('S'),
    USBStringDescriptor_UNICODE('A'),
    USBStringDescriptor_UNICODE('M'),
    USBStringDescriptor_UNICODE(' '),
    USBStringDescriptor_UNICODE('T'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('s'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE(' '),
    USBStringDescriptor_UNICODE('B'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('n'),
    USBStringDescriptor_UNICODE('c'),
    USBStringDescriptor_UNICODE('h') 
};

// Product serial number.
const unsigned char serialNumberDescriptor[] = {

    USBStringDescriptor_LENGTH(16),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('+'),
    USBStringDescriptor_UNICODE('8'),
    USBStringDescriptor_UNICODE('6'),
    USBStringDescriptor_UNICODE('-'),
    USBStringDescriptor_UNICODE('0'),
    USBStringDescriptor_UNICODE('2'),
    USBStringDescriptor_UNICODE('5'),
    USBStringDescriptor_UNICODE('-'),
    USBStringDescriptor_UNICODE('8'),
    USBStringDescriptor_UNICODE('3'),
    USBStringDescriptor_UNICODE('1'),
    USBStringDescriptor_UNICODE('9'),
    USBStringDescriptor_UNICODE('9'),
    USBStringDescriptor_UNICODE('9'),
    USBStringDescriptor_UNICODE('9'),
    USBStringDescriptor_UNICODE('7')
};

/// List of string descriptors used by the device
const unsigned char *stringDescriptors[] = {

    languageIdStringDescriptor,
    manufacturerDescriptor,
    productStringDescriptor,
    serialNumberDescriptor,
};

/// List of standard descriptors for the serial driver.
USBDDriverDescriptors cdcdSerialDriverDescriptors = {

    &deviceDescriptor,
    (USBConfigurationDescriptor *) &(configurationDescriptorsFS),
#if defined(BOARD_USB_UDPHS) || defined(BOARD_USB_OTGHS)
    &qualifierDescriptor,
    (USBConfigurationDescriptor *) &(otherSpeedDescriptorsFS),
    &deviceDescriptor,
    (USBConfigurationDescriptor *) &(configurationDescriptorsHS),
    &qualifierDescriptor,
    (USBConfigurationDescriptor *) &(otherSpeedDescriptorsHS),
#else
    0, // No full-speed device qualifier descriptor
    0, // No full-speed other speed configuration
    0, // No high-speed device descriptor
    0, // No high-speed configuration descriptor
    0, // No high-speed device qualifier descriptor
    0, // No high-speed other speed configuration descriptor

#endif
    stringDescriptors,
    4 // 2 string descriptors in list
};

