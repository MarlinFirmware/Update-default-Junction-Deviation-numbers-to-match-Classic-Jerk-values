/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

#if defined(LOAD_USB_HOST_SYSTEM) && !defined(USB_HOST_SYSTEM_LOADED)
#define USB_HOST_SYSTEM_LOADED

#ifndef DEBUG_PRINTF_EXTRA_HUGE_UHS_HOST
#define DEBUG_PRINTF_EXTRA_HUGE_UHS_HOST 0
#endif

#if DEBUG_PRINTF_EXTRA_HUGE
#if DEBUG_PRINTF_EXTRA_HUGE_UHS_HOST
#define HOST_DEBUG(...) printf(__VA_ARGS__)
#else
#define HOST_DEBUG(...) VOID0
#endif
#else
#define HOST_DEBUG(...) VOID0
#endif

UHS_EpInfo* UHS_USB_HOST_BASE::getEpInfoEntry(uint8_t addr, uint8_t ep) {
        UHS_Device *p = addrPool.GetUsbDevicePtr(addr);

        if(!p || !p->epinfo)
                return NULL;

        UHS_EpInfo *pep;
        for(uint8_t j = 0; j < UHS_HOST_MAX_INTERFACE_DRIVERS; j++) {
                pep = (UHS_EpInfo *)(p->epinfo[j]);

                for(uint8_t i = 0; i < p->epcount; i++) {
                        if((pep)->epAddr == ep) {
                                HOST_DEBUG("ep entry for interface %d ep %d max packet size = %d\r\n", pep->bIface, ep, pep->maxPktSize);
                                return pep;
                        }

                        pep++;
                }
        }
        return NULL;
}

/**
 * Sets a device table entry for a device.
 * Each device is different and has different number of endpoints.
 * This function plugs endpoint record structure, defined in application, to devtable
 *
 * @param addr device address
 * @param epcount how many endpoints
 * @param eprecord pointer to the endpoint structure
 * @return Zero for success, or error code
 */
uint8_t UHS_USB_HOST_BASE::setEpInfoEntry(uint8_t addr, uint8_t iface, uint8_t epcount, volatile UHS_EpInfo* eprecord) {
        if(!eprecord)
                return UHS_HOST_ERROR_BAD_ARGUMENT;

        UHS_Device *p = addrPool.GetUsbDevicePtr(addr);

        if(!p)
                return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

        p->address.devAddress = addr;
        p->epinfo[iface] = eprecord;
        p->epcount = epcount;
        return 0;
}

/**
 * sets all endpoint addresses to zero.
 * Sets all max packet sizes to defaults
 * Clears all endpoint attributes
 * Sets bmNakPower to USB_NAK_DEFAULT
 * Sets binterface to zero.
 * Sets bNumEP to zero.
 * Sets bAddress to zero.
 * Clears qNextPollTime and sets bPollEnable to false.
 *
 * @param maxep How many endpoints to initialize
 * @param device pointer to the device driver instance (this)
 */

void UHS_USB_HOST_BASE::DeviceDefaults(uint8_t maxep, UHS_USBInterface *interface) {

        for(uint8_t i = 0; i < maxep; i++) {
                interface->epInfo[i].epAddr = 0;
                interface->epInfo[i].maxPktSize = (i) ? 0 : 8;
                interface->epInfo[i].epAttribs = 0;
                interface->epInfo[i].bmNakPower = UHS_USB_NAK_DEFAULT;
        }
        interface->pUsb->GetAddressPool()->FreeAddress(interface->bAddress);
        interface->bIface = 0;
        interface->bNumEP = 1;
        interface->bAddress = 0;
        interface->qNextPollTime = 0;
        interface->bPollEnable = false;
}

/**
 * Perform a bus reset to the port of the connected device
 *
 * @param parent index to Parent
 * @param port what port on the parent
 * @param address address of the device
 * @return Zero for success, or error code
 */

uint8_t UHS_USB_HOST_BASE::doSoftReset(uint8_t parent, uint8_t port, uint8_t address) {
        uint8_t rcode = 0;

        if(parent == 0) {
                // Send a bus reset on the root interface.
                doHostReset();
        } else {
                // reset parent port
                devConfig[parent]->ResetHubPort(port);
        }

        //
        // Many devices require a delay before setting the address here...
        // We loop upon fails for up to 2 seconds instead.
        // Most devices will be happy without a retry.
        //
        uint8_t retries = 0;
        if(address) {
                do {
                        rcode = setAddr(0, address);
                        if(!rcode) break;
                        retries++;
                        sof_delay(10);
                } while(retries < 200);
                HOST_DEBUG("%i retries.\r\n", retries);
        } else {
#if DEBUG_PRINTF_EXTRA_HUGE
                printf("\r\ndoSoftReset called with address == 0.\r\n");
#endif
        }
        return rcode;
}

/**
 * Pseudo code so you may understand the code flow.
 *
 *      reset; (happens at the lower level)
 *      GetDevDescr();
 *      reset;
 *      If there are no configuration descriptors {
 *              //
 *              // NOTE: I know of no device that does this.
 *              // I suppose there could be one though.
 *              //
 *              try to enumerate.
 *      } else {
 *              last success count = 0
 *              best config = 0
 *              for each configuration descriptor {
 *                      for each interface descriptor {
 *                              get the endpoint descriptors for this interface.
 *                              Check to see if a driver can handle this interface.
 *                              If it can, add 1 to the success count.
 *                      }
 *                      if success count > last success count {
 *                              best config = current config
 *                              last success count = success count
 *                      }
 *              }
 *              set the device config to the best config
 *              for each best config interface descriptor {
 *                      initialize driver that can handle this interface config
 *              }
 *      }
 *
 *       NOTES:
 *              1: We do not need to save toggle states anymore and have not
 *                      needed to for some time, because the lower level driver
 *                      actually corrects wrong toggles on-the-fly for us.
 *
 *              2: We always do a second reset, since this stupid bug is
 *                      actually part of the specification documents that I
 *                      have found all over the net. Even Linux does it, and
 *                      many devices actually EXPECT this behavior. Some devices
 *                      will not enumerate without it. For devices that do not
 *                      need it, the additional reset is harmless. Here is an
 *                      example of one of these documents, see page Five:
 *                      https://www.ftdichip.com/Support/Documents/TechnicalNotes/TN_113_Simplified%20Description%20of%20USB%20Device%20Enumeration.pdf
 *
 */

/**
 * Enumerates interfaces on devices
 *
 * @param parent index to Parent
 * @param port what port on the parent
 * @param speed the speed of the device
 * @return Zero for success, or error code
 */
uint8_t UHS_USB_HOST_BASE::Configuring(uint8_t parent, uint8_t port, uint8_t speed) {
        //uint8_t bAddress = 0;
        HOST_DEBUG("\r\n\r\n\r\nConfiguring: parent = %i, port = %i, speed = %i\r\n", parent, port, speed);
        uint8_t rcode = 0;
        uint8_t retries = 0;
        uint8_t numinf = 0;
        uint8_t configs;
        UHS_Device *p = NULL;
        //EpInfo epInfo; // cap at 16, this should be fairly reasonable.
        ENUMERATION_INFO ei;
        uint8_t bestconf = 0;
        uint8_t bestsuccess = 0;

        uint8_t devConfigIndex;

#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
        const uint8_t biggest = 0x40;
        // wrap in {} to throw away the 64 byte buffer when we are done with it
        {
                uint8_t buf[biggest];
                USB_FD_DEVICE_DESCRIPTOR *udd = reinterpret_cast<USB_FD_DEVICE_DESCRIPTOR *>(buf);
#else
        const uint8_t biggest = 18;
        uint8_t buf[biggest];
        USB_FD_DEVICE_DESCRIPTOR *udd = reinterpret_cast<USB_FD_DEVICE_DESCRIPTOR *>(buf);
        USB_FD_CONFIGURATION_DESCRIPTOR *ucd = reinterpret_cast<USB_FD_CONFIGURATION_DESCRIPTOR *>(buf);
#endif

                //for(devConfigIndex = 0; devConfigIndex < UHS_HOST_MAX_INTERFACE_DRIVERS; devConfigIndex++) {
                //        if((devConfig[devConfigIndex]->bAddress) && (!devConfig[devConfigIndex]->bPollEnable)) {
                //                devConfig[devConfigIndex]->bAddress = 0;
                //        }
                //}
                //        Serial.print("HOST USB Host @ 0x");
                //        Serial.println((uint32_t)this, HEX);
                //        Serial.print("HOST USB Host Address Pool @ 0x");
                //        Serial.println((uint32_t)GetAddressPool(), HEX);

                sof_delay(200);
                p = addrPool.GetUsbDevicePtr(0);
                if(!p) {
                        HOST_DEBUG("Configuring error: USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL\r\n");
                        return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;
                }

                p->speed = speed;
#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE

                p->epinfo[0][0].maxPktSize = 0x40; // Windows bug is expected.
                // poison data
                // udd->bMaxPacketSize0 = 0U;
#else
                p->epinfo[0][0].maxPktSize = 0x08; // USB Spec, start small, work your way up.
#endif
again:
                memset((void *)buf, 0, biggest);
                HOST_DEBUG("\r\n\r\nConfiguring PktSize 0x%2.2x,  rcode: 0x%2.2x, retries %i,\r\n", p->epinfo[0][0].maxPktSize, rcode, retries);
                rcode = getDevDescr(0, biggest, (uint8_t*)buf);
#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
                if(rcode || udd->bMaxPacketSize0 < 8)
#else
                if(rcode)
#endif
                {
                        if(rcode == UHS_HOST_ERROR_JERR && retries < 4) {
                                //
                                // Some devices return JERR when plugged in.
                                // Attempts to reinitialize the device usually works.
                                //
                                // I have a hub that will refuse to work and acts like
                                // this unless external power is supplied.
                                // So this may not always work, and you may be fooled.
                                //
                                sof_delay(100);
                                retries++;
                                goto again;
#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
                        } else if(((rcode == UHS_HOST_ERROR_DMA || rcode == UHS_HOST_ERROR_MEM_LAT) && retries < 4) || (udd->bMaxPacketSize0 < 8 && !rcode)) {

                                if(p->epinfo[0][0].maxPktSize > 8 && rcode == UHS_HOST_ERROR_DMA) p->epinfo[0][0].maxPktSize = p->epinfo[0][0].maxPktSize >> 1;
#else
                        } else if((rcode == UHS_HOST_ERROR_DMA || rcode == UHS_HOST_ERROR_MEM_LAT) && retries < 4) {
                                if(p->epinfo[0][0].maxPktSize < 32) p->epinfo[0][0].maxPktSize = p->epinfo[0][0].maxPktSize << 1;
#endif
                                HOST_DEBUG("Configuring error: 0x%2.2x UHS_HOST_ERROR_DMA. Retry with maxPktSize: %i\r\n", rcode, p->epinfo[0][0].maxPktSize);
                                doSoftReset(parent, port, 0);
                                retries++;
                                sof_delay(200);
                                goto again;
                        }
                        HOST_DEBUG("Configuring error: 0x%2.2x Can't get USB_FD_DEVICE_DESCRIPTOR\r\n", rcode);
                        return rcode;
                }

#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
                ei.address = addrPool.AllocAddress(parent, false, port);

                if(!ei.address) {
                        return UHS_HOST_ERROR_ADDRESS_POOL_FULL;
                }

                p = addrPool.GetUsbDevicePtr(ei.address);
                // set to 1 if you suspect address table corruption.
#if 0
                if(!p) {
                        return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;
                }
#endif

                p->speed = speed;

                rcode = doSoftReset(parent, port, ei.address);

                if(rcode) {
                        addrPool.FreeAddress(ei.address);
                        HOST_DEBUG("Configuring error: %2.2x Can't set USB INTERFACE ADDRESS\r\n", rcode);
                        return rcode;
                }

                { // the { } wrapper saves on stack.
                        HOST_DEBUG("DevDescr 2nd poll, bMaxPacketSize0:%u\r\n", udd->bMaxPacketSize0);
                        UHS_EpInfo dev1ep;
                        dev1ep.maxPktSize = udd->bMaxPacketSize0;
                        dev1ep.epAddr = 0;
                        dev1ep.epAttribs = 0;
                        dev1ep.bmNakPower = UHS_USB_NAK_MAX_POWER;
                        p->address.devAddress = ei.address;
                        p->epcount = 1;
                        p->epinfo[0] = &dev1ep;

                        sof_delay(10);
                        memset((void *)buf, 0, biggest);
                        rcode = getDevDescr(ei.address, 18, (uint8_t*)buf);
                        if(rcode) HOST_DEBUG("getDevDescr err: 0x%x \r\n", rcode);

                        addrPool.FreeAddress(ei.address);
                        if(rcode && rcode != UHS_HOST_ERROR_DMA) {
                                return rcode;
                        }
                        sof_delay(10);
                }
#endif

                ei.vid = udd->idVendor;
                ei.pid = udd->idProduct;
                ei.bcdDevice = udd->bcdDevice;
                ei.klass = udd->bDeviceClass;
                ei.subklass = udd->bDeviceSubClass;
                ei.protocol = udd->bDeviceProtocol;
                ei.bMaxPacketSize0 = udd->bMaxPacketSize0;
                ei.currentconfig = 0;
                ei.parent = parent;
                ei.port = port;
                configs = udd->bNumConfigurations;
#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
        } // unwrapped, old large buf now invalid and discarded.

        uint8_t buf[18];
        USB_FD_CONFIGURATION_DESCRIPTOR *ucd = reinterpret_cast<USB_FD_CONFIGURATION_DESCRIPTOR *>(buf);
#endif

        ei.address = addrPool.AllocAddress(parent, IsHub(ei.klass), port);

        if(!ei.address) {
                return UHS_HOST_ERROR_ADDRESS_POOL_FULL;
        }

        p = addrPool.GetUsbDevicePtr(ei.address);
        // set to 1 if you suspect address table corruption.
#if 0
        if(!p) {
                return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;
        }
#endif

        p->speed = speed;

        rcode = doSoftReset(parent, port, ei.address);

        if(rcode) {
                addrPool.FreeAddress(ei.address);
                HOST_DEBUG("Configuring error: %2.2x Can't set USB INTERFACE ADDRESS\r\n", rcode);
                return rcode;
        }

        if(configs < 1) {
                HOST_DEBUG("No interfaces?!\r\n");
                addrPool.FreeAddress(ei.address);
                // rcode = TestInterface(&ei);
                // Not implemented (yet)
                rcode = UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED;
        } else {
                HOST_DEBUG("configs: %i\r\n", configs);
                for(uint8_t conf = 0; (!rcode) && (conf < configs); conf++) {
                        // read the config descriptor into a buffer.
                        rcode = getConfDescr(ei.address, sizeof (USB_FD_CONFIGURATION_DESCRIPTOR), conf, buf);
                        if(rcode) {
                                HOST_DEBUG("Configuring error: %2.2x Can't get USB_FD_INTERFACE_DESCRIPTOR\r\n", rcode);
                                rcode = UHS_HOST_ERROR_FailGetConfDescr;
                                continue;
                        }
                        ei.currentconfig = conf;
                        numinf = ucd->bNumInterfaces; // Does _not_ include alternates!
                        HOST_DEBUG("CONFIGURATION: %i, bNumInterfaces %i, wTotalLength %i\r\n", conf, numinf, ucd->wTotalLength);
                        uint8_t success = 0;
                        uint16_t inf = 0;
                        uint8_t data[ei.bMaxPacketSize0];
                        UHS_EpInfo *pep;
                        pep = ctrlReqOpen(ei.address, mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, ei.currentconfig, USB_DESCRIPTOR_CONFIGURATION, 0x0000U, ucd->wTotalLength), data);
                        if(!pep) {
                                rcode = UHS_HOST_ERROR_NULL_EPINFO;
                                continue;
                        }
                        uint16_t left;
                        uint16_t read;
                        uint8_t offset;
                        rcode = initDescrStream(&ei, ucd, pep, data, &left, &read, &offset);
                        if(rcode) {
                                HOST_DEBUG("Configuring error: %2.2x Can't get USB_FD_INTERFACE_DESCRIPTOR stream.\r\n", rcode);
                                break;
                        }
                        for(; (numinf) && (!rcode); inf++) {
                                // iterate for each interface on this config
                                rcode = getNextInterface(&ei, pep, data, &left, &read, &offset);
                                if(rcode == UHS_HOST_ERROR_END_OF_STREAM) {
                                        HOST_DEBUG("USB_INTERFACE END OF STREAM\r\n");
                                        ctrlReqClose(pep, UHS_bmREQ_GET_DESCR, left, ei.bMaxPacketSize0, data);
                                        rcode = 0;
                                        break;
                                }
                                if(rcode) {
                                        HOST_DEBUG("Configuring error: %2.2x Can't close USB_FD_INTERFACE_DESCRIPTOR stream.\r\n", rcode);
                                        continue;
                                }
                                rcode = TestInterface(&ei);
                                if(!rcode) success++;
                                rcode = 0;
                        }
                        if(!inf) {
                                rcode = TestInterface(&ei);
                                if(!rcode) success++;
                                rcode = 0;
                        }
                        if(success > bestsuccess) {
                                bestconf = conf;
                                bestsuccess = success;
                        }
                }
                if(!bestsuccess) rcode = UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED;
        }
        if(!rcode) {
                rcode = getConfDescr(ei.address, sizeof (USB_FD_CONFIGURATION_DESCRIPTOR), bestconf, buf);
                if(rcode) {
                        HOST_DEBUG("Configuring error: %2.2x Can't get USB_FD_INTERFACE_DESCRIPTOR\r\n", rcode);
                        rcode = UHS_HOST_ERROR_FailGetConfDescr;
                }
        }
        if(!rcode) {
                bestconf++;
                ei.currentconfig = bestconf;
                numinf = ucd->bNumInterfaces; // Does _not_ include alternates!
                HOST_DEBUG("CONFIGURATION: %i, bNumInterfaces %i, wTotalLength %i\r\n", bestconf, numinf, ucd->wTotalLength);
                if(!rcode) {
                        HOST_DEBUG("Best configuration is %i, enumerating interfaces.\r\n", bestconf);
                        uint16_t inf = 0;
                        uint8_t data[ei.bMaxPacketSize0];
                        UHS_EpInfo *pep;
                        pep = ctrlReqOpen(ei.address, mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, ei.currentconfig - 1, USB_DESCRIPTOR_CONFIGURATION, 0x0000U, ucd->wTotalLength), data);
                        if(!pep) {
                                rcode = UHS_HOST_ERROR_NULL_EPINFO;

                        } else {
                                uint16_t left;
                                uint16_t read;
                                uint8_t offset;
                                rcode = initDescrStream(&ei, ucd, pep, data, &left, &read, &offset);
                                if(rcode) {
                                        HOST_DEBUG("Configuring error: %2.2x Can't get USB_FD_INTERFACE_DESCRIPTOR stream.\r\n", rcode);
                                } else {
                                        for(; (numinf) && (!rcode); inf++) {
                                                // iterate for each interface on this config
                                                rcode = getNextInterface(&ei, pep, data, &left, &read, &offset);
                                                if(rcode == UHS_HOST_ERROR_END_OF_STREAM) {
                                                        ctrlReqClose(pep, UHS_bmREQ_GET_DESCR, left, ei.bMaxPacketSize0, data);
                                                        rcode = 0;
                                                        break;
                                                }
                                                if(rcode) {
                                                        HOST_DEBUG("Configuring error: %2.2x Can't close USB_FD_INTERFACE_DESCRIPTOR stream.\r\n", rcode);
                                                        continue;
                                                }

                                                if(enumerateInterface(&ei) == UHS_HOST_MAX_INTERFACE_DRIVERS) {
                                                        HOST_DEBUG("No interface driver for this interface.");
                                                } else {
                                                        HOST_DEBUG("Interface Configured\r\n");
                                                }
                                        }
                                }
                        }
                } else {
                        HOST_DEBUG("Configuring error: %2.2x Can't set USB_INTERFACE_CONFIG stream.\r\n", rcode);
                }
        }

        if(!rcode) {
                rcode = setConf(ei.address, bestconf);
                if(rcode) {
                        HOST_DEBUG("Configuring error: %2.2x Can't set Configuration.\r\n", rcode);
                        addrPool.FreeAddress(ei.address);
                } else {
                        for(devConfigIndex = 0; devConfigIndex < UHS_HOST_MAX_INTERFACE_DRIVERS; devConfigIndex++) {
                                HOST_DEBUG("Driver %i ", devConfigIndex);
                                if(!devConfig[devConfigIndex]) {
                                        HOST_DEBUG("no driver at this index.\r\n");
                                        continue; // no driver
                                }
                                HOST_DEBUG("@ %2.2x ", devConfig[devConfigIndex]->bAddress);
                                if(devConfig[devConfigIndex]->bAddress) {
                                        if(!devConfig[devConfigIndex]->bPollEnable) {
                                                HOST_DEBUG("Initialize\r\n");
                                                rcode = devConfig[devConfigIndex]->Finalize();
                                                rcode = devConfig[devConfigIndex]->Start();
                                                if(!rcode) {
                                                        HOST_DEBUG("Total endpoints = (%i)%i\r\n", p->epcount, devConfig[devConfigIndex]->bNumEP);
                                                } else {
                                                        break;
                                                }
                                        } else {
                                                HOST_DEBUG("Already initialized.\r\n");
                                                continue; // consumed
                                        }
                                } else {
                                        HOST_DEBUG("Skipped\r\n");
                                }
                        }
#if 0 // defined(UHS_HID_LOADED)
                        // Now do HID
#endif
                }
        } else {
                addrPool.FreeAddress(ei.address);
        }
        return rcode;
}

/**
 * Removes a device from the tables
 *
 * @param addr address of the device
 * @return nothing
 */
void UHS_USB_HOST_BASE::ReleaseDevice(uint8_t addr) {
        if(addr) {
#if 0 // defined(UHS_HID_LOADED)
                // Release any HID children
                UHS_HID_Release(this, addr);
#endif
                for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
                        if(!devConfig[i]) continue;
                        if(devConfig[i]->bAddress == addr) {
                                devConfig[i]->Release();
                                break;
                        }
                }
        }
}

/**
 * Gets the device descriptor, or part of it from endpoint Zero.
 *
 * @param addr Address of the device
 * @param nbytes how many bytes to return
 * @param dataptr pointer to the data to return
 * @return status of the request, zero is success.
 */
uint8_t UHS_USB_HOST_BASE::getDevDescr(uint8_t addr, uint16_t nbytes, uint8_t *dataptr) {
        return ( ctrlReq(addr, mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, nbytes), nbytes, dataptr));
}

/**
 * Gets the config descriptor, or part of it from endpoint Zero.
 *
 * @param addr Address of the device
 * @param nbytes how many bytes to return
 * @param conf index to descriptor to return
 * @param dataptr ointer to the data to return
 * @return status of the request, zero is success.
 */
uint8_t UHS_USB_HOST_BASE::getConfDescr(uint8_t addr, uint16_t nbytes, uint8_t conf, uint8_t *dataptr) {
        return ( ctrlReq(addr, mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes), nbytes, dataptr));
}

/**
 * Get the string descriptor from a device
 *
 * @param addr Address of the device
 * @param ns
 * @param index
 * @param langid language ID
 * @param dataptr pointer to the data to return
 * @return status of the request, zero is success.
 */
uint8_t UHS_USB_HOST_BASE::getStrDescr(uint8_t addr, uint16_t ns, uint8_t index, uint16_t langid, uint8_t *dataptr) {
        return ( ctrlReq(addr, mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, langid, ns), ns, dataptr));
}

//
//set address
//

/**
 * Set the address of a device to a new address via endpoint Zero.
 *
 * @param oldaddr current address
 * @param newaddr new address
 * @return status of the request, zero is success.
 */
uint8_t UHS_USB_HOST_BASE::setAddr(uint8_t oldaddr, uint8_t newaddr) {
        uint8_t rcode = ctrlReq(oldaddr, mkSETUP_PKT8(UHS_bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000), 0x0000, NULL);
        sof_delay(300); // Older spec says you should wait at least 200ms
        return rcode;
}

//
//set configuration
//

/**
 * Set the configuration for the device to use via endpoint Zero.
 *
 * @param addr Address of the device
 * @param conf_value configuration index value
 * @return status of the request, zero is success.
 */
uint8_t UHS_USB_HOST_BASE::setConf(uint8_t addr, uint8_t conf_value) {
        return ( ctrlReq(addr, mkSETUP_PKT8(UHS_bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000), 0x0000, NULL));
}

/* rcode 0 if no errors. rcode 01-0f is relayed from HRSL                       */

/**
 * Writes data to an interface pipe
 *
 * @param addr Address of the device
 * @param ep Endpoint of the pipe
 * @param nbytes number of bytes to transfer
 * @param data pointer to buffer to hold transfer
 * @return zero for success or error code
 */
uint8_t UHS_USB_HOST_BASE::outTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *data) {
        UHS_EpInfo *pep = NULL;
        uint16_t nak_limit = 0;
        HOST_DEBUG("outTransfer: addr: 0x%2.2x ep: 0x%2.2x nbytes: 0x%4.4x data: 0x%p\r\n", addr, ep, nbytes, data);

        uint8_t rcode = SetAddress(addr, ep, &pep, nak_limit);
        HOST_DEBUG("outTransfer: SetAddress 0x%2.2x\r\n", rcode);
        if(!rcode)
                rcode = OutTransfer(pep, nak_limit, nbytes, data);
        return rcode;
};

/**
 * Reads data from an interface pipe
 *
 * @param addr Address of the device
 * @param ep Endpoint of the pipe
 * @param nbytesptr number of bytes to transfer
 * @param data pointer to buffer to hold transfer
 * @return zero for success or error code
 */
uint8_t UHS_USB_HOST_BASE::inTransfer(uint8_t addr, uint8_t ep, uint16_t *nbytesptr, uint8_t *data) {
        UHS_EpInfo *pep = NULL;
        uint16_t nak_limit = 0;

        uint8_t rcode = SetAddress(addr, ep, &pep, nak_limit);

        //        if(rcode) {
        //                USBTRACE3("(USB::InTransfer) SetAddress Failed ", rcode, 0x81);
        //                USBTRACE3("(USB::InTransfer) addr requested ", addr, 0x81);
        //                USBTRACE3("(USB::InTransfer) ep requested ", ep, 0x81);
        //                return rcode;
        //        }
        if(!rcode)
                rcode = InTransfer(pep, nak_limit, nbytesptr, data);
        return rcode;

}

/**
 * Initialize the descriptor stream, works much like opening a file.
 *
 * @param ei
 * @param ucd
 * @param pep
 * @param data
 * @param left
 * @param read
 * @param offset
 * @return zero for success or error code
 */
uint8_t UHS_USB_HOST_BASE::initDescrStream(ENUMERATION_INFO *ei, USB_FD_CONFIGURATION_DESCRIPTOR *ucd, UHS_EpInfo *pep, uint8_t *data, uint16_t *left, uint16_t *read, uint8_t *offset) {
        if(!ei || !ucd) return UHS_HOST_ERROR_BAD_ARGUMENT;
        if(!pep) return UHS_HOST_ERROR_NULL_EPINFO;
        *left = ucd->wTotalLength;
        *read = 0;
        *offset = 1;
        uint8_t rcode;
        pep->maxPktSize = ei->bMaxPacketSize0;
        rcode = getone(pep, left, read, data, offset);
        return rcode;
}

uint8_t UHS_USB_HOST_BASE::getNextInterface(ENUMERATION_INFO *ei, UHS_EpInfo *pep, uint8_t data[], uint16_t *left, uint16_t *read, uint8_t *offset) {
        uint16_t remain;
        uint8_t ty;
        uint8_t rcode = UHS_HOST_ERROR_END_OF_STREAM;
        uint8_t *ptr;
        uint8_t epc = 0;
        ei->interface.numep = 0;
        ei->interface.klass = 0;
        ei->interface.subklass = 0;
        ei->interface.protocol = 0;
        while(*left + *read) {
                remain = data[*offset]; // bLength
                while(remain < 2) {
                        rcode = getone(pep, left, read, data, offset);
                        if(rcode)
                                return rcode;
                        remain = data[*offset];
                }
                rcode = getone(pep, left, read, data, offset);
                if(rcode)
                        return rcode;
                ty = data[*offset]; // bDescriptorType
                HOST_DEBUG("bLength: %i ", remain);
                HOST_DEBUG("bDescriptorType: %2.2x\r\n", ty);
                remain--;
                if(ty == USB_DESCRIPTOR_INTERFACE) {
                        HOST_DEBUG("INTERFACE DESCRIPTOR FOUND\r\n");
                        ptr = (uint8_t *)(&(ei->interface.bInterfaceNumber));
                        for(int i = 0; i < 6; i++) {
                                rcode = getone(pep, left, read, data, offset);
                                if(rcode)
                                        return rcode;
                                *ptr = data[*offset];
                                ptr++;
                        }
                        rcode = getone(pep, left, read, data, offset);
                        if(rcode)
                                return rcode;
                        // Now at iInterface
                        // Get endpoints.
                        HOST_DEBUG("Getting %i endpoints\r\n", ei->interface.numep);
                        while(epc < ei->interface.numep) {
                                rcode = getone(pep, left, read, data, offset);
                                if(rcode) {
                                        HOST_DEBUG("ENDPOINT DESCRIPTOR DIED WAY EARLY\r\n");
                                        return rcode;
                                }
                                remain = data[*offset]; // bLength
                                while(remain < 2) {
                                        rcode = getone(pep, left, read, data, offset);
                                        if(rcode)
                                                return rcode;
                                        remain = data[*offset];
                                }
                                rcode = getone(pep, left, read, data, offset);
                                if(rcode) {
                                        HOST_DEBUG("ENDPOINT DESCRIPTOR DIED EARLY\r\n");
                                        return rcode;
                                }
                                ty = data[*offset]; // bDescriptorType
                                HOST_DEBUG("bLength: %i ", remain);
                                HOST_DEBUG("bDescriptorType: %2.2x\r\n", ty);
                                remain -= 2;
                                if(ty == USB_DESCRIPTOR_ENDPOINT) {
                                        HOST_DEBUG("ENDPOINT DESCRIPTOR: %i\r\n", epc);
                                        ptr = (uint8_t *)(&(ei->interface.epInfo[epc].bEndpointAddress));
                                        for(unsigned int i = 0; i< sizeof (ENDPOINT_INFO); i++) {
                                                rcode = getone(pep, left, read, data, offset);
                                                if(rcode) {
                                                        HOST_DEBUG("ENDPOINT DESCRIPTOR DIED LATE\r\n");
                                                        return rcode;
                                                }
                                                *ptr = data[*offset];
                                                ptr++;
                                                remain--;
                                        }
                                        epc++;
                                        HOST_DEBUG("ENDPOINT DESCRIPTOR OK\r\n");
                                }
                                rcode = eat(pep, left, read, data, offset, &remain);
                                if(rcode) {
                                        HOST_DEBUG("ENDPOINT DESCRIPTOR DIED EATING\r\n");
                                        return rcode;
                                }
                                remain = 0;
                        }
                        remain = 1;
                        // queue ahead, but do not report if error.
                        rcode = eat(pep, left, read, data, offset, &remain);
                        if(!ei->interface.numep && rcode) {
                                return rcode;
                        }
                        HOST_DEBUG("ENDPOINT DESCRIPTORS FILLED\r\n");
                        return 0;
                } else {
                        rcode = eat(pep, left, read, data, offset, &remain);
                        if(rcode)
                                return rcode;
                }
                rcode = UHS_HOST_ERROR_END_OF_STREAM;
        }
        return rcode;
}

uint8_t UHS_USB_HOST_BASE::seekInterface(ENUMERATION_INFO *ei, uint16_t inf, USB_FD_CONFIGURATION_DESCRIPTOR *ucd) {
        if(!ei || !ucd) return UHS_HOST_ERROR_BAD_ARGUMENT;
        uint8_t data[ei->bMaxPacketSize0];
        UHS_EpInfo *pep;
        pep = ctrlReqOpen(ei->address, mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, ei->currentconfig,
                USB_DESCRIPTOR_CONFIGURATION, 0x0000U, ucd->wTotalLength), data);
        if(!pep) return UHS_HOST_ERROR_NULL_EPINFO;
        uint16_t left = ucd->wTotalLength;
        uint8_t cinf = 0;
        uint8_t ty;
        uint8_t epc = 0;
        uint16_t remain = ucd->bLength;
        uint16_t read = 0;
        uint8_t offset = remain;
        uint8_t *ptr;
        uint8_t rcode;
        ei->interface.numep = 0;
        ei->interface.klass = 0;
        ei->interface.subklass = 0;
        ei->interface.protocol = 0;
        pep->maxPktSize = ei->bMaxPacketSize0;

        rcode = getone(pep, &left, &read, data, &offset);
        if(rcode)
                return rcode;
        HOST_DEBUG("\r\nGetting interface: %i\r\n", inf);
        inf++;
        while(cinf != inf && (left + read)) {
                //HOST_DEBUG("getInterface: cinf: %i inf: %i left: %i read: %i offset: %i remain %i\r\n", cinf, inf, left, read, offset, remain);
                // Go past current descriptor
                HOST_DEBUG("Skip: %i\r\n", remain);
                rcode = eat(pep, &left, &read, data, &offset, &remain);
                if(rcode)
                        return rcode;
                remain = data[offset]; // bLength
                while(remain < 2) {
                        rcode = getone(pep, &left, &read, data, &offset);
                        if(rcode)
                                return rcode;
                        remain = data[offset];
                }
                rcode = getone(pep, &left, &read, data, &offset);
                if(rcode)
                        return rcode;
                ty = data[offset]; // bDescriptorType
                HOST_DEBUG("bLength: %i ", remain);
                HOST_DEBUG("bDescriptorType: %2.2x\r\n", ty);
                remain--;
                if(ty == USB_DESCRIPTOR_INTERFACE) {
                        HOST_DEBUG("INTERFACE DESCRIPTOR: %i\r\n", cinf);
                        cinf++;
                        if(cinf == inf) {
                                // Get the interface descriptor information.
                                ptr = (uint8_t *)(&(ei->interface.bInterfaceNumber));
                                for(int i = 0; i < 6; i++) {
                                        rcode = getone(pep, &left, &read, data, &offset);
                                        if(rcode)
                                                return rcode;
                                        *ptr = data[offset];
                                        ptr++;
                                }
                                rcode = getone(pep, &left, &read, data, &offset);
                                if(rcode)
                                        return rcode;
                                // Now at iInterface
                                remain = 0;
                                // Get endpoints.
                                HOST_DEBUG("Getting %i endpoints\r\n", ei->interface.numep);
                                while(epc < ei->interface.numep) {
                                        rcode = getone(pep, &left, &read, data, &offset);
                                        if(rcode)
                                                return rcode;
                                        remain = data[offset]; // bLength
                                        while(remain < 2) {
                                                rcode = getone(pep, &left, &read, data, &offset);
                                                if(rcode)
                                                        return rcode;
                                                remain = data[offset];
                                        }
                                        rcode = getone(pep, &left, &read, data, &offset);
                                        if(rcode)
                                                return rcode;
                                        ty = data[offset]; // bDescriptorType
                                        HOST_DEBUG("bLength: %i ", remain);
                                        HOST_DEBUG("bDescriptorType: %2.2x\r\n", ty);
                                        remain--;
                                        if(ty == USB_DESCRIPTOR_ENDPOINT) {
                                                HOST_DEBUG("ENDPOINT DESCRIPTOR: %i\r\n", epc);
                                                ptr = (uint8_t *)(&(ei->interface.epInfo[epc].bEndpointAddress));
                                                for(unsigned int i = 0; i< sizeof (ENDPOINT_INFO); i++) {
                                                        rcode = getone(pep, &left, &read, data, &offset);
                                                        if(rcode)
                                                                return rcode;
                                                        *ptr = data[offset];
                                                        ptr++;
                                                }
                                                epc++;
                                                remain = 0;
                                        } else {
                                                rcode = eat(pep, &left, &read, data, &offset, &remain);
                                                if(rcode)
                                                        return rcode;
                                                remain = 0;
                                        }
                                }
                        }
                }
        }

        return ctrlReqClose(pep, UHS_bmREQ_GET_DESCR, left, ei->bMaxPacketSize0, data);
}

uint8_t UHS_USB_HOST_BASE::getone(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset) {
        uint8_t rcode = 0;
        *offset += 1;
        if(*offset < *read) {
                return 0;
        } else if(*left > 0) {
                // uint16_t num = *left;
                uint16_t num = pep->maxPktSize;
                if(num > *left) num = *left;
                *offset = 0;
                rcode = ctrlReqRead(pep, left, read, num, dataptr);
                if(rcode == 0) {
                        if(*read == 0) {
                                rcode = UHS_HOST_ERROR_END_OF_STREAM;
                        } else if(*read < num) *left = 0;
                }
        } else {
                rcode = UHS_HOST_ERROR_END_OF_STREAM;
        }
        return rcode;
}

uint8_t UHS_USB_HOST_BASE::eat(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset, uint16_t *yum) {
        uint8_t rcode = 0;
        HOST_DEBUG("eating %i\r\n", *yum);
        while(*yum) {
                *yum -= 1;
                rcode = getone(pep, left, read, dataptr, offset);
                if(rcode) break;
        }
        return rcode;
}

uint8_t UHS_USB_HOST_BASE::ctrlReq(uint8_t addr, uint64_t Request, uint16_t nbytes, uint8_t *dataptr) {
        //bool direction = bmReqType & 0x80; //request direction, IN or OUT
        uint8_t rcode = 0;

        //Serial.println();
        UHS_EpInfo *pep = ctrlReqOpen(addr, Request, dataptr);
        if(!pep) {
                HOST_DEBUG("ctrlReq1: ERROR_NULL_EPINFO addr: %d\r\n", addr);
                return UHS_HOST_ERROR_NULL_EPINFO;
        }
        uint8_t rt = (uint8_t)(Request & 0xFFU);

        //        Serial.println("Opened");
        uint16_t left = (uint16_t)(Request >> 48) /*total*/;
        if(dataptr != NULL) {
                //data stage
                if((rt & 0x80) == 0x80) {
                        //IN transfer
                        while(left) {
                                // Bytes read into buffer
                                uint16_t read = nbytes;
                                HOST_DEBUG("ctrlReq2: left: %i, read:%i, nbytes %i\r\n", left, read, nbytes);
                                rcode = ctrlReqRead(pep, &left, &read, nbytes, dataptr);

#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
                                HOST_DEBUG("RESULT: 0x%2.2x 0x%2.2x 0x%2.2x 0x%8.8lx%8.8lx\r\n", rcode, addr, read, (uint32_t)((Request>>32)&0xFFFFFFFFLU), (uint32_t)(Request&0xFFFFFFFFLU));
                                // Should only be used for GET_DESCRIPTOR USB_DESCRIPTOR_DEVICE
                                constexpr uint32_t req_match = ((uint32_t)USB_DESCRIPTOR_DEVICE      << 24) |
                                                               ((uint32_t)USB_REQUEST_GET_DESCRIPTOR <<  8);
                                const     uint32_t req_found = Request & 0xFF00FF00ul;
                                if(!addr && read && (req_found == req_match)) {
                                        HOST_DEBUG("ctrlReq3: acceptBuffer sz %i nbytes %i left %i\n\r", read, nbytes, left);
                                        left = 0;
                                        rcode = UHS_HOST_ERROR_NONE;
                                        break;
                                }
#endif
                                if(rcode) {
                                        return rcode;
                                }
                        }
                } else {
                        // OUT transfer
                        rcode = OutTransfer(pep, 0, nbytes, dataptr);
                }
                if(rcode) {
                        //return error
                        return ( rcode);
                }
        }

        //        Serial.println("Close Phase");
        //        Serial.flush();
        // Status stage
        rcode = ctrlReqClose(pep, rt, left, nbytes, dataptr);
        //        Serial.println("Closed");
        return rcode;
}

uint8_t UHS_USB_HOST_BASE::EPClearHalt(uint8_t addr, uint8_t ep) {
        return ctrlReq(addr, mkSETUP_PKT8(USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_ENDPOINT, USB_REQUEST_CLEAR_FEATURE, USB_FEATURE_ENDPOINT_HALT, 0, ep, 0), 0, NULL);
}

uint8_t UHS_USB_HOST_BASE::TestInterface(ENUMERATION_INFO *ei) {

        uint8_t devConfigIndex;
        uint8_t rcode = 0;
        HOST_DEBUG("TestInterface VID:%4.4x PID:%4.4x Class:%2.2x Subclass:%2.2x Protocol %2.2x\r\n", ei->vid, ei->pid, ei->klass, ei->subklass, ei->protocol);
        HOST_DEBUG("Interface data: Class:%2.2x Subclass:%2.2x Protocol %2.2x, number of endpoints %i\r\n", ei->interface.klass, ei->interface.subklass, ei->interface.subklass, ei->interface.numep);
        HOST_DEBUG("Parent: %2.2x, bAddress: %2.2x\r\n", ei->parent, ei->address);
        for(devConfigIndex = 0; devConfigIndex < UHS_HOST_MAX_INTERFACE_DRIVERS; devConfigIndex++) {
                if(!devConfig[devConfigIndex]) {
                        HOST_DEBUG("No driver at index %i\r\n", devConfigIndex);
                        continue; // no driver
                }
                if(devConfig[devConfigIndex]->bAddress) {
                        HOST_DEBUG("Driver %i is already consumed @ %2.2x\r\n", devConfigIndex, devConfig[devConfigIndex]->bAddress);
                        continue; // consumed
                }

                if(devConfig[devConfigIndex]->OKtoEnumerate(ei)) {
                        HOST_DEBUG("Driver %i supports this interface\r\n", devConfigIndex);
                        break;
                }
        }
        if(devConfigIndex == UHS_HOST_MAX_INTERFACE_DRIVERS) {
                rcode = UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED;
#if 0 // defined(UHS_HID_LOADED)
                // Check HID here, if it is, then lie
                if(ei->klass == UHS_USB_CLASS_HID) {
                        devConfigIndex = UHS_HID_INDEX; // for debugging, otherwise this has no use.
                        rcode = 0;
                }
#endif
        }
        if(!rcode) HOST_DEBUG("Driver %i can be used for this interface\r\n", devConfigIndex);
        else HOST_DEBUG("No driver for this interface.\r\n");
        return rcode;
};

uint8_t UHS_USB_HOST_BASE::enumerateInterface(ENUMERATION_INFO *ei) {
        uint8_t devConfigIndex;

        HOST_DEBUG("AttemptConfig: parent = %i, port = %i\r\n", ei->parent, ei->port);

#if 0 // defined(UHS_HID_LOADED)
        // Check HID here, if it is, then lie
        if(ei->klass == UHS_USB_CLASS_HID || ei->interface.klass == UHS_USB_CLASS_HID) {
                UHS_HID_SetUSBInterface(this, ENUMERATION_INFO * ei);
                devConfigIndex = UHS_HID_INDEX;
        } else
#endif
                for(devConfigIndex = 0; devConfigIndex < UHS_HOST_MAX_INTERFACE_DRIVERS; devConfigIndex++) {
                        if(!devConfig[devConfigIndex]) {
                                HOST_DEBUG("No driver at index %i\r\n", devConfigIndex);
                                continue; // no driver
                        }
                        if(devConfig[devConfigIndex]->bAddress) {
                                HOST_DEBUG("Driver %i is already consumed @ %2.2x\r\n", devConfigIndex, devConfig[devConfigIndex]->bAddress);
                                continue; // consumed
                        }

                        if(devConfig[devConfigIndex]->OKtoEnumerate(ei)) {
                                HOST_DEBUG("Driver %i supports this interface\r\n", devConfigIndex);
                                if(!devConfig[devConfigIndex]->SetInterface(ei)) break;
                                else devConfigIndex = UHS_HOST_MAX_INTERFACE_DRIVERS;
                        }
                }
        return devConfigIndex;
};

////////////////////////////////////////////////////////////////////////////////
// Vendor Specific Interface Class
////////////////////////////////////////////////////////////////////////////////

#if 0
/**
 * Might go away, depends on if it is useful, or not.
 *
 * @param ei Enumeration information
 * @return true if this interface driver can handle this interface description
 */
bool UHS_NI UHS_VSI::OKtoEnumerate(ENUMERATION_INFO *ei) {
        return (
                (ei->subklass == UHS_USB_CLASS_VENDOR_SPECIFIC) ||
                (ei->interface.subklass == UHS_USB_CLASS_VENDOR_SPECIFIC)
                );
}

/**
 * Copy the entire ENUMERATION_INFO structure
 * @param ei Enumeration information
 * @return 0
 */
uint8_t UHS_NI UHS_VSI::SetInterface(ENUMERATION_INFO *ei) {
        bNumEP = 1;
        bAddress = ei->address;

        eInfo.address = ei->address;
        eInfo.bMaxPacketSize0 = ei->bMaxPacketSize0;
        eInfo.currentconfig = ei->currentconfig;
        eInfo.interface.bAlternateSetting = ei->interface.bAlternateSetting;
        eInfo.interface.bInterfaceNumber = ei->interface.bInterfaceNumber;
        eInfo.interface.numep = ei->interface.numep;
        eInfo.interface.protocol = ei->interface.protocol;
        eInfo.interface.subklass = ei->interface.subklass;
        eInfo.klass = ei->klass;
        eInfo.parent = ei->parent;
        eInfo.pid = ei->pid;
        eInfo.port = ei->port;
        eInfo.protocol = ei->protocol;
        eInfo.subklass = ei->subklass;
        eInfo.vid = ei->vid;
        for(uint8_t i = 0; i < eInfo.interface.numep; i++) {
                eInfo.interface.epInfo[i].bEndpointAddress = ei->interface.epInfo[i].bEndpointAddress;
                eInfo.interface.epInfo[i].bInterval = ei->interface.epInfo[i].bInterval;
                eInfo.interface.epInfo[i].bmAttributes = ei->interface.epInfo[i].bmAttributes;
                eInfo.interface.epInfo[i].wMaxPacketSize = ei->interface.epInfo[i].wMaxPacketSize;
        }
        return 0;
}
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#if 0

/**
 * TODO: Move this silliness to a NONE driver.
 * When we have a generic NONE driver we can:
 *  - Extract ALL device information to help users with a new device.
 *  - Use an unknown device from a sketch, kind of like usblib does.
 *    This will aid in making more drivers in a faster way.
 */
uint8_t UHS_USB_HOST_BASE::DefaultAddressing(uint8_t parent, uint8_t port, uint8_t speed) {
        uint8_t rcode;
        UHS_Device *p0 = NULL, *p = NULL;

        // Get pointer to pseudo device with address 0 assigned
        p0 = addrPool.GetUsbDevicePtr(0);

        if(!p0)
                return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

        if(!p0->epinfo)
                return UHS_HOST_ERROR_NULL_EPINFO;

        p0->speed = speed;

        // Allocate new address according to device class
        uint8_t bAddress = addrPool.AllocAddress(parent, false, port);

        if(!bAddress)
                return UHS_HOST_ERROR_ADDRESS_POOL_FULL;

        p = addrPool.GetUsbDevicePtr(bAddress);

        if(!p)
                return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

        p->speed = speed;

        // Assign new address to the device
        rcode = setAddr(0, bAddress);

        if(rcode) {
                addrPool.FreeAddress(bAddress);
                bAddress = 0;
                return rcode;
        }
        return 0;
}
#endif

#else
#error "Never include UHS_host_INLINE.h, include UHS_host.h instead"
#endif
