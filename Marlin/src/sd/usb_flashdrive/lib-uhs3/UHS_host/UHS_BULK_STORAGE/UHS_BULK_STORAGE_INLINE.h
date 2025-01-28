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

#if defined(LOAD_UHS_BULK_STORAGE) && defined(__UHS_BULK_STORAGE_H__) && !defined(UHS_BULK_STORAGE_LOADED)
#define UHS_BULK_STORAGE_LOADED

// uncomment to get 'printf' console debugging. NOT FOR UNO!
//#define DEBUG_PRINTF_EXTRA_HUGE_UHS_BULK_STORAGE

#if DEBUG_PRINTF_EXTRA_HUGE
#ifdef DEBUG_PRINTF_EXTRA_HUGE_UHS_BULK_STORAGE
#define BS_HOST_DEBUG(...) printf(__VA_ARGS__)
#else
#define BS_HOST_DEBUG(...) VOID0
#endif
#else
#define BS_HOST_DEBUG(...) VOID0
#endif

////////////////////////////////////////////////////////////////////////////////
// Interface code
////////////////////////////////////////////////////////////////////////////////

/**
 * Get the capacity of the media
 *
 * @param lun Logical Unit Number
 * @return media capacity
 */
uint32_t UHS_NI UHS_Bulk_Storage::GetCapacity(uint8_t lun) {
        uint32_t v = 0LU;
        pUsb->DisablePoll();
        if(LUNOk[lun])
                v = CurrentCapacity[lun];
        pUsb->EnablePoll();
        return v;
}

/**
 * Get the sector (block) size used on the media
 *
 * @param lun Logical Unit Number
 * @return media sector size
 */
uint16_t UHS_NI UHS_Bulk_Storage::GetSectorSize(uint8_t lun) {
        uint16_t v = 0U;
        pUsb->DisablePoll();
        if(LUNOk[lun])
                v = CurrentSectorSize[lun];
        pUsb->EnablePoll();
        return v;
}

/**
 * Test if LUN is ready for use
 *
 * @param lun Logical Unit Number
 * @return true if LUN is ready for use
 */
bool UHS_NI UHS_Bulk_Storage::LUNIsGood(uint8_t lun) {
        bool v;
        pUsb->DisablePoll();
        v = LUNOk[lun];
        pUsb->EnablePoll();
        return v;
}

/**
 * Test if LUN is write protected
 *
 * @param lun Logical Unit Number
 * @return cached status of write protect switch
 */
bool UHS_NI UHS_Bulk_Storage::WriteProtected(uint8_t lun) {
        bool v;
        pUsb->DisablePoll();
        v = WriteOk[lun];
        pUsb->EnablePoll();
        return v;
}

/**
 * Wrap and execute a SCSI CDB with length of 6
 *
 * @param cdb CDB to execute
 * @param buf_size Size of expected transaction
 * @param buf Buffer
 * @param dir MASS_CMD_DIR_IN | MASS_CMD_DIR_OUT
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::SCSITransaction6(SCSI_CDB6_t *cdb, uint16_t buf_size, void *buf, uint8_t dir) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        pUsb->DisablePoll();
        // promote buf_size to 32bits.
        UHS_BULK_CommandBlockWrapper cbw = UHS_BULK_CommandBlockWrapper(++dCBWTag, (uint32_t)buf_size, cdb, dir);

#if 0
        // Lets check the CBW here:
        printf("\r\n");
        printf("\r\n");
        uint8_t *dump = (uint8_t*)(&cbw);

        for(int i=0; i<(sizeof (UHS_BULK_CommandBlockWrapper)); i++) {
                printf("%02.2x ", *dump);
                dump++;
        }
        printf("\r\n");
        printf("\r\n");
#endif

        uint8_t v = (HandleSCSIError(Transaction(&cbw, buf_size, buf)));
        pUsb->EnablePoll();
        return v;
}

/**
 * Wrap and execute a SCSI CDB with length of 10
 *
 * @param cdb CDB to execute
 * @param buf_size Size of expected transaction
 * @param buf Buffer
 * @param dir MASS_CMD_DIR_IN | MASS_CMD_DIR_OUT
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::SCSITransaction10(SCSI_CDB10_t *cdb, uint16_t buf_size, void *buf, uint8_t dir) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        pUsb->DisablePoll();
        // promote buf_size to 32bits.
        UHS_BULK_CommandBlockWrapper cbw = UHS_BULK_CommandBlockWrapper(++dCBWTag, (uint32_t)buf_size, cdb, dir);
        //SetCurLUN(cdb->LUN);
        uint8_t v = (HandleSCSIError(Transaction(&cbw, buf_size, buf)));
        pUsb->EnablePoll();
        return v;
}

/**
 * Lock or Unlock the tray or door on device.
 * Caution: Some devices with buggy firmware will lock up.
 *
 * @param lun Logical Unit Number
 * @param lock 1 to lock, 0 to unlock
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::LockMedia(uint8_t lun, uint8_t lock) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        pUsb->DisablePoll();
        Notify(PSTR("\r\nLockMedia\r\n"), 0x80);
        Notify(PSTR("---------\r\n"), 0x80);

        SCSI_CDB6_t cdb = SCSI_CDB6_t(SCSI_CMD_PREVENT_REMOVAL, lun, (uint8_t)0, lock);

        uint8_t v = SCSITransaction6(&cdb, (uint16_t)0, NULL, (uint8_t)UHS_BULK_CMD_DIR_IN);
        pUsb->EnablePoll();
        return v;
}

/**
 * Media control, for spindle motor and media tray or door.
 * This includes CDROM, TAPE and anything with a media loader.
 *
 * @param lun Logical Unit Number
 * @param ctl 0x00 Stop Motor, 0x01 Start Motor, 0x02 Eject Media, 0x03 Load Media
 * @return 0 on success
 */
uint8_t UHS_NI UHS_Bulk_Storage::MediaCTL(uint8_t lun, uint8_t ctl) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        pUsb->DisablePoll();
        Notify(PSTR("\r\nMediaCTL\r\n"), 0x80);
        Notify(PSTR("-----------------\r\n"), 0x80);

        uint8_t rcode = UHS_BULK_ERR_UNIT_NOT_READY;
        if(bAddress) {
                SCSI_CDB6_t cdb = SCSI_CDB6_t(SCSI_CMD_START_STOP_UNIT, lun, ctl & 0x03, 0);
                rcode = SCSITransaction6(&cdb, (uint16_t)0, NULL, (uint8_t)UHS_BULK_CMD_DIR_OUT);
        } else {
                SetCurLUN(lun);
        }
        pUsb->EnablePoll();
        return rcode;
}

/**
 * Read data from media
 *
 * @param lun Logical Unit Number
 * @param addr LBA address on media to read
 * @param bsize size of a block (we should probably use the cached size)
 * @param blocks how many blocks to read
 * @param buf memory that is able to hold the requested data
 * @return 0 on success
 */
uint8_t UHS_NI UHS_Bulk_Storage::Read(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, uint8_t *buf) {
        if(!bAddress) return UHS_BULK_ERR_NO_MEDIA;
        uint8_t er = UHS_BULK_ERR_NO_MEDIA;
        pUsb->DisablePoll();
        if(LUNOk[lun]) {
                Notify(PSTR("\r\nRead LUN:\t"), 0x80);
                D_PrintHex<uint8_t > (lun, 0x90);
                Notify(PSTR("\r\nLBA:\t\t"), 0x90);
                D_PrintHex<uint32_t > (addr, 0x90);
                Notify(PSTR("\r\nblocks:\t\t"), 0x90);
                D_PrintHex<uint8_t > (blocks, 0x90);
                Notify(PSTR("\r\nblock size:\t"), 0x90);
                D_PrintHex<uint16_t > (bsize, 0x90);
                Notify(PSTR("\r\n---------\r\n"), 0x80);
                SCSI_CDB10_t cdb = SCSI_CDB10_t(SCSI_CMD_READ_10, lun, blocks, addr);

again:
                er = SCSITransaction10(&cdb, ((uint16_t)bsize * blocks), buf, (uint8_t)UHS_BULK_CMD_DIR_IN);

                if(er == UHS_BULK_ERR_STALL) {
                        MediaCTL(lun, 1);

                        if(UHS_SLEEP_MS(150)) {
                                if(!TestUnitReady(lun)) goto again;
                        }
                }
        }
        qNextPollTime = millis() + 100;
        pUsb->EnablePoll();

        return er;
}

/**
 * Write data to media
 *
 * @param lun Logical Unit Number
 * @param addr LBA address on media to write
 * @param bsize size of a block (we should probably use the cached size)
 * @param blocks how many blocks to write
 * @param buf memory that contains the data to write
 * @return 0 on success
 */
uint8_t UHS_NI UHS_Bulk_Storage::Write(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, const uint8_t * buf) {
        if(!bAddress) return UHS_BULK_ERR_NO_MEDIA;
        uint8_t er = UHS_BULK_ERR_NO_MEDIA;
        pUsb->DisablePoll();
        if(LUNOk[lun]) {
                if(!WriteOk[lun]) {
                        er = UHS_BULK_ERR_WRITE_PROTECTED;
                } else {
                        Notify(PSTR("\r\nWrite LUN:\t"), 0x80);
                        D_PrintHex<uint8_t > (lun, 0x90);
                        Notify(PSTR("\r\nLBA:\t\t"), 0x90);
                        D_PrintHex<uint32_t > (addr, 0x90);
                        Notify(PSTR("\r\nblocks:\t\t"), 0x90);
                        D_PrintHex<uint8_t > (blocks, 0x90);
                        Notify(PSTR("\r\nblock size:\t"), 0x90);
                        D_PrintHex<uint16_t > (bsize, 0x90);
                        Notify(PSTR("\r\n---------\r\n"), 0x80);
                        SCSI_CDB10_t cdb = SCSI_CDB10_t(SCSI_CMD_WRITE_10, lun, blocks, addr);

again:
                        er = SCSITransaction10(&cdb, ((uint16_t)bsize * blocks), (void*)buf, (uint8_t)UHS_BULK_CMD_DIR_OUT);

                        if(er == UHS_BULK_ERR_WRITE_STALL) {
                                MediaCTL(lun, 1);

                                if(UHS_SLEEP_MS(150)) {
                                        if(!TestUnitReady(lun)) goto again;
                                }
                        }
                }
        }
        qNextPollTime = millis() + 100;
        pUsb->EnablePoll();

        return er;
}

// End of user functions, the remaining code below is driver internals.
// Only developer serviceable parts below!

////////////////////////////////////////////////////////////////////////////////
// Main driver code
////////////////////////////////////////////////////////////////////////////////

UHS_NI UHS_Bulk_Storage::UHS_Bulk_Storage(UHS_USB_HOST_BASE *p) {
        pUsb = p;
        dCBWTag = 0;
        if(pUsb) {

                DriverDefaults();
                pUsb->RegisterDeviceClass(this);
                //                Serial.print("Bulk Register to USB Host @ 0x");
                //                Serial.println((uint32_t)pUsb, HEX);
                //                Serial.print("Bulk Register to USB Host Address Pool @ 0x");
                //                Serial.println((uint32_t)pUsb->GetAddressPool(), HEX);
        }
}

/**
 * @param ei Enumeration information
 * @return true if this interface driver can handle this interface description
 */
bool UHS_NI UHS_Bulk_Storage::OKtoEnumerate(ENUMERATION_INFO *ei) {
        BS_HOST_DEBUG("BulkOnly: checking numep %i, klass %2.2x, subklass %2.2x\r\n", ei->interface.numep, ei->klass, ei->subklass);
        BS_HOST_DEBUG("BulkOnly: checking protocol %2.2x, interface.klass %2.2x, interface.subklass %2.2x\r\n", ei->protocol, ei->interface.klass, ei->interface.subklass);
        BS_HOST_DEBUG("BulkOnly: checking interface.protocol %2.2x\r\n", ei->interface.protocol);
        //
        // TODO:
        // Check that we have 2 bulk endpoints, and one in each direction??
        // e.g.       (ei->interface.numep > 1) && // two or more endpoints AND check types
        // This will work with proper hardware though.
        //

        return (
                ((ei->klass == UHS_USB_CLASS_MASS_STORAGE) || (ei->interface.klass == UHS_USB_CLASS_MASS_STORAGE)) && // mass storage class AND
                ((ei->subklass == UHS_BULK_SUBCLASS_SCSI) || (ei->interface.subklass == UHS_BULK_SUBCLASS_SCSI)) && // SCSI command set AND
                ((ei->protocol == UHS_STOR_PROTO_BBB) || (ei->interface.protocol == UHS_STOR_PROTO_BBB)) // Bulk Only transport
                );
}

/**
 * @param ei Enumeration information
 * @return 0 always
 */
uint8_t UHS_NI UHS_Bulk_Storage::SetInterface(ENUMERATION_INFO *ei) {
        uint8_t index;

        bAddress = ei->address;
        BS_HOST_DEBUG("BS SetInterface\r\n");
        // Fill in the endpoint info structure
        for(uint8_t ep = 0; ep < ei->interface.numep; ep++) {
                BS_HOST_DEBUG("ep: 0x%2.2x bmAttributes: 0x%2.2x ", ep, ei->interface.epInfo[ep].bmAttributes);
                if(ei->interface.epInfo[ep].bmAttributes == USB_TRANSFER_TYPE_BULK) {
                        index = ((ei->interface.epInfo[ep].bEndpointAddress & USB_TRANSFER_DIRECTION_IN) == USB_TRANSFER_DIRECTION_IN) ? epDataInIndex : epDataOutIndex;
                        epInfo[index].epAddr = (ei->interface.epInfo[ep].bEndpointAddress & 0x0F);
                        epInfo[index].maxPktSize = ei->interface.epInfo[ep].wMaxPacketSize;
                        epInfo[index].epAttribs = 0;
                        epInfo[index].bmNakPower = UHS_USB_NAK_MAX_POWER;
                        epInfo[index].bmSndToggle = 0;
                        epInfo[index].bmRcvToggle = 0;
                        epInfo[index].bIface=ei->interface.bInterfaceNumber;
                        BS_HOST_DEBUG("index: %i\r\n", index);
                }
                BS_HOST_DEBUG("\r\n");
        }
        bNumEP = 3;
        epInfo[0].epAddr = 0;
        epInfo[0].maxPktSize = ei->bMaxPacketSize0;
        epInfo[0].bmNakPower = UHS_USB_NAK_MAX_POWER;
        bIface = ei->interface.bInterfaceNumber;

        return 0;
};

/**
 * @return 0 for success
 */
uint8_t UHS_NI UHS_Bulk_Storage::Start() {
        uint8_t rcode;
        //        Serial.print("Bulk Start from USB Host @ 0x");
        //        Serial.println((uint32_t)pUsb, HEX);
        //        Serial.print("Bulk Start USB Host Address Pool @ 0x");
        //        Serial.println((uint32_t)pUsb->GetAddressPool(), HEX);

        BS_HOST_DEBUG("BS Start, speed: %i\r\n", pUsb->GetAddressPool()->GetUsbDevicePtr(bAddress)->speed);
        BS_HOST_DEBUG("BS Start\r\n");
        rcode = pUsb->setEpInfoEntry(bAddress, bIface, 3, epInfo);
        //        Serial.println(rcode,HEX);
        if(rcode) goto FailOnInit;

        // Do a 1 second delay before LUN query
        if(!UHS_SLEEP_MS(1000)) goto FailUnPlug;

        rcode = GetMaxLUN(&bMaxLUN);
        BS_HOST_DEBUG("GetMaxLUN 0x%2.2x\r\n", rcode);
        if(rcode) {
                goto FailGetMaxLUN;
        }
        if(bMaxLUN >= MASS_MAX_SUPPORTED_LUN) bMaxLUN = MASS_MAX_SUPPORTED_LUN - 1;
        BS_HOST_DEBUG("MaxLUN %u\r\n", bMaxLUN);
        //ErrorMessage<uint8_t > (PSTR("MaxLUN"), bMaxLUN);
        if(!UHS_SLEEP_MS(150)) goto FailUnPlug; // Delay a bit for slow firmware. (again)

        for(uint8_t lun = 0; lun <= bMaxLUN; lun++) {
                if(!UHS_SLEEP_MS(3)) goto FailUnPlug;
                SCSI_Inquiry_Response response;
                rcode = Inquiry(lun, sizeof (SCSI_Inquiry_Response), (uint8_t*) & response);
                BS_HOST_DEBUG("Inquiry 0x%2.2x 0x%2.2x\r\n", sizeof (SCSI_Inquiry_Response), rcode);
                if(rcode) {
                        goto FailInquiry;
#if 0
                } else {
                        BS_HOST_DEBUG("LUN %i `", lun);
                        uint8_t *buf = response.VendorID;
                        for(int i = 0; i < 28; i++) BS_HOST_DEBUG("%c", buf[i]);
                        BS_HOST_DEBUG("'\r\nQualifier %1.1X ", response.PeripheralQualifier);
                        BS_HOST_DEBUG("Device type %2.2X ", response.DeviceType);
                        BS_HOST_DEBUG("RMB %1.1X ", response.Removable);
                        BS_HOST_DEBUG("SSCS %1.1X ", response.SCCS);
                        uint8_t sv = response.Version;
                        BS_HOST_DEBUG("SCSI version %2.2X\r\nDevice conforms to ", sv);
                        switch(sv) {
                                case 0:
                                        BS_HOST_DEBUG("No specific");
                                        break;
                                case 1:
                                        BS_HOST_DEBUG("ANSI X3.131-1986 (ANSI 1)");
                                        break;
                                case 2:
                                        BS_HOST_DEBUG("ANSI X3.131-1994 (ANSI 2)");
                                        break;
                                case 3:
                                        BS_HOST_DEBUG("ANSI INCITS 301-1997 (SPC)");
                                        break;
                                case 4:
                                        BS_HOST_DEBUG("ANSI INCITS 351-2001 (SPC-2)");
                                        break;
                                case 5:
                                        BS_HOST_DEBUG("ANSI INCITS 408-2005 (SPC-4)");
                                        break;
                                case 6:
                                        BS_HOST_DEBUG("T10/1731-D (SPC-4)");
                                        break;
                                default:
                                        BS_HOST_DEBUG("unknown");
                        }
                        BS_HOST_DEBUG(" standards.\r\n");
#endif
                }
        }

        for(uint8_t lun = 0; lun <= bMaxLUN; lun++) {
                if(!UHS_SLEEP_MS(3)) goto FailUnPlug;
                #ifndef USB_NO_TEST_UNIT_READY
                uint8_t tries = 0xF0;
                while((rcode = TestUnitReady(lun))) {
                        BS_HOST_DEBUG("\r\nTry %2.2x TestUnitReady %2.2x\r\n", tries - 0xF0, rcode);
                        if(rcode == 0x08) break; // break on no media, this is OK to do.
                        if(rcode == UHS_BULK_ERR_DEVICE_DISCONNECTED) goto FailUnPlug;
                        if(rcode == UHS_BULK_ERR_INVALID_CSW) goto Fail;
                        if(rcode != UHS_BULK_ERR_MEDIA_CHANGED) goto Fail;
                        if(!UHS_SLEEP_MS(2 * (tries + 1))) goto FailUnPlug;
                        tries++;
                        if(!tries) break;
                }
                #else
                        // Don't wait for the LUN to become ready, as this will
                        // trigger Marlin's watchdog timer
                        rcode = -1;
                #endif
                if(!UHS_SLEEP_MS(3)) goto FailUnPlug;
                LockMedia(lun, 1);
                if(rcode == 0x08) {
                        if(!UHS_SLEEP_MS(3)) goto FailUnPlug;
                        if(MediaCTL(lun, 1) == UHS_BULK_ERR_DEVICE_DISCONNECTED) goto FailUnPlug; // I actually have a USB stick that needs this!
                }
                BS_HOST_DEBUG("\r\nTry %2.2x TestUnitReady %2.2x\r\n", tries - 0xF0, rcode);
                if(!rcode) {
                        if(!UHS_SLEEP_MS(3)) goto FailUnPlug;
                        BS_HOST_DEBUG("CheckLUN...\r\n");
                        BS_HOST_DEBUG("%lu\r\n", millis()/1000);
                        // Stalls on ***some*** devices, ***WHY***?! Device SAID it is READY!!
                        LUNOk[lun] = CheckLUN(lun);
                        BS_HOST_DEBUG("%lu\r\n", millis()/1000);
                        if(!LUNOk[lun]) LUNOk[lun] = CheckLUN(lun);
                        if(!UHS_SLEEP_MS(1)) goto FailUnPlug;
                        BS_HOST_DEBUG("Checked LUN...\r\n");
                } else {
                        LUNOk[lun] = false;
                }
        }

        rcode = OnStart();

        if(rcode) goto FailOnInit;

#ifdef DEBUG_USB_HOST
        USBTRACE("BS configured\r\n\r\n");
#endif
        qNextPollTime = millis() + 100;
        bPollEnable = true;

        return 0;
FailUnPlug:
        rcode = UHS_BULK_ERR_DEVICE_DISCONNECTED;
        goto Fail;

FailOnInit:
#ifdef DEBUG_USB_HOST
        USBTRACE("OnStart:");
        goto Fail;
#endif

FailGetMaxLUN:
#ifdef DEBUG_USB_HOST
        USBTRACE("GetMaxLUN:");
        goto Fail;
#endif

FailInquiry:
#ifdef DEBUG_USB_HOST
        USBTRACE("Inquiry:");
#endif

Fail:
#ifdef DEBUG_USB_HOST
        NotifyFail(rcode);
#endif
        Release();

        return rcode;
}

// Base class definition of Release() used. See UHS_USBInterface class definition for details

/**
 * For driver use only.
 *
 * @return
 */
//void UHS_NI UHS_Bulk_Storage::Release() {
//        pUsb->DisablePoll();
//        OnRelease();
//        DriverDefaults();
//        pUsb->EnablePoll();
//        return;
//}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @return true if LUN is ready for use.
 */
bool UHS_NI UHS_Bulk_Storage::CheckLUN(uint8_t lun) {
        uint8_t rcode;
        SCSI_Capacity capacity;
        for(uint8_t i = 0; i < 8; i++) capacity.data[i] = 0;

        rcode = ReadCapacity10(lun, (uint8_t*)capacity.data);
        if(rcode) {
                BS_HOST_DEBUG(">>>>>>>>>>>>>>>>ReadCapacity returned %i\r\n", rcode);
                return false;
        }
#ifdef DEBUG_USB_HOST
        ErrorMessage<uint8_t > (PSTR(">>>>>>>>>>>>>>>>CAPACITY OK ON LUN"), lun);
        for(uint8_t i = 0; i < 8 /*sizeof (Capacity)*/; i++)
                D_PrintHex<uint8_t > (capacity.data[i], 0x80);
        Notify(PSTR("\r\n\r\n"), 0x80);
#endif
        // Only 512/1024/2048/4096 are valid values!
        uint32_t c = UHS_BYTES_TO_UINT32(capacity.data[4], capacity.data[5], capacity.data[6], capacity.data[7]);
        if(c != 0x0200LU && c != 0x0400LU && c != 0x0800LU && c != 0x1000LU) {
                return false;
        }
        // Store capacity information.
        CurrentSectorSize[lun] = (uint16_t)(c); // & 0xFFFF);

        CurrentCapacity[lun] = UHS_BYTES_TO_UINT32(capacity.data[0], capacity.data[1], capacity.data[2], capacity.data[3]) + 1;
        if(CurrentCapacity[lun] == /*0xffffffffLU */ 0x01LU || CurrentCapacity[lun] == 0x00LU) {
                // Buggy firmware will report 0xFFFFFFFF or 0 for no media
#ifdef DEBUG_USB_HOST
                if(CurrentCapacity[lun])
                        ErrorMessage<uint8_t > (PSTR(">>>>>>>>>>>>>>>>BUGGY FIRMWARE. CAPACITY FAIL ON LUN"), lun);
#endif
                return false;
        }
        if(!UHS_SLEEP_MS(20)) return false;
        #ifndef SKIP_PAGE3F
        Page3F(lun);
        #endif
        if(!TestUnitReady(lun)) return true;

        return false;
}

/**
 * For driver use only.
 *
 * Scan for media change on all LUNs
 */
void UHS_NI UHS_Bulk_Storage::CheckMedia() {
        if(!bAddress) return;
        for(uint8_t lun = 0; lun <= bMaxLUN; lun++) {
                if(TestUnitReady(lun)) {
                        LUNOk[lun] = false;
                        continue;
                }
                if(!LUNOk[lun])
                        LUNOk[lun] = CheckLUN(lun);
        }
#if 0
        BS_HOST_DEBUG("}}}}}}}}}}}}}}}}STATUS ");
        for(uint8_t lun = 0; lun <= bMaxLUN; lun++) {
                if(LUNOk[lun])
                        BS_HOST_DEBUG("#");

                else BS_HOST_DEBUG(".");
        }
        BS_HOST_DEBUG("\r\n");
#endif
        OnPoll();
        qNextPollTime = millis() + 100;
}

/**
 * For driver use only.
 */
void UHS_NI UHS_Bulk_Storage::Poll() {
        if((long)(millis() - qNextPollTime) >= 0L) {

                CheckMedia();
        }

        return;
}

////////////////////////////////////////////////////////////////////////////////
// SCSI code
////////////////////////////////////////////////////////////////////////////////

/**
 * For driver use only.
 *
 * @param plun
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::GetMaxLUN(uint8_t *plun) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        uint8_t ret = pUsb->ctrlReq(bAddress, mkSETUP_PKT16(UHS_BULK_bmREQ_IN, UHS_BULK_REQ_GET_MAX_LUN, 0x0000U, bIface, 1), 1, plun);

        if(ret == UHS_HOST_ERROR_STALL) {

                *plun = 0;
                Notify(PSTR("\r\nGetMaxLUN Stalled\r\n"), 0x80);
        }
        return 0;
}

/**
 * For driver use only. Used during Driver Start
 *
 * @param lun Logical Unit Number
 * @param bsize
 * @param buf
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::Inquiry(uint8_t lun, uint16_t bsize, uint8_t *buf) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        Notify(PSTR("\r\nInquiry\r\n"), 0x80);
        Notify(PSTR("---------\r\n"), 0x80);

        SCSI_CDB6_t cdb = SCSI_CDB6_t(SCSI_CMD_INQUIRY, lun, 0LU, (uint8_t)bsize, 0);
        uint8_t rc = SCSITransaction6(&cdb, bsize, buf, (uint8_t)UHS_BULK_CMD_DIR_IN);

        return rc;
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::TestUnitReady(uint8_t lun) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        //SetCurLUN(lun);
        if(!bAddress)
                return UHS_BULK_ERR_UNIT_NOT_READY;

        Notify(PSTR("\r\nTestUnitReady\r\n"), 0x80);
        Notify(PSTR("-----------------\r\n"), 0x80);

        SCSI_CDB6_t cdb = SCSI_CDB6_t(SCSI_CMD_TEST_UNIT_READY, lun, (uint8_t)0, 0);

        return SCSITransaction6(&cdb, 0, NULL, (uint8_t)UHS_BULK_CMD_DIR_IN);

}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @param pc
 * @param page
 * @param subpage
 * @param len
 * @param pbuf
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::ModeSense6(uint8_t lun, uint8_t pc, uint8_t page, uint8_t subpage, uint8_t len, uint8_t * pbuf) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        Notify(PSTR("\r\rModeSense\r\n"), 0x80);
        Notify(PSTR("------------\r\n"), 0x80);

        SCSI_CDB6_t cdb = SCSI_CDB6_t(SCSI_CMD_MODE_SENSE_6, lun, (uint32_t)((((pc << 6) | page) << 8) | subpage), len, 0);

        return SCSITransaction6(&cdb, len, pbuf, (uint8_t)UHS_BULK_CMD_DIR_IN);
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @param bsize
 * @param buf
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::ReadCapacity10(uint8_t lun, uint8_t *buf) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        Notify(PSTR("\r\nReadCapacity\r\n"), 0x80);
        Notify(PSTR("---------------\r\n"), 0x80);

        SCSI_CDB10_t cdb = SCSI_CDB10_t(SCSI_CMD_READ_CAPACITY_10, lun);

        return SCSITransaction10(&cdb, 8, buf, (uint8_t)UHS_BULK_CMD_DIR_IN);
}

/**
 * For driver use only.
 *
 * Page 3F contains write protect status.
 *
 * @param lun Logical Unit Number to test.
 * @return Write protect switch status.
 */
uint8_t UHS_NI UHS_Bulk_Storage::Page3F(uint8_t lun) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        uint8_t buf[192];
        for(int i = 0; i < 192; i++) {
                buf[i] = 0x00;
        }
        WriteOk[lun] = true;
        uint8_t rc = ModeSense6(lun, 0, 0x3F, 0, 192, buf);
        if(!rc) {
                WriteOk[lun] = ((buf[2] & 0x80) == 0);
#ifdef DEBUG_USB_HOST
                Notify(PSTR("Mode Sense: "), 0x80);
                for(int i = 0; i < 4; i++) {

                        D_PrintHex<uint8_t > (buf[i], 0x80);
                        Notify(PSTR(" "), 0x80);
                }
                Notify(PSTR("\r\n"), 0x80);
#endif
        }
        return rc;
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @param size
 * @param buf
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::RequestSense(uint8_t lun, uint16_t size, uint8_t *buf) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        pUsb->DisablePoll();
        Notify(PSTR("\r\nRequestSense\r\n"), 0x80);
        Notify(PSTR("----------------\r\n"), 0x80);

        SCSI_CDB6_t cdb = SCSI_CDB6_t(SCSI_CMD_REQUEST_SENSE, lun, 0LU, (uint8_t)size, 0);
        UHS_BULK_CommandBlockWrapper cbw = UHS_BULK_CommandBlockWrapper(++dCBWTag, (uint32_t)size, &cdb, (uint8_t)UHS_BULK_CMD_DIR_IN);
        uint8_t v = Transaction(&cbw, size, buf);
        pUsb->EnablePoll();

        return v;
}

////////////////////////////////////////////////////////////////////////////////
// USB code
////////////////////////////////////////////////////////////////////////////////

/**
 * For driver use only.
 *
 * @param index
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::ClearEpHalt(uint8_t index) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        uint8_t ret = 0;
        if(index != 0) {
                uint8_t ep = (index == epDataInIndex) ? (0x80 | epInfo[index].epAddr) : epInfo[index].epAddr;
                do {
                        ret = pUsb->EPClearHalt(bAddress, ep);
                        if(!UHS_SLEEP_MS(6)) break;
                } while(ret == 0x01);

                if(ret) {
                        ErrorMessage<uint8_t > (PSTR("ClearEpHalt"), ret);
                        ErrorMessage<uint8_t > (PSTR("EP"), ep);
                        epInfo[index].bmSndToggle = 0;
                        epInfo[index].bmRcvToggle = 0;
                        return ret;
                } else {

                        epInfo[index].bmSndToggle = 0;
                        epInfo[index].bmRcvToggle = 0;
                }
        }
        return ret;
}

/**
 * For driver use only.
 */
void UHS_NI UHS_Bulk_Storage::Reset() {
        if(!bAddress) return;

        while(pUsb->ctrlReq(bAddress, mkSETUP_PKT16(UHS_BULK_bmREQ_OUT, UHS_BULK_REQ_BOMSR, 0x0000U, bIface, 0), 0, NULL) == 0x01) {
                if(!UHS_SLEEP_MS(6)) break;
        }

        if(!bAddress) return;

        UHS_SLEEP_MS(2500);
}

/**
 * For driver use only.
 *
 * @return 0 if successful
 */
uint8_t UHS_NI UHS_Bulk_Storage::ResetRecovery() {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        Notify(PSTR("\r\nResetRecovery\r\n"), 0x80);
        Notify(PSTR("-----------------\r\n"), 0x80);
        qNextPollTime = millis() + 90000;
        uint8_t bLastUsbError = UHS_HOST_ERROR_UNPLUGGED;
        if(UHS_SLEEP_MS(6)) {
                Reset();
                if(UHS_SLEEP_MS(6)) {
                        bLastUsbError = ClearEpHalt(epDataInIndex);
                        if(UHS_SLEEP_MS(6)) {

                                bLastUsbError = ClearEpHalt(epDataOutIndex);
                                UHS_SLEEP_MS(6);
                        }
                }
        }
        return bLastUsbError;
}

/**
 * For driver use only.
 *
 * Clear all EP data and clear all LUN status
 */
void UHS_NI UHS_Bulk_Storage::DriverDefaults() {

        pUsb->DeviceDefaults(MASS_MAX_ENDPOINTS, this);

        for(uint8_t i = 0; i < MASS_MAX_SUPPORTED_LUN; i++) {

                LUNOk[i] = false;
                WriteOk[i] = false;
                CurrentCapacity[i] = 0lu;
                CurrentSectorSize[i] = 0;
        }

        dCBWTag = 0;
        bMaxLUN = 0;
        bTheLUN = 0;
}

/**
 * For driver use only.
 *
 * @param pcsw
 * @param pcbw
 * @return
 */
bool UHS_NI UHS_Bulk_Storage::IsValidCSW(UHS_BULK_CommandStatusWrapper *pcsw, UHS_BULK_CommandBlockWrapperBase *pcbw) {
        if(!bAddress) return false;
        if(pcsw->dCSWSignature != UHS_BULK_CSW_SIGNATURE) {
                Notify(PSTR("CSW:Sig error\r\n"), 0x80);
                return false;
        }
        if(pcsw->dCSWTag != pcbw->dCBWTag) {
                Notify(PSTR("CSW:Wrong tag\r\n"), 0x80);
                ErrorMessage<uint32_t > (PSTR("dCSWTag"), pcsw->dCSWTag);
                ErrorMessage<uint32_t > (PSTR("dCBWTag"), pcbw->dCBWTag);

                return false;
        }
        return true;
}

/**
 * For driver use only.
 *
 * @param error
 * @param index
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::HandleUsbError(uint8_t error, uint8_t index) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;

        uint8_t count = 3;
        while(error && count) {
                if(error != UHS_HOST_ERROR_NONE) {
                        ErrorMessage<uint8_t > (PSTR("USB Error"), error);
                        ErrorMessage<uint8_t > (PSTR("Index"), index);
                }
                switch(error) {
                                // case UHS_HOST_ERROR_WRONGPID:
                        case UHS_HOST_ERROR_NONE:
                                return UHS_BULK_ERR_SUCCESS;
                        case UHS_HOST_ERROR_BUSY:
                                // SIE is busy, just hang out and try again.
                                return UHS_BULK_ERR_UNIT_BUSY;
                        case UHS_HOST_ERROR_NAK:
                                return UHS_BULK_ERR_UNIT_BUSY;
                        case UHS_HOST_ERROR_UNPLUGGED:
                        case UHS_HOST_ERROR_TIMEOUT:
                        case UHS_HOST_ERROR_JERR:
                                return UHS_BULK_ERR_DEVICE_DISCONNECTED;
                        case UHS_HOST_ERROR_STALL:
                                if(index == 0) return UHS_BULK_ERR_STALL;
                                ClearEpHalt(index);
                                if(index != epDataInIndex) return UHS_BULK_ERR_WRITE_STALL;
                                return UHS_BULK_ERR_STALL;

                        case UHS_HOST_ERROR_TOGERR:
                                // Handle a very super rare corner case, where toggles become de-synched.
                                // I have only ran into one device that has this firmware bug, and this is
                                // the only clean way to get back into sync with the buggy device firmware.
                                //   --AJK
                                if(bAddress && bConfNum) {
                                        error = pUsb->setConf(bAddress, bConfNum);

                                        if(error) break;
                                }
                                return UHS_BULK_ERR_SUCCESS;

                        default:
                                ErrorMessage<uint8_t > (PSTR("\r\nUSB"), error);

                                return UHS_BULK_ERR_GENERAL_USB_ERROR;
                }
                count--;
        } // while

        return ((error && !count) ? UHS_BULK_ERR_GENERAL_USB_ERROR : UHS_BULK_ERR_SUCCESS);
}

/**
 * For driver use only.
 *
 * @param pcbw
 * @param buf_size
 * @param buf
 * @param flags
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::Transaction(UHS_BULK_CommandBlockWrapper *pcbw, uint16_t buf_size, void *buf) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;

        uint16_t bytes = buf_size;
        bool write = (pcbw->bmCBWFlags & UHS_BULK_CMD_DIR_IN) != UHS_BULK_CMD_DIR_IN;
        uint8_t ret = 0;
        uint8_t usberr;
        UHS_BULK_CommandStatusWrapper csw; // up here, we allocate ahead to save cpu cycles.
        SetCurLUN(pcbw->bmCBWLUN);
        ErrorMessage<uint32_t > (PSTR("CBW.dCBWTag"), pcbw->dCBWTag);

        while((usberr = pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].epAddr, sizeof (UHS_BULK_CommandBlockWrapper), (uint8_t*)pcbw)) == UHS_HOST_ERROR_BUSY) {
                if(!UHS_SLEEP_MS(1)) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        }
        ret = HandleUsbError(usberr, epDataOutIndex);
        if(ret) {
                ErrorMessage<uint8_t > (PSTR("============================ CBW"), ret);
        } else {
                if(bytes) {
                        if(!write) {
                                while((usberr = pUsb->inTransfer(bAddress, epInfo[epDataInIndex].epAddr, &bytes, (uint8_t*)buf)) == UHS_HOST_ERROR_BUSY) {
                                        if(!UHS_SLEEP_MS(1)) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
                                }
                                ret = HandleUsbError(usberr, epDataInIndex);
                        } else {
                                while((usberr = pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].epAddr, bytes, (uint8_t*)buf)) == UHS_HOST_ERROR_BUSY) {
                                        if(!UHS_SLEEP_MS(1)) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
                                }
                                ret = HandleUsbError(usberr, epDataOutIndex);
                        }
                        if(ret) {
                                ErrorMessage<uint8_t > (PSTR("============================ DAT"), ret);
                        }
                }
        }

        {
                bytes = sizeof (UHS_BULK_CommandStatusWrapper);
                int tries = 2;
                while(tries--) {
                        while((usberr = pUsb->inTransfer(bAddress, epInfo[epDataInIndex].epAddr, &bytes, (uint8_t*) & csw)) == UHS_HOST_ERROR_BUSY) {
                                if(!UHS_SLEEP_MS(1)) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
                        }
                        if(!usberr) break;
                        if(tries) {
                                if(usberr == UHS_HOST_ERROR_STALL) {
                                        ResetRecovery();
                                } else {
                                        ClearEpHalt(epDataInIndex);
                                }
                        }
                }
                if(!ret) {
                        Notify(PSTR("CBW:\t\tOK\r\n"), 0x80);
                        Notify(PSTR("Data Stage:\tOK\r\n"), 0x80);
                } else {
                        // Throw away csw, IT IS NOT OF ANY USE.
                        ResetRecovery();
                        return ret;
                }
                ret = HandleUsbError(usberr, epDataInIndex);
                if(ret) {
                        ErrorMessage<uint8_t > (PSTR("============================ CSW"), ret);
                }
                if(usberr == UHS_HOST_ERROR_NONE) {
                        if(IsValidCSW(&csw, pcbw)) {
                                //ErrorMessage<uint32_t > (PSTR("CSW.dCBWTag"), csw.dCSWTag);
                                //ErrorMessage<uint8_t > (PSTR("bCSWStatus"), csw.bCSWStatus);
                                //ErrorMessage<uint32_t > (PSTR("dCSWDataResidue"), csw.dCSWDataResidue);
                                Notify(PSTR("CSW:\t\tOK\r\n\r\n"), 0x80);
                                return csw.bCSWStatus;
                        } else {
                                // NOTE! Sometimes this is caused by the reported residue being wrong.
                                // Get a different device. It isn't compliant, and should have never passed Q&A.
                                // I own one... 05e3:0701 Genesys Logic, Inc. USB 2.0 IDE Adapter.
                                // Other devices that exhibit this behavior exist in the wild too.
                                // Be sure to check quirks in the Linux source code before reporting a bug. --xxxajk
                                Notify(PSTR("Invalid CSW\r\n"), 0x80);
                                Reset();
                                ResetRecovery();

                                return UHS_BULK_ERR_INVALID_CSW;
                        }
                }
        }
        return ret;
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::SetCurLUN(uint8_t lun) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        if(lun > bMaxLUN)
                return UHS_BULK_ERR_INVALID_LUN;
        bTheLUN = lun;

        return UHS_BULK_ERR_SUCCESS;
};

/**
 * For driver use only.
 *
 * @param status
 * @return
 */
uint8_t UHS_NI UHS_Bulk_Storage::HandleSCSIError(uint8_t status) {
        if(!bAddress) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
        uint8_t ret = 0;
        switch(status) {
                case 0: return UHS_BULK_ERR_SUCCESS;

                case 2:
                        ErrorMessage<uint8_t > (PSTR("Phase Error"), status);
                        ErrorMessage<uint8_t > (PSTR("LUN"), bTheLUN);
                        ResetRecovery();
                        return UHS_BULK_ERR_GENERAL_SCSI_ERROR;

                case 1:
                        ErrorMessage<uint8_t > (PSTR("SCSI Error"), status);
                        ErrorMessage<uint8_t > (PSTR("LUN"), bTheLUN);
                        SCSI_Request_Sense_Response rsp;

                        ret = RequestSense(bTheLUN, sizeof (SCSI_Request_Sense_Response), (uint8_t*) & rsp);

                        if(ret) {
                                if(ret == UHS_BULK_ERR_DEVICE_DISCONNECTED) return UHS_BULK_ERR_DEVICE_DISCONNECTED;
                                return UHS_BULK_ERR_GENERAL_SCSI_ERROR;
                        }
#if ENABLE_UHS_DEBUGGING
                        ErrorMessage<uint8_t > (PSTR("Response Code"), rsp.bResponseCode);
                        if(rsp.bResponseCode & 0x80) {
                                Notify(PSTR("Information field: "), 0x80);
                                for(int i = 0; i < 4; i++) {
                                        D_PrintHex<uint8_t > (rsp.CmdSpecificInformation[i], 0x80);
                                        Notify(PSTR(" "), 0x80);
                                }
                                Notify(PSTR("\r\n"), 0x80);
                        }
                        ErrorMessage<uint8_t > (PSTR("Sense Key"), rsp.bmSenseKey);
                        ErrorMessage<uint8_t > (PSTR("Add Sense Code"), rsp.bAdditionalSenseCode);
                        ErrorMessage<uint8_t > (PSTR("Add Sense Qual"), rsp.bAdditionalSenseQualifier);
#endif
                        // warning, this is not testing ASQ, only SK and ASC.
                        switch(rsp.bmSenseKey) {
                                case SCSI_S_UNIT_ATTENTION:
                                        switch(rsp.bAdditionalSenseCode) {
                                                case SCSI_ASC_MEDIA_CHANGED:
                                                        return UHS_BULK_ERR_MEDIA_CHANGED;
                                                default:
                                                        return UHS_BULK_ERR_UNIT_NOT_READY;
                                        }
                                case SCSI_S_NOT_READY:
                                        switch(rsp.bAdditionalSenseCode) {
                                                case SCSI_ASC_MEDIUM_NOT_PRESENT:
                                                        return UHS_BULK_ERR_NO_MEDIA;
                                                default:
                                                        return UHS_BULK_ERR_UNIT_NOT_READY;
                                        }
                                case SCSI_S_ILLEGAL_REQUEST:
                                        switch(rsp.bAdditionalSenseCode) {
                                                case SCSI_ASC_LBA_OUT_OF_RANGE:
                                                        return UHS_BULK_ERR_BAD_LBA;
                                                default:
                                                        return UHS_BULK_ERR_CMD_NOT_SUPPORTED;
                                        }
                                default:
                                        return UHS_BULK_ERR_GENERAL_SCSI_ERROR;
                        }

                        // case 4: return MASS_ERR_UNIT_BUSY; // Busy means retry later.
                        //    case 0x05/0x14: we stalled out
                        //    case 0x15/0x16: we naked out.
                default:
                        ErrorMessage<uint8_t > (PSTR("Gen SCSI Err"), status);
                        ErrorMessage<uint8_t > (PSTR("LUN"), bTheLUN);

                        return status;
        } // switch
}

////////////////////////////////////////////////////////////////////////////////
// Debugging code
////////////////////////////////////////////////////////////////////////////////

/**
 * @param ep_ptr
 */
void UHS_NI UHS_Bulk_Storage::PrintEndpointDescriptor(const USB_FD_ENDPOINT_DESCRIPTOR * ep_ptr) {
        Notify(PSTR("Endpoint descriptor:"), 0x80);
        Notify(PSTR("\r\nLength:\t\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bLength, 0x80);
        Notify(PSTR("\r\nType:\t\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bDescriptorType, 0x80);
        Notify(PSTR("\r\nAddress:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bEndpointAddress, 0x80);
        Notify(PSTR("\r\nAttributes:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bmAttributes, 0x80);
        Notify(PSTR("\r\nMaxPktSize:\t"), 0x80);
        D_PrintHex<uint16_t > (ep_ptr->wMaxPacketSize, 0x80);
        Notify(PSTR("\r\nPoll Intrv:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bInterval, 0x80);
        Notify(PSTR("\r\n"), 0x80);
}

#else
#error "Never include UHS_BULK_STORAGE_INLINE.h, include UHS_host.h instead"
#endif
