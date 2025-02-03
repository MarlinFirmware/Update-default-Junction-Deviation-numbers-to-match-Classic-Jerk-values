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

#ifndef UHS_SCSI_H
#define UHS_SCSI_H

/**
 * Reference documents from T10 (https://www.t10.org)
 * SCSI Primary Commands - 3 (SPC-3)
 * SCSI Block Commands - 2 (SBC-2)
 * Multi-Media Commands - 5 (MMC-5)
 */

// Group 1 commands (CDB's here are should all be 6-bytes)
#define SCSI_CMD_TEST_UNIT_READY        0x00U
#define SCSI_CMD_REQUEST_SENSE          0x03U
#define SCSI_CMD_FORMAT_UNIT            0x04U
#define SCSI_CMD_READ_6                 0x08U
#define SCSI_CMD_WRITE_6                0x0AU
#define SCSI_CMD_INQUIRY                0x12U
#define SCSI_CMD_MODE_SELECT_6          0x15U
#define SCSI_CMD_MODE_SENSE_6           0x1AU
#define SCSI_CMD_START_STOP_UNIT        0x1BU
#define SCSI_CMD_PREVENT_REMOVAL        0x1EU

// Group 2 Commands (CDB's here are 10-bytes)
#define SCSI_CMD_READ_FORMAT_CAPACITIES 0x23U
#define SCSI_CMD_READ_CAPACITY_10       0x25U
#define SCSI_CMD_READ_10                0x28U
#define SCSI_CMD_WRITE_10               0x2AU
#define SCSI_CMD_SEEK_10                0x2BU
#define SCSI_CMD_ERASE_10               0x2CU
#define SCSI_CMD_WRITE_AND_VERIFY_10    0x2EU
#define SCSI_CMD_VERIFY_10              0x2FU
#define SCSI_CMD_SYNCHRONIZE_CACHE      0x35U
#define SCSI_CMD_WRITE_BUFFER           0x3BU
#define SCSI_CMD_READ_BUFFER            0x3CU
#define SCSI_CMD_READ_SUBCHANNEL        0x42U
#define SCSI_CMD_READ_TOC               0x43U
#define SCSI_CMD_READ_HEADER            0x44U
#define SCSI_CMD_PLAY_AUDIO_10          0x45U
#define SCSI_CMD_GET_CONFIGURATION      0x46U
#define SCSI_CMD_PLAY_AUDIO_MSF         0x47U
#define SCSI_CMD_PLAY_AUDIO_TI          0x48U
#define SCSI_CMD_PLAY_TRACK_REL_10      0x49U
#define SCSI_CMD_GET_EVENT_STATUS       0x4AU
#define SCSI_CMD_PAUSE_RESUME           0x4BU
#define SCSI_CMD_READ_DISC_INFORMATION  0x51U
#define SCSI_CMD_READ_TRACK_INFORMATION 0x52U
#define SCSI_CMD_RESERVE_TRACK          0x53U
#define SCSI_CMD_SEND_OPC_INFORMATION   0x54U
#define SCSI_CMD_MODE_SELECT_10         0x55U
#define SCSI_CMD_REPAIR_TRACK           0x58U
#define SCSI_CMD_MODE_SENSE_10          0x5AU
#define SCSI_CMD_CLOSE_TRACK_SESSION    0x5BU
#define SCSI_CMD_READ_BUFFER_CAPACITY   0x5CU
#define SCSI_CMD_SEND_CUE_SHEET         0x5DU

// Group 5 Commands (CDB's here are 12-bytes)
#define SCSI_CMD_REPORT_LUNS            0xA0U
#define SCSI_CMD_BLANK                  0xA1U
#define SCSI_CMD_SECURITY_PROTOCOL_IN   0xA2U
#define SCSI_CMD_SEND_KEY               0xA3U
#define SCSI_CMD_REPORT_KEY             0xA4U
#define SCSI_CMD_PLAY_AUDIO_12          0xA5U
#define SCSI_CMD_LOAD_UNLOAD            0xA6U
#define SCSI_CMD_SET_READ_AHEAD         0xA7U
#define SCSI_CMD_READ_12                0xA8U
#define SCSI_CMD_PLAY_TRACK_REL_12      0xA9U
#define SCSI_CMD_WRITE_12               0xAAU
#define SCSI_CMD_READ_MEDIA_SERIAL_12   0xABU
#define SCSI_CMD_GET_PERFORMANCE        0xACU
#define SCSI_CMD_READ_DVD_STRUCTURE     0xADU
#define SCSI_CMD_SECURITY_PROTOCOL_OUT  0xB5U
#define SCSI_CMD_SET_STREAMING          0xB6U
#define SCSI_CMD_READ_MSF               0xB9U
#define SCSI_CMD_SET_SPEED              0xBBU
#define SCSI_CMD_MECHANISM_STATUS       0xBDU
#define SCSI_CMD_READ_CD                0xBEU
#define SCSI_CMD_SEND_DISC_STRUCTURE    0xBFU

// Vendor-unique Commands, included for completeness
#define SCSI_CMD_CD_PLAYBACK_STATUS     0xC4U // SONY unique
#define SCSI_CMD_PLAYBACK_CONTROL       0xC9U // SONY unique
#define SCSI_CMD_READ_CDDA              0xD8U // Vendor unique
#define SCSI_CMD_READ_CDXA              0xDBU // Vendor unique
#define SCSI_CMD_READ_ALL_SUBCODES      0xDFU // Vendor unique

// SCSI error codes
#define SCSI_S_NOT_READY                0x02U
#define SCSI_S_MEDIUM_ERROR             0x03U
#define SCSI_S_ILLEGAL_REQUEST          0x05U
#define SCSI_S_UNIT_ATTENTION           0x06U
#define SCSI_ASC_LBA_OUT_OF_RANGE       0x21U
#define SCSI_ASC_MEDIA_CHANGED          0x28U
#define SCSI_ASC_MEDIUM_NOT_PRESENT     0x3AU

struct SCSI_Capacity {
        uint8_t data[8];
        //uint32_t dwBlockAddress;
        //uint32_t dwBlockLength;
} __attribute__((packed));

struct SCSI_CDB_BASE {
        uint8_t Opcode;

        unsigned unused : 5;
        unsigned LUN : 3;

        uint8_t info[12];
} __attribute__((packed));

typedef SCSI_CDB_BASE SCSI_CDB_BASE_t;

struct SCSI_CDB6 {
        uint8_t Opcode;

        unsigned LBAMSB : 5;
        unsigned LUN : 3;

        uint8_t LBAHB;
        uint8_t LBALB;
        uint8_t AllocationLength;
        uint8_t Control;

public:

        SCSI_CDB6(uint8_t _Opcode, uint8_t _LUN, uint32_t LBA, uint8_t _AllocationLength, uint8_t _Control) :
        Opcode(_Opcode), LBAMSB(UHS_UINT8_BYTE2(LBA) & 0x1F), LUN(_LUN), LBAHB(UHS_UINT8_BYTE1(LBA)), LBALB(UHS_UINT8_BYTE0(LBA)),
        AllocationLength(_AllocationLength), Control(_Control) {
        }

        SCSI_CDB6(uint8_t _Opcode, uint8_t _LUN, uint8_t _AllocationLength, uint8_t _Control) :
        Opcode(_Opcode), LBAMSB(0), LUN(_LUN), LBAHB(0), LBALB(0),
        AllocationLength(_AllocationLength), Control(_Control) {
        }
} __attribute__((packed));

typedef SCSI_CDB6 SCSI_CDB6_t;

struct SCSI_CDB10 {
        uint8_t Opcode;

        unsigned Service_Action : 5;
        unsigned LUN : 3;

        uint8_t LBA_L_M_MB;
        uint8_t LBA_L_M_LB;
        uint8_t LBA_L_L_MB;
        uint8_t LBA_L_L_LB;

        uint8_t Misc2;

        uint8_t ALC_MB;
        uint8_t ALC_LB;

        uint8_t Control;
public:

        SCSI_CDB10(uint8_t _Opcode, uint8_t _LUN) :
        Opcode(_Opcode), Service_Action(0), LUN(_LUN),
        LBA_L_M_MB(0), LBA_L_M_LB(0), LBA_L_L_MB(0), LBA_L_L_LB(0),
        Misc2(0), ALC_MB(0), ALC_LB(0), Control(0) {
        }

        SCSI_CDB10(uint8_t _Opcode, uint8_t _LUN, uint16_t xflen, uint32_t _LBA) :
        Opcode(_Opcode), Service_Action(0), LUN(_LUN),
        LBA_L_M_MB(UHS_UINT8_BYTE3(_LBA)), LBA_L_M_LB(UHS_UINT8_BYTE2(_LBA)), LBA_L_L_MB(UHS_UINT8_BYTE1(_LBA)), LBA_L_L_LB(UHS_UINT8_BYTE0(_LBA)),
        Misc2(0), ALC_MB(UHS_UINT8_BYTE1(xflen)), ALC_LB(UHS_UINT8_BYTE0(xflen)), Control(0) {
        }
} __attribute__((packed));

typedef SCSI_CDB10 SCSI_CDB10_t;

struct SCSI_CDB12 {
        uint8_t Opcode;

        unsigned Service_Action : 5;
        unsigned Misc : 3;

        uint8_t LBA_L_M_LB;
        uint8_t LBA_L_L_MB;
        uint8_t LBA_L_L_LB;

        uint8_t ALC_M_LB;
        uint8_t ALC_L_MB;
        uint8_t ALC_L_LB;
        uint8_t Control;
} __attribute__((packed));

typedef SCSI_CDB12 SCSI_CDB12_t;

struct SCSI_CDB_LBA32_16 {
        uint8_t Opcode;

        unsigned Service_Action : 5;
        unsigned Misc : 3;

        uint8_t LBA_L_M_MB;
        uint8_t LBA_L_M_LB;
        uint8_t LBA_L_L_MB;
        uint8_t LBA_L_L_LB;

        uint8_t A_M_M_MB;
        uint8_t A_M_M_LB;
        uint8_t A_M_L_MB;
        uint8_t A_M_L_LB;

        uint8_t ALC_M_MB;
        uint8_t ALC_M_LB;
        uint8_t ALC_L_MB;
        uint8_t ALC_L_LB;

        uint8_t Misc2;
        uint8_t Control;
} __attribute__((packed));

struct SCSI_CDB_LBA64_16 {
        uint8_t Opcode;
        uint8_t Misc;

        uint8_t LBA_M_M_MB;
        uint8_t LBA_M_M_LB;
        uint8_t LBA_M_L_MB;
        uint8_t LBA_M_L_LB;

        uint8_t LBA_L_M_MB;
        uint8_t LBA_L_M_LB;
        uint8_t LBA_L_L_MB;
        uint8_t LBA_L_L_LB;

        uint8_t ALC_M_MB;
        uint8_t ALC_M_LB;
        uint8_t ALC_L_MB;
        uint8_t ALC_L_LB;

        uint8_t Misc2;
        uint8_t Control;
} __attribute__((packed));

struct SCSI_Inquiry_Response {
        uint8_t DeviceType : 5;
        uint8_t PeripheralQualifier : 3;

        unsigned Reserved : 7;
        unsigned Removable : 1;

        uint8_t Version;

        unsigned ResponseDataFormat : 4;
        unsigned HISUP : 1;
        unsigned NormACA : 1;
        unsigned TrmTsk : 1;
        unsigned AERC : 1;

        uint8_t AdditionalLength;

        unsigned PROTECT : 1;
        unsigned Res : 2;
        unsigned ThreePC : 1;
        unsigned TPGS : 2;
        unsigned ACC : 1;
        unsigned SCCS : 1;

        unsigned ADDR16 : 1;
        unsigned R1 : 1;
        unsigned R2 : 1;
        unsigned MCHNGR : 1;
        unsigned MULTIP : 1;
        unsigned VS : 1;
        unsigned ENCSERV : 1;
        unsigned BQUE : 1;

        unsigned SoftReset : 1;
        unsigned CmdQue : 1;
        unsigned Reserved4 : 1;
        unsigned Linked : 1;
        unsigned Sync : 1;
        unsigned WideBus16Bit : 1;
        unsigned WideBus32Bit : 1;
        unsigned RelAddr : 1;

        uint8_t VendorID[8];
        uint8_t ProductID[16];
        uint8_t RevisionID[4];
} __attribute__((packed));

struct SCSI_Request_Sense_Response {
        uint8_t bResponseCode;
        uint8_t bSegmentNumber;

        uint8_t bmSenseKey : 4;
        uint8_t bmReserved : 1;
        uint8_t bmILI : 1;
        uint8_t bmEOM : 1;
        uint8_t bmFileMark : 1;

        uint8_t Information[4];
        uint8_t bAdditionalLength;
        uint8_t CmdSpecificInformation[4];
        uint8_t bAdditionalSenseCode;
        uint8_t bAdditionalSenseQualifier;
        uint8_t bFieldReplaceableUnitCode;
        uint8_t SenseKeySpecific[3];
} __attribute__((packed));

#endif  // UHS_SCSI_H
