/*
 * Copyright (c) 2020 Dimitar Kunchev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <Arduino.h>
#ifndef __REG_DEFINITIONS__
#define __REG_DEFINITIONS__

enum {
  SWD_ACK_OK   = 1<<0,
  SWD_ACK_WAIT = 1<<1,
  SWD_ACK_ERR  = 1<<2
};

enum {
  SWD_DP_REG_IDCODE = 0x00,
  SWD_DP_REG_ABORT = 0x00,
  SWD_DP_REG_CTRL_STAT = 0x04, // When CTRLSEL == 0
  SWD_DP_REG_WCR = 0x04,       // When CTRLSEL == 1
  SWD_DP_REG_RESEND = 0x08,
  SWD_DP_REG_SELECT = 0x08,
  SWD_DP_REG_RDBUFF = 0x0c,
};

enum {
  SWD_AP_REG_CSW = 0x00, // | DAP_TRANSFER_AP,
  SWD_AP_REG_TAR = 0x04, // | DAP_TRANSFER_AP,
  SWD_AP_REG_DRW = 0x0c, // | DAP_TRANSFER_AP,

  SWD_AP_REG_DB0 = 0x00, // | DAP_TRANSFER_AP, // 0x10
  SWD_AP_REG_DB1 = 0x04, // | DAP_TRANSFER_AP, // 0x14
  SWD_AP_REG_DB2 = 0x08, // | DAP_TRANSFER_AP, // 0x18
  SWD_AP_REG_DB3 = 0x0c, // | DAP_TRANSFER_AP, // 0x1c

  SWD_AP_REG_CFG = 0x04, // | DAP_TRANSFER_AP,  // 0xf4
  SWD_AP_REG_BASE = 0x08, // | DAP_TRANSFER_AP, // 0xf8
  SWD_AP_REG_IDR = 0x0c, // | DAP_TRANSFER_AP,  // 0xfc
};


/// CTRL/STAT DP Register

// The following structure is mostly for reference, one would rather just use uint32_t really
typedef union {
  struct {
    uint32_t  CSYSPWRUPACK:1;           // System powerup acknowledge. Indicates the status of the CSYSPWRUPACK signal
    uint32_t  CSYSPWRUPREQ:1;           // System powerup request. This bit controls the CSYSPWRUPREQ signal
    uint32_t  CDBGPWRUPACK:1;           // Debug powerup acknowledge. Indicates the status of the CDBGPWRUPACK signal.
    uint32_t  CDBGPWRUPREQ:1;           // Debug powerup request. This bit controls the CDBGPRWUPREQ signal
    uint32_t  CDBGRSTACK:1;             // Debug reset acknowledge. Indicates the status of the CDBGRSTACK signal. 
    uint32_t  CDBGRSTREQ:1;             // Debug reset request. This bit controls the CDBGRSTREQ signal. 
    uint32_t  _reserved:2;              // reserved
    uint32_t  TRNCNT:12;                // Transaction counter
    uint32_t  MASKLANE:4;               // Indicates the bytes to be masked in pushed-compare and pushed-verify operations
    uint32_t  WDATAERR:1;               // For SWD: This bit is set to 1 if a Write Data Error occurs (parity/framing error or write was accepted by DP but was discarded before reaching AP)
    uint32_t  READOK:1;                 // The bit is set to 1 if the response to the previous AP read or RDBUFF read was OK.
    uint32_t  STICKYERR:1;              // This bit is set to 1 if an error is returned by an AP transaction. 
    uint32_t  STICKYCMP:1;              // This bit is set to 1 when a match occurs on a pushed-compare or a pushed-verify operation.
    uint32_t  TRNMODE:2;                // This field sets the transfer mode for AP operations
    uint32_t  STICKYORUN:1;             // If overrun detection is enabled, this bit is set to 1 when an overrun occurs (if enabled via ORUNDETECT)
    uint32_t  ORUNDETECT:1;             // This bit is set to 1 to enable overrun detection.
  };
  uint32_t reg;
} SWD_REG_CTRL_STAT_t;

#define SWD_REG_CTRL_STAT_CSYSPWRUPACK  (0x1ul << 31)
#define SWD_REG_CTRL_STAT_CSYSPWRUPREQ  (0x1ul << 30)
#define SWD_REG_CTRL_STAT_CDBGPWRUPACK  (0x1ul << 29)
#define SWD_REG_CTRL_STAT_CDBGPWRUPREQ  (0x1ul << 28)
#define SWD_REG_CTRL_STAT_CDBGRSTACK    (0x1ul << 27)
#define SWD_REG_CTRL_STAT_CDBGRSTREQ    (0x1ul << 26)
#define SWD_REG_CTRL_STAT_TRNCNT_Pos    (12)
#define SWD_REG_CTRL_STAT_TRNCNT_Mask   (0xFFF << SWD_REG_CTRL_STAT_TRNCNT_Pos)
#define SWD_REG_CTRL_STAT_TRNCNT(val)   (SWD_REG_CTRL_STAT_TRNCNT_Mask & (val << SWD_REG_CTRL_STAT_TRNCNT_Pos))

#define SWD_REG_CTRL_STAT_MASKLANE_Pos  8
#define SWD_REG_CTRL_STAT_MASKLANE_Mask (0x0F)
#define SWD_REG_CTRL_STAT_MASKLANE(val) (SWD_REG_CTRL_STAT_MASKLANE_Mask & (val << SWD_REG_CTRL_STAT_MASKLANE_Pos))

#define SWD_REG_CTRL_STAT_WDATAERR      (0x1ul << 7)
#define SWD_REG_CTRL_STAT_READOK        (0x1ul << 6)
#define SWD_REG_CTRL_STAT_STICKYERR     (0x1ul << 5)
#define SWD_REG_CTRL_STAT_STICKYCMP     (0x1ul << 4)

#define SWD_REG_CTRL_STAT_TRNMODE_Pos   2
#define SWD_REG_CTRL_STAT_TRNMODE_Mask  (0x03 << SWD_REG_CTRL_STAT_TRNMODE_Pos)
#define SWD_REG_CTRL_STAT_TRNMODE(val)  (SWD_REG_CTRL_STAT_TRNMODE_Mask & (val << SWD_REG_CTRL_STAT_TRNMODE_Pos))
#define SWD_REG_CTRL_STAT_TRNMODE_NORMAL          SWD_REG_CTRL_STAT_TRNMODE(0)
#define SWD_REG_CTRL_STAT_TRNMODE_PUSHED_VERIFY   SWD_REG_CTRL_STAT_TRNMODE(1)
#define SWD_REG_CTRL_STAT_TRNMODE_PUSHED_COMPARE  SWD_REG_CTRL_STAT_TRNMODE(2)

#define SWD_REG_CTRL_STAT_STICKYORUN    (0x1ul << 1)
#define SWD_REG_CTRL_STAT_ORUNDETECT    (0x1ul << 0)

// ABORT DP/AP REGISTER
typedef union {
  struct {
    uint32_t  _reserved:27;         // reserved
    uint32_t  ORUNERRCLR:1;         // Write 1 to this bit to clear the CTRL/STAT.STICKYORUN overrun error bit to 0.
    uint32_t  WDERRCLR:1;           // Write 1 to this bit to clear the CTRL/STAT.WDATAERR write data error bit to 0.
    uint32_t  STKERRCLR:1;          // Write 1 to this bit to clear the CTRL/STAT.STICKYERR sticky error bit to 0.
    uint32_t  STKCMPCLR:1;          // Write 1 to this bit to clear the CTRL/STAT.STICKYCMP sticky compare bit to 0.
    uint32_t  DAPABORT:1;           // Write 1 to this bit to generate a DAP abort. This aborts the current AP transaction.
  };
  uint32_t reg;
} SWD_REG_ABORT_t;

#define SWD_REG_ABORT_ORUNERRCLR  (0x1ul << 4)
#define SWD_REG_ABORT_WDERRCLR    (0x1ul << 3)
#define SWD_REG_ABORT_STKERRCLR   (0x1ul << 2)
#define SWD_REG_ABORT_STKCMPCLR   (0x1ul << 1)
#define SWD_REG_ABORT_DAPABORT    (0x1ul << 0)

#define SWD_REG_ABORT_Clear_All   (SWD_REG_ABORT_ORUNERRCLR|SWD_REG_ABORT_WDERRCLR|SWD_REG_ABORT_STKERRCLR|SWD_REG_ABORT_STKCMPCLR)

////////////

// MEM-AP CSW Register
// I am sorry these use camel case, but I prefer to maintain them as per the original ARM documentation
typedef union {
  struct {
    uint32_t DbgSwEnable:1;     // Debug software access enable. DbgSwEnable must be ignored and treated as one if DeviceEn is set to 0.
    uint32_t Prot:7;            // Prot and Type: Bus access protection control. A debugger can use these fields to specify flags for a debug access.
    uint32_t SPIDEN:1;          // (readonly) Secure Privileged Debug Enabled.
    uint32_t _reserved:7;       // reserved
    uint32_t Type:4;            // See Prot
    uint32_t Mode:4;            // Mode of operation. Basic or with Barrier Protection enabled
    uint32_t TrInProg:1;        // (readonly) Transfer in progress. This bit is set to 1 while a transfer is in progress (0 when idle).
    uint32_t DeviceEn:1;        // (readonly) Device enabled. This bit is set to 1 when transactions can be issued through the MEM-AP.
    uint32_t AddrInc:2;         // Address auto-increment and packing mode for the TAR register.  This field controls whether the access address (in TAR) increments automatically on read and write data accesses through the DRW register
    uint32_t _reserved2:1;      // reserved
    uint32_t Size:3;            // Size of access. This field indicates the size of access to perform.
  };
  uint32_t reg;
} SWD_REG_MEM_AP_CSW_t;
#define SWD_REG_MEM_AP_CSW_DbgSwEnable  (0x1ul << 31)
#define SWD_REG_MEM_AP_CSW_Prot_Pos     (24)
#define SWD_REG_MEM_AP_CSW_Prot_Mask    (0x7F << SWD_REG_MEM_AP_CSW_Prot_Pos)
#define SWD_REG_MEM_AP_CSW_Prot(val)    (SWD_REG_MEM_AP_CSW_Prot_Mask & (val << SWD_REG_MEM_AP_CSW_Prot_Pos))

#define SWD_REG_MEM_AP_CSW_SPIDEN       (0x1ul << 23)

#define SWD_REG_MEM_AP_CSW_Type_Pos     12
#define SWD_REG_MEM_AP_CSW_Type_Mask    (0x07 << SWD_REG_MEM_AP_CSW_Type_Pos)
#define SWD_REG_MEM_AP_CSW_Type(val)    (SWD_REG_MEM_AP_CSW_Type_Mask & (val << SWD_REG_MEM_AP_CSW_Type_Pos))

#define SWD_REG_MEM_AP_CSW_Mode_Pos     8
#define SWD_REG_MEM_AP_CSW_Mode_Mask    (0x0F << SWD_REG_MEM_AP_CSW_Mode_Pos)
#define SWD_REG_MEM_AP_CSW_Mode(val)    (SWD_REG_MEM_AP_CSW_Mode_Mask & (val << SWD_REG_MEM_AP_CSW_Mode_Pos))

#define SWD_REG_MEM_AP_CSW_TrInProg     (0x01ul << 7)
#define SWD_REG_MEM_AP_CSW_DeviceEn     (0x01ul << 6)
#define SWD_REG_MEM_AP_CSW_AddrInc_Pos  4
#define SWD_REG_MEM_AP_CSW_AddrInc_Mask (0x03 << SWD_REG_MEM_AP_CSW_AddrInc_Pos)
#define SWD_REG_MEM_AP_CSW_AddrInc(val) (SWD_REG_MEM_AP_CSW_AddrInc_Mask & (val << SWD_REG_MEM_AP_CSW_AddrInc_Pos))
#define SWD_REG_MEM_AP_CSW_AddrInc_Off        SWD_REG_MEM_AP_CSW_AddrInc(0b00)  // No increment after reading/writing DWR
#define SWD_REG_MEM_AP_CSW_AddrInc_Single     SWD_REG_MEM_AP_CSW_AddrInc(0b01)  // After a successful Data Read/Write Register access, the address in the TAR is incremented by the size of the access
#define SWD_REG_MEM_AP_CSW_AddrInc_Packed     SWD_REG_MEM_AP_CSW_AddrInc(0b10)  // Enables packed transfer operations

#define SWD_REG_MEM_AP_CSW_Size_Pos     (0)
#define SWD_REG_MEM_AP_CSW_Size_Mask    (0x07 << SWD_REG_MEM_AP_CSW_Size_Pos)
#define SWD_REG_MEM_AP_CSW_Size(val)    (SWD_REG_MEM_AP_CSW_Size_Mask & (val << SWD_REG_MEM_AP_CSW_Size_Pos))
#define SWD_REG_MEM_AP_CSW_Size_Byte        SWD_REG_MEM_AP_CSW_Size(0b000)
#define SWD_REG_MEM_AP_CSW_Size_HalfWord    SWD_REG_MEM_AP_CSW_Size(0b001)
#define SWD_REG_MEM_AP_CSW_Size_Word        SWD_REG_MEM_AP_CSW_Size(0b010)
#define SWD_REG_MEM_AP_CSW_Size_DoubleWord  SWD_REG_MEM_AP_CSW_Size(0b011)
#define SWD_REG_MEM_AP_CSW_Size_128         SWD_REG_MEM_AP_CSW_Size(0b100)
#define SWD_REG_MEM_AP_CSW_Size_256         SWD_REG_MEM_AP_CSW_Size(0b101)

// SAMDx5 specific

#define DHCSR 0xe000edf0      /// Debug Halting Control and Status Register
// #define DCRSR 0xE000EDF4      /// Debug Core Register Selector Register
#define DEMCR 0xe000edfc      /// Debug Exception and Monitor Control Register
// #define DCRDR 0xE000EDF8      /// Debug Core Register Data Register
#define AIRCR 0xe000ed0c      /// Application Interrupt and Reset Control Register

// As per chapter 12.9 of the SAMD51 datasheet: DSU registers are mirrored at offset 0x100 from the base 0x41002000 address for security reasons. External probes have to access it there
#define DAP_DSU_BASE        0x41002100
#define DAP_DSU_CTRL_STATUS (DAP_DSU_BASE + 0)
#define DAP_DSU_ADDR        (DAP_DSU_BASE + 0x04)
#define DAP_DSU_LENGTH      (DAP_DSU_BASE + 0x08)
#define DAP_DSU_DATA        (DAP_DSU_BASE + 0x0C)

#define DAP_DSU_DID         (DAP_DSU_BASE + 0x18)

//

#define USER_ROW_ADDR 0x00804000
#define USER_ROW_SIZE 32
#define FLASH_START_ADDR  0x00

#define DAP_NVMCTRL_BASE      (0x41004000)
#define DAP_NVMCTRL_CTRLA     (DAP_NVMCTRL_BASE + 0x00)
#define DAP_NVMCTRL_CTRLB     (DAP_NVMCTRL_BASE + 0x04)
#define DAP_NVMCTRL_INTFLAG   (DAP_NVMCTRL_BASE + 0x10)
///#define DAP_NVMCTRL_STATUS    (DAP_NVMCTRL_BASE + 0x12) // use the INTFLAG instead and offset by 16 bits. That is because we use 32-bit read/writes
#define DAP_NVMCTRL_ADDR      (DAP_NVMCTRL_BASE + 0x14)

#define DAP_NVMCTRL_CTRLA_WMODE_Pos 4
#define DAP_NVMCTRL_CTRLA_WMODE_Msk (0x3ul << 4)
#define DAP_NVMCTRL_CTRLA_WMODE(val)  (DAP_NVMCTRL_CTRLA_WMODE_Msk & ((value) << DAP_NVMCTRL_CTRLA_WMODE_Pos))
#define   DAP_NVMCTRL_CTRLA_WMODE_MAN_Val     (0x0ul)   /**< \brief (NVMCTRL_CTRLA) Manual Write */
#define   DAP_NVMCTRL_CTRLA_WMODE_ADW_Val     (0x1ul)   /**< \brief (NVMCTRL_CTRLA) Automatic Double Word Write */
#define   DAP_NVMCTRL_CTRLA_WMODE_AQW_Val     (0x2ul)   /**< \brief (NVMCTRL_CTRLA) Automatic Quad Word */
#define   DAP_NVMCTRL_CTRLA_WMODE_AP_Val      (0x3ul)   /**< \brief (NVMCTRL_CTRLA) Automatic Page Write */
#define DAP_NVMCTRL_CTRLA_WMODE_MAN     (DAP_NVMCTRL_CTRLA_WMODE_MAN_Val   << DAP_NVMCTRL_CTRLA_WMODE_Pos)
#define DAP_NVMCTRL_CTRLA_WMODE_ADW     (DAP_NVMCTRL_CTRLA_WMODE_ADW_Val   << DAP_NVMCTRL_CTRLA_WMODE_Pos)
#define DAP_NVMCTRL_CTRLA_WMODE_AQW     (DAP_NVMCTRL_CTRLA_WMODE_AQW_Val   << DAP_NVMCTRL_CTRLA_WMODE_Pos)
#define DAP_NVMCTRL_CTRLA_WMODE_AP      (DAP_NVMCTRL_CTRLA_WMODE_AP_Val    << DAP_NVMCTRL_CTRLA_WMODE_Pos)

#define DAP_NVMCTRL_CTRLA_AUTOWS    (1<<2)
#define DAP_NVMCTRL_CTRLB_CMDEX_Pos        8
#define DAP_NVMCTRL_CTRLB_CMDEX_KEY_Val    (0xA5ul)
#define DAP_NVMCTRL_CTRLB_CMDEX_KEY        (DAP_NVMCTRL_CTRLB_CMDEX_KEY_Val << DAP_NVMCTRL_CTRLB_CMDEX_Pos)
#define DAP_NVMCTRL_CTRLB_CMD_Pos       0
#define DAP_NVMCTRL_CTRLB_CMD_Msk       (0x7Ful << DAPNVMCTRL_CTRLB_CMD_Pos)
#define DAP_NVMCTRL_CTRLB_CMD(val)      (DAP_NVMCTRL_CTRLB_CMD_Msk & ((val) << DAP_NVMCTRL_CTRLB_CMD_Pos))
#define   DAP_NVMCTRL_CTRLB_CMD_EP_Val        0x0   /**< \brief (NVMCTRL_CTRLB) Erase Page - Only supported in the USER and AUX pages. */
#define   DAP_NVMCTRL_CTRLB_CMD_EB_Val        0x1   /**< \brief (NVMCTRL_CTRLB) Erase Block - Erases the block addressed by the ADDR register, not supported in the user page */
#define   DAP_NVMCTRL_CTRLB_CMD_WP_Val        0x3   /**< \brief (NVMCTRL_CTRLB) Write Page - Writes the contents of the page buffer to the page addressed by the ADDR register, not supported in the user page */
#define   DAP_NVMCTRL_CTRLB_CMD_WQW_Val       0x4   /**< \brief (NVMCTRL_CTRLB) Write Quad Word - Writes a 128-bit word at the location addressed by the ADDR register. */
#define   DAP_NVMCTRL_CTRLB_CMD_SWRST_Val     0x10   /**< \brief (NVMCTRL_CTRLB) Software Reset - Power-Cycle the NVM memory and replay the device automatic calibration procedure and resets the module configuration registers */
#define   DAP_NVMCTRL_CTRLB_CMD_LR_Val        0x11   /**< \brief (NVMCTRL_CTRLB) Lock Region - Locks the region containing the address location in the ADDR register. */
#define   DAP_NVMCTRL_CTRLB_CMD_UR_Val        0x12   /**< \brief (NVMCTRL_CTRLB) Unlock Region - Unlocks the region containing the address location in the ADDR register. */
#define   DAP_NVMCTRL_CTRLB_CMD_SPRM_Val      0x13   /**< \brief (NVMCTRL_CTRLB) Sets the power reduction mode. */
#define   DAP_NVMCTRL_CTRLB_CMD_CPRM_Val      0x14   /**< \brief (NVMCTRL_CTRLB) Clears the power reduction mode. */
#define   DAP_NVMCTRL_CTRLB_CMD_PBC_Val       0x15   /**< \brief (NVMCTRL_CTRLB) Page Buffer Clear - Clears the page buffer. */
#define   DAP_NVMCTRL_CTRLB_CMD_SSB_Val       0x16   /**< \brief (NVMCTRL_CTRLB) Set Security Bit */
#define   DAP_NVMCTRL_CTRLB_CMD_BKSWRST_Val   0x17   /**< \brief (NVMCTRL_CTRLB) Bank swap and system reset, if SMEE is used also reallocate SMEE data into the opposite BANK */
#define   DAP_NVMCTRL_CTRLB_CMD_CELCK_Val     0x18   /**< \brief (NVMCTRL_CTRLB) Chip Erase Lock - DSU.CE command is not available */
#define   DAP_NVMCTRL_CTRLB_CMD_CEULCK_Val    0x19   /**< \brief (NVMCTRL_CTRLB) Chip Erase Unlock - DSU.CE command is available */
#define   DAP_NVMCTRL_CTRLB_CMD_SBPDIS_Val    0x1A   /**< \brief (NVMCTRL_CTRLB) Sets STATUS.BPDIS, Boot loader protection is discarded until CBPDIS is issued or next start-up sequence */
#define   DAP_NVMCTRL_CTRLB_CMD_CBPDIS_Val    0x1B   /**< \brief (NVMCTRL_CTRLB) Clears STATUS.BPDIS, Boot loader protection is not discarded */
#define   DAP_NVMCTRL_CTRLB_CMD_ASEES0_Val    0x30   /**< \brief (NVMCTRL_CTRLB) Activate SmartEEPROM Sector 0, deactivate Sector 1 */
#define   DAP_NVMCTRL_CTRLB_CMD_ASEES1_Val    0x31   /**< \brief (NVMCTRL_CTRLB) Activate SmartEEPROM Sector 1, deactivate Sector 0 */
#define   DAP_NVMCTRL_CTRLB_CMD_SEERALOC_Val  0x32   /**< \brief (NVMCTRL_CTRLB) Starts SmartEEPROM sector reallocation algorithm */
#define   DAP_NVMCTRL_CTRLB_CMD_SEEFLUSH_Val  0x33   /**< \brief (NVMCTRL_CTRLB) Flush SMEE data when in buffered mode */
#define   DAP_NVMCTRL_CTRLB_CMD_LSEE_Val      0x34   /**< \brief (NVMCTRL_CTRLB) Lock access to SmartEEPROM data from any mean */
#define   DAP_NVMCTRL_CTRLB_CMD_USEE_Val      0x35   /**< \brief (NVMCTRL_CTRLB) Unlock access to SmartEEPROM data */
#define   DAP_NVMCTRL_CTRLB_CMD_LSEER_Val     0x36   /**< \brief (NVMCTRL_CTRLB) Lock access to the SmartEEPROM Register Address Space (above 64KB) */
#define   DAP_NVMCTRL_CTRLB_CMD_USEER_Val     0x37   /**< \brief (NVMCTRL_CTRLB) Unlock access to the SmartEEPROM Register Address Space (above 64KB) */
#define DAP_NVMCTRL_CTRLB_CMDEX_EP      (DAP_NVMCTRL_CTRLB_CMD_EP_Val      << DAP_NVMCTRL_CTRLB_CMD_Pos)
#define DAP_NVMCTRL_CTRLB_CMD_WP        (DAP_NVMCTRL_CTRLB_CMD_WP_Val      << DAP_NVMCTRL_CTRLB_CMD_Pos)
#define DAP_NVMCTRL_CTRLB_CMD_WQW       (DAP_NVMCTRL_CTRLB_CMD_WQW_Val     << DAP_NVMCTRL_CTRLB_CMD_Pos)
#define DAP_NVMCTRL_CTRLB_CMD_PBC       (DAP_NVMCTRL_CTRLB_CMD_PBC_Val     << DAP_NVMCTRL_CTRLB_CMD_Pos)

#endif // __REG_DEFINITIONS__
