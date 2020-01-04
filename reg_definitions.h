#include <Arduino.h>
#ifndef __REG_DEFINITIONS__
#define __REG_DEFINITIONS__

// #define DAP_TRANSFER_AP = 1 << 0,
// #define DAP_TRANSFER_READ = 1 << 1,

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

// SAMDx5 specific


#define DHCSR 0xe000edf0      /// Debug Halting Control and Status Register
// #define DCRSR 0xE000EDF4      /// Debug Core Register Selector Register
#define DEMCR 0xe000edfc      /// Debug Exception and Monitor Control Register
// #define DCRDR 0xE000EDF8      /// Debug Core Register Data Register
#define AIRCR 0xe000ed0c      /// Application Interrupt and Reset Control Register

#define DAP_DSU_CTRL_STATUS 0x41002100
#define DAP_DSU_DID 0x41002018 // 0x41002118
#define DAP_DSU_ADDR 0x41002104
#define DAP_DSU_DATA 0x4100210C
#define DAP_DSU_LENGTH 0x41002108


#endif // __REG_DEFINITIONS__
