
#include "line_write.h"
#include "reg_definitions.h"

#define SWDIO 9
#define SWCLK 8
#define SWRST 10

static uint32_t calc_parity(uint32_t value) {
  value ^= value >> 16;
  value ^= value >> 8;
  value ^= value >> 4;
  value &= 0x0f;
  return (0x6996 >> value) & 1;
}

static inline uint32_t build_request(uint8_t addr, bool is_ap_reg, bool is_read) {
  uint8_t request = 0; ;
  if (is_ap_reg) {
    request |= 1 << 1;            // set APnDP bit
  }
  if (is_read) {
    request |= 1 << 2;              // set RnW bit
  }
  request |= (addr & 0x0C) << 1;  // set ADDR[2:3] bits
  request |= calc_parity(request) << 5; //set parity
  request |= (1 << 7) | (1);      // set start & stop bits
  return request;
}

bool read_reg(uint8_t addr, bool is_ap_reg, uint32_t * res) {
  int attempts_to_read_reg = 0;
  retry_read_reg:
  uint8_t request = build_request(addr, is_ap_reg, true);
  *res = 0;
  
  write_line(request, 8);  // Write the request
  turn_around_to_input();

  uint8_t ack = read_ack(); // read the ACK
  if (ack != 1) {
    turn_around_to_output();
    
    if (ack == 2) { // wait
      delayMicroseconds(10);
      if (attempts_to_read_reg < 10) {
        Serial.println("ACK WAIT: retrying");
        attempts_to_read_reg ++;
        goto retry_read_reg;
      }
    } else {
      Serial.print("ACK ERR in reg read ");
      Serial.print(addr, HEX);
      Serial.print(" AP: "); Serial.print(is_ap_reg ? "Y" : "N");
      Serial.print(" ack: ");
      Serial.println(ack);
    }
    return false;
  }
  
  *res = read_line(32);
  
  uint8_t parity_bit = read_line(1); // read parity
  turn_around_to_output();

  Serial.println("ACK OK");
  if (parity_bit != calc_parity(*res)) {
    Serial.println("Parity mismatch");
  }
  Serial.println(*res, HEX);
  return true;
}

bool write_reg(uint8_t addr, bool is_ap_reg, uint32_t val) {
  uint8_t request = build_request(addr, is_ap_reg, false);
  
  write_line(request, 8);  // Write the request
  turn_around_to_input();

  uint8_t ack = read_ack(); // read the ACK
  if (ack != 1) {
    Serial.print("ACK ERR in reg write ");
    Serial.print(addr, HEX);
    Serial.print(" AP: "); Serial.print(is_ap_reg ? "Y" : "N");
    Serial.print(" val: "); Serial.print(val, HEX);
    Serial.print(" ack: ");
    Serial.println(ack);
    return false;
  }
  
  turn_around_to_output();
  uint8_t parity_bit = calc_parity(val);
  write_line(val, 32);
  write_line(parity_bit, 1);
  
  Serial.println("ACK OK"); 
  return ack == 1;
}

uint32_t read_id_code() {
  Serial.println("ID Code");
  uint32_t id_code = 0;
  read_reg(SWD_DP_REG_IDCODE, false, &id_code);
  return id_code;
}

uint32_t read_ctrl_stat() {
  Serial.println("CTRL/STAT");
  uint32_t ctrl_stat = 0;
  read_reg(SWD_DP_REG_CTRL_STAT, false, &ctrl_stat);
  return ctrl_stat;
}

uint32_t read_read_buf() {
  Serial.println("READ BUF");
  uint32_t read_buf = 0;
  if (!read_reg(SWD_DP_REG_RDBUFF, false, &read_buf)) {
    Serial.println(read_ctrl_stat());
    while(1);
  }
  return read_buf;
}

bool write_ctrl_stat(uint32_t val) {
  Serial.println("Write CTRL STAT");
  return write_reg(SWD_DP_REG_CTRL_STAT, false, val);
}

bool write_abort(uint32_t val) {
  Serial.println("W ABORT");
  return write_reg(SWD_DP_REG_ABORT, false, val);
}

bool write_select(uint32_t val) {
  Serial.println("W SELECT");
  return write_reg(SWD_DP_REG_SELECT, false, val);
}

///

bool write_csw(uint32_t val) {
  Serial.println("W AP CSW");
  return write_reg(SWD_AP_REG_CSW, true, 0x23000012);
}

//

uint32_t read_word(uint32_t addr) {
  uint32_t res = 0;
  write_reg(SWD_AP_REG_TAR, true, addr);
  read_reg(SWD_AP_REG_DRW, true, &res);
  return res;
}

bool write_word(uint32_t addr, uint32_t data) {
  return write_reg(SWD_AP_REG_TAR, true, addr) && write_reg(SWD_AP_REG_DRW, true, data);
}

///

void stop_core() {
  Serial.println("Stop core");
  
  write_word(DHCSR, 0xa05f0003);
  read_read_buf();
  write_word(DEMCR, 0x00100501); // ICE writes 1000001 to DEMCR?
  read_read_buf();
  write_word(AIRCR, 0x05fa0004);
  Serial.println("Core stopped\r\n");
}

uint32_t read_did() {
  Serial.println("Read DSU DID");
  if (read_word(DAP_DSU_DID) == 0) {
    uint32_t did = read_read_buf();
    Serial.print("DID: 0x"); Serial.println(did, HEX);
    return did;
  } else {
    return 0;
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    delay(1);         // will pause the chip until it opens serial console
  }
/*
  uint32_t tmp = build_request(0x00, false, true);
  Serial.println(tmp, HEX); 
  while (1);*/
  
  // Prepare registers we will be using, allowing us to switch pins quickly, instead of using the slow Arduino functions
  // Reset the target, prep clk/data pins
  prepare_pin_registers_and_reset_target(SWDIO, SWCLK, SWRST);

  // switch to quicker clocks
  //set_clock_delay_us(4);
  line_rst_switch_to_swd();
  //set_clock_delay_us(1);
  delayMicroseconds(500);

  // let's get busy
  read_id_code();
  uint32_t ctrl_stat = read_ctrl_stat();
  read_read_buf();

  write_select(0x00);
  read_read_buf();  // should return 0

  // power-up and debug power-up request
  write_ctrl_stat((1 << 30) | (1 << 28));// CSYSPWRUPREQ | CDBGPRWUPREQ
  read_read_buf();  // should return 0
  ctrl_stat = read_ctrl_stat();
  for (int i = 0; i < 100; i ++){
    if ((ctrl_stat & (1 << 31)) && (ctrl_stat & (1 << 29)) && (ctrl_stat & (1 << 6))) { // check READOK
      Serial.println("Power-up requests complete");
      break;
    } else {
      delay(1);
    }
  }

  // abort?
  write_abort(0x0000001E);
  read_ctrl_stat();
  
  // Debug start request
  // Technically what we should do is: write bit 26 (CDBGRSTREQ) of the CTRL/STAT, wait until bit 27 (CDBGRSTACK) is set and then clear bit 26. 
  // However it seems the Atmel ICE gives up after few ms

  for (int i = 0; i < 20; i ++){
    write_ctrl_stat((1 << 30) | (1 << 28) | (1 << 26));// CSYSPWRUPREQ | CDBGPRWUPREQ | CDBGRSTREQ
    read_read_buf();  // should return 0
    ctrl_stat = read_ctrl_stat();
    if ((ctrl_stat & (1 << 27))) {
      Serial.println("Debugger reset request complete");
      write_ctrl_stat((1 << 30) | (1 << 28) | (15 << 8));// CSYSPWRUPREQ | CDBGPRWUPREQ | MASKLANE=15
      break;
    } else {
      delayMicroseconds(100);
    }
  }
  if (!(ctrl_stat & (1 << 27))) {
    Serial.println("Debugger reset request doesn't seem to work...");
  }
  
  write_csw(0x23000052);
  read_read_buf();

  stop_core();
  
  read_did(); // should be 0x60060004 for SAMD51J20A
  
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
}
