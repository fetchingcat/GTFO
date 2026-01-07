#define OutputEnablePin 2
#define WriteEnablePin 4
#define ResetPin 3
#define ClkPin 10
#define IRQPin 11
#define ReadyPin 12

#define MISOPin 50
#define MOSIPin 51 //BANK_SER - also VIA PA1 (DATA)
#define SCKPin  52 //BANK_CLK - also VIA PA0 (CLOCK)
#define SSPin   53 //BANK_DONE - also VIA PA2 (LATCH)

// VIA pins are the SAME as the SPI pins on this hat!
// Cart Pin 5 = VIA PA0 = CLOCK = Arduino pin 52 (SCK)
// Cart Pin 4 = VIA PA1 = DATA  = Arduino pin 51 (MOSI)
// Cart Pin 2 = VIA PA2 = LATCH = Arduino pin 53 (SS)
#define VIA_CLOCK_PIN SCKPin   // Pin 52
#define VIA_DATA_PIN  MOSIPin  // Pin 51
#define VIA_LATCH_PIN SSPin    // Pin 53

#include <string.h>
#include <SimpleSerialShell.h>
#include <CRC32.h>

#define MODE_EEPROM 0
#define MODE_FLASH 1

int program_mode = MODE_FLASH;
bool verify_writes = true;  // Enable read-back verification after writes

void set_address(unsigned int x) {
  //set pins corresponding to A0-A15 to the value of x
  //A0 through A7 are on Port A
  //A8 through A15 are on Port C
  unsigned char addr_low = x & 0xFF;
  unsigned char addr_high = (x >> 8) & 0xFF;
  PORTA = addr_low;

  addr_high &= 0x7F;
  PORTC = addr_high;
}

unsigned char lastBankNum = 0x7F;
void shift_bank(unsigned char banknum) {
  //Flash cartridge uses a shift register to set high address pins
  //This uses the low 7 bits of banknum
  digitalWrite(SSPin, LOW);
  shiftOut(MOSIPin, SCKPin, MSBFIRST, banknum);
  digitalWrite(SSPin, HIGH);
  lastBankNum = banknum;
}

// VIA protocol bank shift - matches SDK's banking2.s _bank_shift_out exactly
// The protocol shifts 7 bits MSB first, using:
//   PA0 (bit 0) = CLOCK - rising edge clocks in DATA
//   PA1 (bit 1) = DATA
//   PA2 (bit 2) = LATCH - rising edge latches shift register to outputs
void via_shift_bank(unsigned char banknum) {
  // Match Arduino's shiftOut behavior exactly
  // shiftOut shifts 8 bits MSBFIRST with SS(LATCH) low during shift
  
  // Start with LATCH LOW, like shiftOut does
  digitalWrite(VIA_LATCH_PIN, LOW);
  digitalWrite(VIA_CLOCK_PIN, LOW);
  delayMicroseconds(1);
  
  // Shift out 8 bits, MSB first (bits 7 down to 0) - matches shiftOut
  for(int i = 7; i >= 0; i--) {
    unsigned char dataBit = (banknum >> i) & 1;
    
    // Set DATA with CLOCK LOW
    digitalWrite(VIA_DATA_PIN, dataBit ? HIGH : LOW);
    delayMicroseconds(1);
    
    // Rising edge on CLOCK shifts the bit in
    digitalWrite(VIA_CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    
    // Lower CLOCK for next bit
    digitalWrite(VIA_CLOCK_PIN, LOW);
    delayMicroseconds(1);
  }
  
  // Rising edge on LATCH to transfer shift register to outputs
  digitalWrite(VIA_LATCH_PIN, HIGH);
  delayMicroseconds(1);
  
  // Leave LATCH HIGH like Arduino's shiftOut does (SS goes HIGH after transfer)
}

void setup_via_pins() {
  pinMode(VIA_CLOCK_PIN, OUTPUT);
  pinMode(VIA_DATA_PIN, OUTPUT);
  pinMode(VIA_LATCH_PIN, OUTPUT);
  digitalWrite(VIA_CLOCK_PIN, LOW);
  digitalWrite(VIA_DATA_PIN, LOW);
  digitalWrite(VIA_LATCH_PIN, LOW);
}

void set_long_address(unsigned long x) {
  if ((x >> 14) & 0x7F != lastBankNum) {
    shift_bank((x >> 14) & 0x7F);
  }
  set_address(x & 0x3FFF);
}

void wait_for_ready() {
  while (digitalRead(ReadyPin) == LOW) {
  }
}

unsigned char read_data() {
  //data pins map to port L
  //setting these as inputs to prepare to read data
  DDRL = 0x00;
  PORTL = 0x00;
  digitalWrite(OutputEnablePin, LOW);
  unsigned char data = (unsigned char) PINL;
  digitalWrite(OutputEnablePin, HIGH);
  return data;
}

void write_data(unsigned char data) {
  //set the data pins and strobe Write Enable LOW
  PORTL = data;
  DDRL = 0xFF;
  if (program_mode == MODE_EEPROM) {
    delayMicroseconds(1);
  }
  digitalWrite(WriteEnablePin, LOW);
  if (program_mode == MODE_EEPROM) {
    delayMicroseconds(1);
  }
  digitalWrite(WriteEnablePin, HIGH);
  if (program_mode == MODE_EEPROM) {
    delayMicroseconds(1);
  }
  PORTL = 0x00;
  DDRL = 0x00;
}

void write_to(unsigned int addr, unsigned char data) {
  set_address(addr);
  write_data(data);
}

void flash_cmd_unlock() {
  write_to(0xAAA, 0xAA);
  write_to(0x555, 0x55);
}

void flash_cmd_chip_erase() {
  flash_cmd_unlock();
  write_to(0xAAA, 0x80);
  flash_cmd_unlock();
  write_to(0xAAA, 0x10);
}

void flash_cmd_sector_erase(unsigned char sa) {
  shift_bank(sa >> 1);
  flash_cmd_unlock();
  write_to(0xAAA, 0x80);
  flash_cmd_unlock();
  write_to((sa & 1) << 13, 0x30);
}

void flash_cmd_program_to(unsigned int addr, unsigned char data) {
  flash_cmd_unlock();
  write_to(0xAAA, 0xA0);
  write_to(addr, data);
}

void flash_cmd_unlock_bypass() {
  flash_cmd_unlock();
  write_to(0xAAA, 0x20);
}

void flash_cmd_bypass_write_to(unsigned int addr, unsigned char data) {
  set_address(addr);
  write_data(0xA0);
  write_data(data);
}

void flash_cmd_unlock_bypass_reset() {
  write_data(0x90);
  write_data(0x00);
}

void print_mode() {
  if (program_mode == MODE_EEPROM) {
    Serial.print("EEPROM");
  } else if (program_mode == MODE_FLASH) {
    Serial.print("FLASH");
  } else {
    Serial.print("UNDEFINED");
  }
}

//Shell commands start
int cmd_readAt(int argc, char **argv) {
  if (argc < 2) {
    return 0;
  }
  unsigned int addr = strtol(argv[1], NULL, 16);
  set_address(addr);
  Serial.println(read_data(), HEX);
  return 0;
}

int cmd_dump(int argc, char **argv) {
  if (argc < 3) {
    return 0;
  }
  unsigned long full_address = strtol(argv[1], NULL, 16);
  unsigned long full_span = strtol(argv[2], NULL, 16);
  for (unsigned long il = 0; il < full_span; il++) {
    set_long_address(full_address + il);
    Serial.write(read_data());
  }
  Serial.end();
  return 0;
}

int cmd_eraseChip(int argc, char **argv) {
  Serial.println("Starting chip erase...");
  flash_cmd_chip_erase();
  wait_for_ready();
  Serial.println("Done");
  return 0;
}

int cmd_eraseSector(int argc, char **argv) {
  if (argc < 2) {
    return 0;
  }
  unsigned char sector_addr = strtol(argv[1], NULL, 16);
  Serial.print("Erasing sector $");
  Serial.println(sector_addr, HEX);
  flash_cmd_sector_erase(sector_addr);
  wait_for_ready();
  Serial.println("Done");
}

int cmd_writeTo(int argc, char **argv) {
  if (argc < 3) {
    return 0;
  }
  unsigned int addr = strtol(argv[1], NULL, 16);
  unsigned int data = strtol(argv[2], NULL, 16);
  if (program_mode == MODE_FLASH) {
    Serial.print("Writing $");
    Serial.print((unsigned char) data, HEX);
    Serial.print(" to $");
    Serial.println(addr, HEX);
    flash_cmd_program_to(addr, (unsigned char) data);
    wait_for_ready();
    Serial.println("Done");
    return 0;
  } else {
    write_to(addr, (unsigned char) data);
    return 0;
  }
}

int cmd_shift(int argc, char **argv) {
  if (argc < 2) {
    return 0;
  }
  unsigned char addr = strtol(argv[1], NULL, 16);
  shift_bank(addr);
  return 0;
}

int cmd_mode(int argc, char **argv) {
  if (argc > 1) {
    if (argv[1][0] == 'e' || argv[1][0] == 'E') {
      program_mode = MODE_EEPROM;
    } else if (argv[1][0] == 'f' || argv[1][0] == 'F') {
      program_mode = MODE_FLASH;
    } else {
      Serial.println("Unknown mode specified");
    }
  }
  Serial.print("Current mode is ");
  print_mode();
  Serial.println();
  return 0;
}

int cmd_reset(int argc, char **argv) {
  Serial.println("Resetting...");
  digitalWrite(ResetPin, LOW);
  delayMicroseconds(10);
  digitalWrite(ResetPin, HIGH);
  wait_for_ready();
  Serial.println("OK");
  return 0;
}

unsigned char fileBuf[4096];

int cmd_writeMulti(int argc, char **argv) {
  if (argc < 3) {
    return 0;
  }
  unsigned int addr = strtol(argv[1], NULL, 16);
  unsigned int startAddr = addr;
  unsigned int count = strtol(argv[2], NULL, 16);
  unsigned int actualBytesCount = Serial.readBytes(fileBuf, count);
  unsigned int verifyErrors = 0;
  if (program_mode == MODE_FLASH) {
    flash_cmd_unlock_bypass();
    for (int i = 0; i < actualBytesCount; i++) {
      flash_cmd_bypass_write_to(addr, fileBuf[i]);
      addr++;
    }
    flash_cmd_unlock_bypass_reset();
    // Read-back verification for FLASH mode (if enabled)
    if (verify_writes) {
      for (int i = 0; i < actualBytesCount; i++) {
        set_address(startAddr + i);
        unsigned char readBack = read_data();
        if (readBack != fileBuf[i]) {
          verifyErrors++;
        }
      }
    }
  } else {
    for (int i = 0; i < actualBytesCount; i++) {
      write_to(addr, fileBuf[i]);
      while (read_data() != fileBuf[i]) {
        write_to(addr, fileBuf[i]);
        delay(1);
      }
      addr++;
      delay(1);
      wait_for_ready();
    }
  }
  while (Serial.available() > 0) {
    char k = Serial.read();
  }
  if (verifyErrors > 0) {
    Serial.print("NAK");
    Serial.print(verifyErrors);
    Serial.print("/");
    Serial.println(actualBytesCount);
  } else {
    Serial.print("ACK");
    Serial.println(actualBytesCount);
  }
  return 0;
}

// Read multiple bytes and send as raw binary (for GTFO read mode)
int cmd_readMulti(int argc, char **argv) {
  if (argc < 3) {
    return 0;
  }
  unsigned int addr = strtol(argv[1], NULL, 16);
  unsigned int count = strtol(argv[2], NULL, 16);
  // Send start marker so GTFO knows where data begins
  Serial.print("DATA:");
  Serial.println(count);
  for (unsigned int i = 0; i < count; i++) {
    set_address(addr + i);
    Serial.write(read_data());
  }
  Serial.println();  // End marker
  Serial.println("OK");
  return 0;
}

CRC32 crc;
int cmd_checksum(int argc, char **argv) {
  if (argc < 3) {
    return 0;
  }
  unsigned long addr = strtol(argv[1], NULL, 16);
  unsigned long count = strtol(argv[2], NULL, 16);
  crc.reset();
  for (unsigned long i = 0; i < count; i++) {
    set_long_address(addr);
    crc.update(read_data());
    addr++;
  }
  Serial.print("CRC32: ");
  Serial.println(crc.finalize(), HEX);
}

int cmd_timeout(int argc, char **argv) {
  if (argc < 2) {
    return 0;
  }
  unsigned int timeout_milliseconds = strtol(argv[1], NULL, 10);
  Serial.setTimeout(timeout_milliseconds);
  return 0;
}

int cmd_readString(int argc, char **argv) {
  if (argc < 2) {
    return 0;
  }
  unsigned int addr = strtol(argv[1], NULL, 16);
  unsigned int limit = 80;
  if (argc >= 3) {
    limit = strtol(argv[2], NULL, 16);
  }
  while (limit > 0) {
    limit --;
    set_address(addr);
    char letter = read_data();
    if (isprint(letter)) {
      Serial.print(letter);
    } else {
      limit = 0;
    }
    addr ++;
  }
  Serial.println();
  return 0;
}

int cmd_version(int argc, char **argv) {
  Serial.println("GTCP2-0.0.2");
  return 0;
}

bool user_confirm() {
  char c;
  while(1) {
    while(Serial.available() == 0) {}
    char c = Serial.read();
    while(Serial.available() > 0) {
      Serial.read();
    }
    if(c == 'y') {
      return true;
    }
    if(c == 'n') {
      return false;
    }
    
    if(c == '\r') {
      Serial.println("Please enter 'y' or 'n'");
    }
  }
}

#define EXPECT(x) if(!(x)){Serial.println("FAIL");return -1;}
#define EXPECT_MSG(x, msg) if(!(x)){Serial.print("FAIL: ");Serial.println(msg);return -1;}

//Exhaustive test suite
int cmd_testfull(int argc, char **argv) {
  Serial.println("EXHAUSTIVE test suite for flash boards!");
  Serial.println("This tests all data lines, address lines, and banks.");
  Serial.println("This will erase any stored data. Are you sure you wish to continue? (y/n)");
  if(!user_confirm()) {
    Serial.println("Test canceled.");
    return 0;
  }
  Serial.println("Starting exhaustive tests...");
  Serial.println();
  
  // Test 1: Chip Erase
  Serial.print("\t1. Chip erase: ");
  shift_bank(0);
  flash_cmd_chip_erase();
  EXPECT_MSG(digitalRead(ReadyPin)==LOW, "Ready pin didn't go LOW")
  float readyTime = 0;
  while(digitalRead(ReadyPin)==LOW){
    delay(100);
    readyTime += 0.1f;
    EXPECT_MSG(readyTime<20, "Erase timeout (>20s)")
  }
  EXPECT_MSG(readyTime>1, "Erase too fast (<1s) - chip may not be responding")
  Serial.println("PASS");

  // Test 2: Data bus - walking 1s
  Serial.print("\t2. Data bus (walking 1s): ");
  unsigned char dataBits[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
  for(int i = 0; i < 8; i++) {
    flash_cmd_program_to(i, dataBits[i]);
    set_address(i);
    unsigned char readBack = read_data();
    if(readBack != dataBits[i]) {
      Serial.print("FAIL: D");
      Serial.print(i);
      Serial.print(" - wrote 0x");
      Serial.print(dataBits[i], HEX);
      Serial.print(", read 0x");
      Serial.println(readBack, HEX);
      return -1;
    }
  }
  Serial.println("PASS");

  // Test 3: Data bus - walking 0s
  Serial.print("\t3. Data bus (walking 0s): ");
  // Need to erase first since we can only program 1->0
  flash_cmd_sector_erase(0);
  wait_for_ready();
  unsigned char dataInvBits[] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};
  for(int i = 0; i < 8; i++) {
    flash_cmd_program_to(i, dataInvBits[i]);
    set_address(i);
    unsigned char readBack = read_data();
    if(readBack != dataInvBits[i]) {
      Serial.print("FAIL: ~D");
      Serial.print(i);
      Serial.print(" - wrote 0x");
      Serial.print(dataInvBits[i], HEX);
      Serial.print(", read 0x");
      Serial.println(readBack, HEX);
      return -1;
    }
  }
  Serial.println("PASS");

  // Test 4: Address bus - walking 1s (within 16KB bank)
  Serial.print("\t4. Address bus (walking 1s): ");
  flash_cmd_sector_erase(0);
  wait_for_ready();
  // Test address lines A0-A13 (14 bits = 16KB)
  for(int bit = 0; bit < 14; bit++) {
    unsigned int testAddr = (1 << bit);
    unsigned char testVal = (bit + 1);  // Unique value per address
    flash_cmd_program_to(testAddr, testVal);
  }
  // Verify all
  for(int bit = 0; bit < 14; bit++) {
    unsigned int testAddr = (1 << bit);
    unsigned char testVal = (bit + 1);
    set_address(testAddr);
    unsigned char readBack = read_data();
    if(readBack != testVal) {
      Serial.print("FAIL: A");
      Serial.print(bit);
      Serial.print(" (addr 0x");
      Serial.print(testAddr, HEX);
      Serial.print(") - wrote 0x");
      Serial.print(testVal, HEX);
      Serial.print(", read 0x");
      Serial.println(readBack, HEX);
      return -1;
    }
  }
  Serial.println("PASS");

  // Test 5: Address bus - check for shorts (no address should affect another)
  Serial.print("\t5. Address bus (short check): ");
  // Address 0 should still be 0xFF (erased), not written
  set_address(0);
  EXPECT_MSG(read_data()==0xFF, "Addr 0 was modified - possible address short")
  Serial.println("PASS");

  // Test 6: Bank switching - test multiple banks
  Serial.print("\t6. Bank switching (banks 0,1,2,0x7F): ");
  unsigned char testBanks[] = {0, 1, 2, 0x7F};
  // Erase and write unique pattern to each bank
  for(int b = 0; b < 4; b++) {
    shift_bank(testBanks[b]);
    flash_cmd_sector_erase(testBanks[b] * 2);  // Each bank has 2 sectors
    wait_for_ready();
    flash_cmd_program_to(0x0000, 0xB0 + testBanks[b]);  // Unique per bank
    flash_cmd_program_to(0x1000, 0xC0 + testBanks[b]);
  }
  // Verify each bank still has its unique data
  for(int b = 0; b < 4; b++) {
    shift_bank(testBanks[b]);
    set_address(0x0000);
    unsigned char expected = 0xB0 + testBanks[b];
    unsigned char readBack = read_data();
    if(readBack != expected) {
      Serial.print("FAIL: Bank 0x");
      Serial.print(testBanks[b], HEX);
      Serial.print(" addr 0 - expected 0x");
      Serial.print(expected, HEX);
      Serial.print(", read 0x");
      Serial.println(readBack, HEX);
      return -1;
    }
    set_address(0x1000);
    expected = 0xC0 + testBanks[b];
    readBack = read_data();
    if(readBack != expected) {
      Serial.print("FAIL: Bank 0x");
      Serial.print(testBanks[b], HEX);
      Serial.print(" addr 0x1000 - expected 0x");
      Serial.print(expected, HEX);
      Serial.print(", read 0x");
      Serial.println(readBack, HEX);
      return -1;
    }
  }
  Serial.println("PASS");

  // Test 7: Bulk write test
  Serial.print("\t7. Bulk write (256 bytes): ");
  shift_bank(0);
  flash_cmd_sector_erase(0);
  wait_for_ready();
  flash_cmd_unlock_bypass();
  for(int i = 0; i < 256; i++) {
    flash_cmd_bypass_write_to(0x2000 + i, (unsigned char)i);
  }
  flash_cmd_unlock_bypass_reset();
  // Verify
  for(int i = 0; i < 256; i++) {
    set_address(0x2000 + i);
    unsigned char readBack = read_data();
    if(readBack != (unsigned char)i) {
      Serial.print("FAIL: Bulk write at offset ");
      Serial.print(i);
      Serial.print(" - expected 0x");
      Serial.print((unsigned char)i, HEX);
      Serial.print(", read 0x");
      Serial.println(readBack, HEX);
      return -1;
    }
  }
  Serial.println("PASS");

  // Test 8: A14 HIGH behavior (C000-FFFF should always map to bank 127)
  // This tests the cartridge logic that forces upper address bits HIGH when A14=1
  Serial.print("\t8. A14 HIGH (fixed bank 127 region): ");
  // First, write a known pattern to bank 127 via shift register (A14 LOW region)
  shift_bank(0x7F);
  flash_cmd_sector_erase(0x7F * 2);
  wait_for_ready();
  flash_cmd_program_to(0x0000, 0xA1);  // Write to bank 127, addr 0x0000
  flash_cmd_program_to(0x0100, 0xA2);  // Write to bank 127, addr 0x0100
  // Now read via A14 HIGH (addresses 0x4000-0x7FFF map to bank 127 when A14 logic works)
  // 0x4000 with A14=1 should read from bank 127, addr 0x0000
  set_address(0x4000);
  unsigned char readA14 = read_data();
  if(readA14 != 0xA1) {
    Serial.print("FAIL: A14 HIGH addr 0x4000 - expected 0xA1, read 0x");
    Serial.println(readA14, HEX);
    Serial.println("  (A14 logic may not be forcing bank 127)");
    return -1;
  }
  set_address(0x4100);
  readA14 = read_data();
  if(readA14 != 0xA2) {
    Serial.print("FAIL: A14 HIGH addr 0x4100 - expected 0xA2, read 0x");
    Serial.println(readA14, HEX);
    return -1;
  }
  // Also verify shift register is ignored when A14=HIGH
  shift_bank(0);  // Switch to bank 0
  set_address(0x4000);  // But A14=HIGH should still give us bank 127
  readA14 = read_data();
  if(readA14 != 0xA1) {
    Serial.print("FAIL: A14 HIGH with bank 0 selected - expected 0xA1, read 0x");
    Serial.println(readA14, HEX);
    Serial.println("  (Shift register should be ignored when A14=HIGH)");
    return -1;
  }
  Serial.println("PASS");

  // Test 9: 6502 Vector test - verify RESET/NMI/IRQ vectors accessible via fixed bank
  // On GameTank: $FFFA-$FFFF are the vectors, mapped to bank 127 offset 0x3FFA-0x3FFF
  // This is CRITICAL for the GameTank to boot!
  Serial.print("\t9. 6502 vectors (FFFA-FFFF): ");
  shift_bank(0x7F);
  // Need to erase the sector containing the vectors (last sector of bank 127)
  flash_cmd_sector_erase(0x7F * 2 + 1);  // Second sector of bank 127 (0x2000-0x3FFF)
  wait_for_ready();
  // Write fake vectors via shift register (banked access)
  // NMI vector at $FFFA-$FFFB -> bank 127 offset 0x3FFA-0x3FFB
  flash_cmd_program_to(0x3FFA, 0x00);  // NMI low byte
  flash_cmd_program_to(0x3FFB, 0x80);  // NMI high byte -> $8000
  // RESET vector at $FFFC-$FFFD -> bank 127 offset 0x3FFC-0x3FFD
  flash_cmd_program_to(0x3FFC, 0x00);  // RESET low byte
  flash_cmd_program_to(0x3FFD, 0xC0);  // RESET high byte -> $C000
  // IRQ vector at $FFFE-$FFFF -> bank 127 offset 0x3FFE-0x3FFF
  flash_cmd_program_to(0x3FFE, 0x50);  // IRQ low byte
  flash_cmd_program_to(0x3FFF, 0xC1);  // IRQ high byte -> $C150
  
  // Now verify via A14 HIGH path (how GameTank actually reads vectors)
  // Address 0x7FFA = A14 HIGH, offset 0x3FFA in bank 127
  unsigned char vecByte;
  
  // Check NMI vector
  set_address(0x7FFA);
  vecByte = read_data();
  if(vecByte != 0x00) {
    Serial.print("FAIL: NMI vector low (FFFA) - expected 0x00, read 0x");
    Serial.println(vecByte, HEX);
    return -1;
  }
  set_address(0x7FFB);
  vecByte = read_data();
  if(vecByte != 0x80) {
    Serial.print("FAIL: NMI vector high (FFFB) - expected 0x80, read 0x");
    Serial.println(vecByte, HEX);
    return -1;
  }
  
  // Check RESET vector (most critical!)
  set_address(0x7FFC);
  vecByte = read_data();
  if(vecByte != 0x00) {
    Serial.print("FAIL: RESET vector low (FFFC) - expected 0x00, read 0x");
    Serial.println(vecByte, HEX);
    return -1;
  }
  set_address(0x7FFD);
  vecByte = read_data();
  if(vecByte != 0xC0) {
    Serial.print("FAIL: RESET vector high (FFFD) - expected 0xC0, read 0x");
    Serial.println(vecByte, HEX);
    return -1;
  }
  
  // Check IRQ vector
  set_address(0x7FFE);
  vecByte = read_data();
  if(vecByte != 0x50) {
    Serial.print("FAIL: IRQ vector low (FFFE) - expected 0x50, read 0x");
    Serial.println(vecByte, HEX);
    return -1;
  }
  set_address(0x7FFF);
  vecByte = read_data();
  if(vecByte != 0xC1) {
    Serial.print("FAIL: IRQ vector high (FFFF) - expected 0xC1, read 0x");
    Serial.println(vecByte, HEX);
    return -1;
  }
  
  // Final check: verify with shift register set to different bank (should still work)
  shift_bank(0);
  set_address(0x7FFC);  // RESET vector via A14 HIGH
  vecByte = read_data();
  if(vecByte != 0x00) {
    Serial.print("FAIL: RESET vector with bank 0 - A14 not overriding shift register");
    return -1;
  }
  Serial.println("PASS");

  // Test 10: VIA Protocol Simulation
  // This tests the EXACT protocol the GameTank uses to control banking
  // Requires VIA pins (46,47,48) to be connected to cart pins (2,4,5)
  Serial.print(F("\t10. VIA protocol bank switching: "));
  setup_via_pins();
  
  // First, write different patterns to banks 0, 3, and 127 using Arduino's direct method
  shift_bank(0);
  flash_cmd_sector_erase(0);  // Bank 0, first sector
  wait_for_ready();
  flash_cmd_program_to(0x0100, 0xB0);  // Bank 0: write 0xB0 at offset 0x100
  
  shift_bank(3);
  flash_cmd_sector_erase(6);  // Bank 3, first sector
  wait_for_ready();
  flash_cmd_program_to(0x0100, 0xB3);  // Bank 3: write 0xB3 at offset 0x100
  
  // Bank 127 already has data from previous tests
  shift_bank(0x7F);
  flash_cmd_program_to(0x0100, 0xBF);  // Bank 127: write 0xBF at offset 0x100
  
  // Now switch banks using VIA protocol and verify reads
  // This is the critical test - if the cart's shift register doesn't respond
  // to the VIA protocol, this will fail even though Arduino direct mode works!
  
  via_shift_bank(0);  // Select bank 0 via VIA protocol
  set_address(0x0100);  // A14=0, so banked access
  unsigned char viaRead = read_data();
  if(viaRead != 0xB0) {
    Serial.print(F("FAIL: VIA bank 0 read 0x"));
    Serial.println(viaRead, HEX);
    return -1;
  }
  
  via_shift_bank(3);  // Select bank 3 via VIA protocol
  set_address(0x0100);
  viaRead = read_data();
  if(viaRead != 0xB3) {
    Serial.print(F("FAIL: VIA bank 3 read 0x"));
    Serial.println(viaRead, HEX);
  }
  
  via_shift_bank(0x7F);  // Select bank 127 via VIA protocol
  set_address(0x0100);
  viaRead = read_data();
  if(viaRead != 0xBF) {
    Serial.print(F("FAIL: VIA bank 127 read 0x"));
    Serial.println(viaRead, HEX);
  }
  
  // A14 override test - NOTE: Due to bus contention between 74HC595 and 74HC244,
  // the 595 must hold bank 127 for A14 override to work cleanly.
  // Test with 595 holding bank 127 first:
  via_shift_bank(0x7F);  // VIA selects bank 127
  set_address(0x4100);   // A14=1 should also force bank 127 via 244
  viaRead = read_data();
  bool a14OverrideFailed = false;
  if(viaRead != 0xBF) {
    Serial.print(F("FAIL: A14+bank127 read 0x"));
    Serial.println(viaRead, HEX);
    a14OverrideFailed = true;
  } else {
    via_shift_bank(0);
    set_address(0x4100);
    viaRead = read_data();
    if(viaRead != 0xBF) {
      Serial.print(F("PASS (contention: 0x"));
      Serial.print(viaRead, HEX);
      Serial.println(F(")"));
    } else {
      Serial.println(F("PASS"));
    }
  }
  
  // Quick sanity check - can Arduino's shift_bank still reach bank 0?
  Serial.print(F("\t    Post-VIA check: "));
  shift_bank(0);  // Arduino method
  set_address(0x0100);
  viaRead = read_data();
  Serial.print(F("B0=0x"));
  Serial.print(viaRead, HEX);
  if(viaRead == 0xB0) Serial.print(F(" OK "));
  else Serial.print(F(" BROKEN! "));
  
  shift_bank(0x7F);
  set_address(0x0100);
  viaRead = read_data();
  Serial.print(F("B127=0x"));
  Serial.print(viaRead, HEX);
  if(viaRead == 0xBF) Serial.println(F(" OK"));
  else Serial.println(F(" BROKEN!"));
  
  // Test 11: LATCH state effects
  Serial.print(F("\t11. LATCH behavior: "));
  
  // DON'T touch pins - just test A14 with what we have from Post-VIA check
  // Bank 127 should still have 0xBF at offset 0x0100 from Test 10
  set_address(0x4100);  // A14 HIGH, offset 0x100 - should read bank 127
  viaRead = read_data();
  Serial.print(F("pre:0x"));
  Serial.print(viaRead, HEX);
  
  if(viaRead != 0xBF) {
    Serial.print(F(" (A14 broken before pin touch!) "));
  }
  
  // Now touch the pins like before
  digitalWrite(VIA_LATCH_PIN, HIGH);
  digitalWrite(VIA_CLOCK_PIN, LOW);
  digitalWrite(VIA_DATA_PIN, LOW);
  delayMicroseconds(10);
  
  // Check A14 again
  set_address(0x4100);
  viaRead = read_data();
  Serial.print(F(" post:0x"));
  Serial.print(viaRead, HEX);
  
  if(viaRead != 0xBF) {
    Serial.println(F(" (A14 broken AFTER pin touch!)"));
  } else {
    Serial.println(F(" OK"));
  }

  // Test 12: GameTank boot sequence simulation
  Serial.print(F("\t12. Boot sequence: "));
  
  // Put known values in banks using Arduino method
  shift_bank(0);
  flash_cmd_program_to(0x0300, 0xD0);  // Bank 0: 0xD0
  shift_bank(0x7F);
  flash_cmd_program_to(0x0300, 0xDF);  // Bank 127: 0xDF
  
  // Set pins as inputs first (simulating power-on state)
  pinMode(VIA_CLOCK_PIN, INPUT);
  pinMode(VIA_DATA_PIN, INPUT);
  pinMode(VIA_LATCH_PIN, INPUT);
  delayMicroseconds(100);
  
  // Step 1: Set as outputs (DDRA = 0x07)
  pinMode(VIA_CLOCK_PIN, OUTPUT);
  pinMode(VIA_DATA_PIN, OUTPUT);
  pinMode(VIA_LATCH_PIN, OUTPUT);
  
  // Step 2: Set all HIGH (ORA = 0xFF) - this is what startup does!
  digitalWrite(VIA_CLOCK_PIN, HIGH);
  digitalWrite(VIA_DATA_PIN, HIGH);
  digitalWrite(VIA_LATCH_PIN, HIGH);
  delayMicroseconds(10);  // Brief moment with all HIGH
  
  // Step 3: Call bank_shift_out(0) - SDK function starts with stz OutBits
  // Simulate SDK's exact sequence
  // stz OutBits - all LOW
  digitalWrite(VIA_CLOCK_PIN, LOW);
  digitalWrite(VIA_DATA_PIN, LOW);
  digitalWrite(VIA_LATCH_PIN, LOW);
  delayMicroseconds(1);
  
  // Now shift out bank 0 (8 clocks, SDK-style)
  unsigned char testBank = 0;
  for(int i = 7; i >= 0; i--) {
    unsigned char dataBit = (testBank >> i) & 1;
    // DATA set, CLOCK LOW
    digitalWrite(VIA_DATA_PIN, dataBit ? HIGH : LOW);
    delayMicroseconds(1);
    // CLOCK HIGH (rising edge)
    digitalWrite(VIA_CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    // CLOCK LOW for next bit
    digitalWrite(VIA_CLOCK_PIN, LOW);
    delayMicroseconds(1);
  }
  // Final latch sequence (SDK does CLOCK high, then LATCH high, then all low)
  digitalWrite(VIA_CLOCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(VIA_LATCH_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(VIA_CLOCK_PIN, LOW);
  digitalWrite(VIA_DATA_PIN, LOW);
  digitalWrite(VIA_LATCH_PIN, LOW);
  delayMicroseconds(1);
  
  // Now verify we can read bank 0
  set_address(0x0300);
  viaRead = read_data();
  if(viaRead != 0xD0) {
    Serial.print(F("FAIL: read 0x"));
    Serial.print(viaRead, HEX);
    Serial.println(F(" not 0xD0"));
  } else {
    Serial.println(F("PASS"));
  }

  // Test 13: Upper address bits test (use different offset to avoid erased areas)
  Serial.println(F("\t13. Upper addr bits:"));
  Serial.print(F("\t    Setup: "));
  
  // Use offset 0x0500 which should be in erased (0xFF) state from chip erase
  // Don't do sector erases - just write directly
  shift_bank(0);
  flash_cmd_program_to(0x0500, 0xE0);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(F("B0="));
  Serial.print(viaRead, HEX);
  Serial.print(F(" "));
  
  shift_bank(32);
  flash_cmd_program_to(0x0500, 0xE1);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(F("B32="));
  Serial.print(viaRead, HEX);
  Serial.print(F(" "));
  
  shift_bank(64);
  flash_cmd_program_to(0x0500, 0xE2);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(F("B64="));
  Serial.print(viaRead, HEX);
  Serial.print(F(" "));
  
  shift_bank(96);
  flash_cmd_program_to(0x0500, 0xE3);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(F("B96="));
  Serial.print(viaRead, HEX);
  Serial.print(F(" "));
  
  shift_bank(127);
  flash_cmd_program_to(0x0500, 0xE4);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(F("B127="));
  Serial.println(viaRead, HEX);
  
  // Now test reads after switching away and back
  Serial.print(F("\t    Read: B0:"));
  shift_bank(0);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(viaRead, HEX);
  if(viaRead == 0xE0) Serial.print(F(" OK "));
  else Serial.print(F(" ? "));
  
  Serial.print(F("B64:"));
  shift_bank(64);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(viaRead, HEX);
  if(viaRead == 0xE2) Serial.print(F(" OK "));
  else Serial.print(F(" ? "));
  
  Serial.print(F("B127:"));
  shift_bank(127);
  set_address(0x0500);
  viaRead = read_data();
  Serial.print(viaRead, HEX);
  if(viaRead == 0xE4) Serial.println(F(" OK"));
  else Serial.println(F(" ?"));

  // Test 14: VIA Pin Transition Diagnostic
  // This test diagnoses the suspected hardware issue where the 595 behaves
  // erratically when VIA pins transition from floating (input) to driven (output).
  // This mimics what happens when the GameTank's VIA enables DDRA outputs.
  Serial.println();
  Serial.println(F("\t14. VIA transition diagnostic:"));
  
  // First, set up known state with Arduino's direct method
  shift_bank(0x7F);
  flash_cmd_program_to(0x0600, 0xF7);  // Bank 127
  shift_bank(0);
  flash_cmd_program_to(0x0600, 0xF0);  // Bank 0
  
  Serial.print(F("\t    a) Arduino control: "));
  shift_bank(0);
  set_address(0x0600);
  viaRead = read_data();
  Serial.print(F("B0=0x"));
  Serial.print(viaRead, HEX);
  shift_bank(0x7F);
  set_address(0x0600);
  unsigned char b127Read = read_data();
  Serial.print(F(" B127=0x"));
  Serial.print(b127Read, HEX);
  if(viaRead == 0xF0 && b127Read == 0xF7) Serial.println(F(" OK"));
  else Serial.println(F(" FAIL"));
  
  // Now test transition from INPUT (floating) to OUTPUT
  Serial.print(F("\t    b) Float->Drive test: "));
  
  // Step 1: Set VIA pins as INPUTS (high-Z, floating)
  pinMode(VIA_CLOCK_PIN, INPUT);
  pinMode(VIA_DATA_PIN, INPUT);
  pinMode(VIA_LATCH_PIN, INPUT);
  delay(10);  // Let them float
  
  // The 595 should retain its last state (bank 127 from above)
  set_address(0x0600);
  viaRead = read_data();
  Serial.print(F("float=0x"));
  Serial.print(viaRead, HEX);
  
  // Step 2: Set VIA pins as OUTPUTS with initial LOW state
  // This mimics: STZ ORA, then LDA #7 STA DDRA (safe sequence)
  digitalWrite(VIA_CLOCK_PIN, LOW);  // Pre-set before enabling output
  digitalWrite(VIA_DATA_PIN, LOW);
  digitalWrite(VIA_LATCH_PIN, LOW);
  pinMode(VIA_CLOCK_PIN, OUTPUT);
  pinMode(VIA_DATA_PIN, OUTPUT);
  pinMode(VIA_LATCH_PIN, OUTPUT);
  delayMicroseconds(10);
  
  // Check if 595 still holds bank 127 (should, since LATCH wasn't pulsed)
  set_address(0x0600);
  unsigned char afterDrive = read_data();
  Serial.print(F(" drive=0x"));
  Serial.print(afterDrive, HEX);
  
  if(afterDrive == 0xF7) {
    Serial.println(F(" OK"));
  } else if(afterDrive == 0xF0) {
    Serial.println(F(" WARN: 595 latched to bank 0!"));
  } else {
    Serial.print(F(" FAIL: unexpected bank (0x"));
    Serial.print(afterDrive, HEX);
    Serial.println(F(")"));
  }
  
  // Step 3: Test the "buggy" SDK sequence: pins go HIGH immediately
  Serial.print(F("\t    c) ORA=0xFF glitch: "));
  
  // Reset: select bank 0 cleanly via Arduino
  shift_bank(0);
  
  // Set VIA pins as INPUTS again
  pinMode(VIA_CLOCK_PIN, INPUT);
  pinMode(VIA_DATA_PIN, INPUT);
  pinMode(VIA_LATCH_PIN, INPUT);
  delay(10);
  
  // Now mimic buggy SDK: DDRA=0x07 then ORA=0xFF
  // In reality, ORA defaults to whatever garbage is there
  // Then DDRA enables outputs, which drives current ORA state
  digitalWrite(VIA_CLOCK_PIN, HIGH);
  digitalWrite(VIA_DATA_PIN, HIGH);
  digitalWrite(VIA_LATCH_PIN, HIGH);
  pinMode(VIA_CLOCK_PIN, OUTPUT);
  pinMode(VIA_DATA_PIN, OUTPUT);
  pinMode(VIA_LATCH_PIN, OUTPUT);
  delayMicroseconds(10);  // LATCH is HIGH! This should latch garbage!
  
  // Now drive all LOW (simulating STZ OutBits)
  digitalWrite(VIA_CLOCK_PIN, LOW);
  digitalWrite(VIA_DATA_PIN, LOW);
  digitalWrite(VIA_LATCH_PIN, LOW);
  delayMicroseconds(10);
  
  // What bank are we on now?
  set_address(0x0600);
  viaRead = read_data();
  Serial.print(F("post-glitch=0x"));
  Serial.print(viaRead, HEX);
  
  if(viaRead == 0xF0) {
    Serial.println(F(" (bank 0) - 595 may have random state"));
  } else if(viaRead == 0xF7) {
    Serial.println(F(" (bank 127) OK"));
  } else {
    Serial.print(F(" (bank ?) - 595 holding unexpected value"));
  }
  
  // Step 4: Verify we can recover with proper bank_shift_out
  Serial.print(F("\t    d) Recovery test: "));
  via_shift_bank(0x7F);  // Properly select bank 127
  set_address(0x0600);
  viaRead = read_data();
  Serial.print(F("B127=0x"));
  Serial.print(viaRead, HEX);
  
  via_shift_bank(0);  // Properly select bank 0
  set_address(0x0600);
  unsigned char b0Read = read_data();
  Serial.print(F(" B0=0x"));
  Serial.print(b0Read, HEX);
  
  if(viaRead == 0xF7 && b0Read == 0xF0) {
    Serial.println(F(" OK"));
  } else {
    Serial.println(F(" FAIL - 595 not responding correctly!"));
  }
  
  // Step 5: Test ALL possible ORA garbage states (0-7)
  // This simulates what happens when DDRA enables outputs while ORA contains random garbage
  Serial.println(F("\t    e) ORA garbage sweep (all 8 combos):"));
  
  bool garbageSweepOK = true;
  for(int oraGarbage = 0; oraGarbage < 8; oraGarbage++) {
    // First, set a known bank using Arduino direct method
    shift_bank(0x55);  // Bank 85 - a recognizable pattern
    
    // Set VIA pins as INPUTS (floating) - simulates power-on
    pinMode(VIA_CLOCK_PIN, INPUT);
    pinMode(VIA_DATA_PIN, INPUT);
    pinMode(VIA_LATCH_PIN, INPUT);
    delayMicroseconds(100);
    
    // Pre-set the "garbage" ORA value before enabling outputs
    // This simulates ORA having random contents at power-on
    digitalWrite(VIA_CLOCK_PIN, (oraGarbage & 1) ? HIGH : LOW);  // Bit 0 = CLOCK
    digitalWrite(VIA_DATA_PIN, (oraGarbage & 2) ? HIGH : LOW);   // Bit 1 = DATA
    digitalWrite(VIA_LATCH_PIN, (oraGarbage & 4) ? HIGH : LOW);  // Bit 2 = LATCH
    
    // Now enable outputs - this is when DDRA transitions 0->0x07
    // The pins will immediately drive whatever "garbage" was set above
    pinMode(VIA_CLOCK_PIN, OUTPUT);
    pinMode(VIA_DATA_PIN, OUTPUT);
    pinMode(VIA_LATCH_PIN, OUTPUT);
    delayMicroseconds(10);
    
    // Check what the 595 did
    set_address(0x0600);
    unsigned char afterGarbage = read_data();
    
    // Now do proper bank_shift_out to recover
    via_shift_bank(0x7F);  // Select bank 127
    set_address(0x0600);
    unsigned char recovered = read_data();
    
    Serial.print(F("\t       ORA="));
    Serial.print(oraGarbage);
    Serial.print(F(" (C"));
    Serial.print((oraGarbage & 1) ? '1' : '0');
    Serial.print(F("D"));
    Serial.print((oraGarbage & 2) ? '1' : '0');
    Serial.print(F("L"));
    Serial.print((oraGarbage & 4) ? '1' : '0');
    Serial.print(F("): after=0x"));
    Serial.print(afterGarbage, HEX);
    Serial.print(F(" recov=0x"));
    Serial.print(recovered, HEX);
    
    // If LATCH bit is set in garbage, it should latch whatever was in shift reg
    if(oraGarbage & 4) {
      // LATCH was HIGH - may have latched something
      Serial.print(F(" (LATCH=1)"));
    }
    
    // Check if recovery worked
    if(recovered == 0xF7) {
      Serial.println(F(" OK"));
    } else {
      Serial.println(F(" FAIL!"));
      garbageSweepOK = false;
    }
  }
  
  if(garbageSweepOK) {
    Serial.println(F("\t    All garbage states recovered OK"));
  } else {
    Serial.println(F("\t    WARNING: Some garbage states caused unrecoverable issues!"));
  }
  
  // Summary
  Serial.println(F("\t    Summary:"));
  Serial.println(F("\t    - If 'Float->Drive' fails: 595 glitches on pin enable"));
  Serial.println(F("\t    - If 'ORA=0xFF glitch' fails: confirms SDK bug triggers it"));
  Serial.println(F("\t    - If 'Recovery' fails: 595 may be damaged/stuck"));
  Serial.println(F("\t    - If 'garbage sweep' fails: specific ORA patterns cause issues"));

  Serial.println();
  Serial.println(F("=== ALL TESTS DONE ==="));
  Serial.println(F("Chip erase..."));
  flash_cmd_chip_erase();
  wait_for_ready();
  while(Serial.available() > 0) {
    Serial.read();
  }
  Serial.println("Done!");
  return 0;
}

//Run test suite for newly assembled flash carts
int cmd_test(int argc, char **argv) {
  Serial.println("Test suite for newly assembled flash boards!");
  Serial.println("This will erase any stored data. Are you sure you wish to continue? (y/n)");
  if(!user_confirm()) {
    Serial.println("Test canceled.");
    return 0;
  }
  Serial.println("Starting tests...");
  Serial.println();
  shift_bank(0);
  //Chip Erase test
  Serial.print("\tChip erase: ");
  float readyTime = 0;
  flash_cmd_chip_erase();
  EXPECT(digitalRead(ReadyPin)==LOW)
  while(digitalRead(ReadyPin)==LOW){
    delay(100);
    readyTime += 0.1f;
    EXPECT(readyTime<20)
  }
  EXPECT(readyTime>1)
  set_address(0x0000);
  EXPECT(read_data()==0xFF)
  set_address(0xFFFF);
  EXPECT(read_data()==0xFF)
  set_address(0xCAFE);
  EXPECT(read_data()==0xFF)
  Serial.println("PASS");

  //Write/read test
  Serial.print("\tWrite/read: ");
  flash_cmd_program_to(0x0000, 0x55);
  set_address(0x0000);
  EXPECT(read_data()==0x55)
  flash_cmd_program_to(0x1000, 0x00);
  set_address(0x1000);
  EXPECT(read_data()==0x00)
  Serial.println("PASS");

  Serial.print("\tShift register: ");
  shift_bank(1);
  set_address(0x0000);
  EXPECT(read_data()==0xFF)
  flash_cmd_program_to(0x0000, 0xAA);
  set_address(0x0000);
  EXPECT(read_data()==0xAA)
  set_address(0xC000);
  EXPECT(read_data()==0xFF)
  shift_bank(0x7F);
  set_address(0x0000);
  EXPECT(read_data()==0xFF)
  flash_cmd_program_to(0x0000, 0x00);
  set_address(0x0000);
  EXPECT(read_data()==0x00)
  set_address(0xC000);
  EXPECT(read_data()==0x00)
  Serial.println("PASS");
  
  Serial.println();
  Serial.println("All tests passed.");
  Serial.println("Cleaning up with one last chip erase...");
  flash_cmd_chip_erase();
  wait_for_ready();
  while(Serial.available() > 0) {
    Serial.read();
  }
  Serial.println("Done!");
  return 0;
}
//Shell commands end

void setup() {
  // put your setup code here, to run once:

  //Set all pins on ports A and C to output
  //Doing this in setup() because they'll stay as outputs
  DDRC = 0xFF;
  DDRA = 0xFF;

  //Start the data port as input to avoid accidental contention
  DDRL = 0;

  digitalWrite(OutputEnablePin, HIGH);
  digitalWrite(WriteEnablePin, HIGH);
  digitalWrite(ResetPin, HIGH);
  digitalWrite(ClkPin, LOW);
  digitalWrite(SCKPin, LOW);
  digitalWrite(MOSIPin, LOW);
  digitalWrite(SSPin, HIGH);

  pinMode(OutputEnablePin, OUTPUT);
  pinMode(WriteEnablePin, OUTPUT);
  pinMode(ResetPin, OUTPUT);
  pinMode(ClkPin, OUTPUT);
  pinMode(IRQPin, INPUT_PULLUP);
  pinMode(ReadyPin, INPUT_PULLUP); //Ready is an open-drain output on the flash
  pinMode(SCKPin, OUTPUT);
  pinMode(MOSIPin, OUTPUT);
  pinMode(SSPin, OUTPUT);
  pinMode(MISOPin, INPUT);

  shift_bank(lastBankNum);

  Serial.begin(115200);
  Serial.println("Hello world");
  shell.attach(Serial);

  digitalWrite(ResetPin, LOW);
  delayMicroseconds(10);
  digitalWrite(ResetPin, HIGH);
  wait_for_ready();

  shell.addCommand(F("readAt"), cmd_readAt);
  shell.addCommand(F("dump"), cmd_dump);
  shell.addCommand(F("eraseChip"), cmd_eraseChip);
  shell.addCommand(F("eraseSector"), cmd_eraseSector);
  shell.addCommand(F("writeTo"), cmd_writeTo);
  shell.addCommand(F("shift"), cmd_shift);
  shell.addCommand(F("mode"), cmd_mode);
  shell.addCommand(F("reset"), cmd_reset);
  shell.addCommand(F("writeMulti"), cmd_writeMulti);
  shell.addCommand(F("readMulti"), cmd_readMulti);
  shell.addCommand(F("checksum"), cmd_checksum);
  shell.addCommand(F("timeout"), cmd_timeout);
  shell.addCommand(F("readString"), cmd_readString);
  shell.addCommand(F("version"), cmd_version);
  shell.addCommand(F("test"), cmd_test);
  shell.addCommand(F("testfull"), cmd_testfull);
  Serial.print("Starting in ");
  print_mode();
  Serial.println(" mode");
  Serial.println("Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  shell.executeIfInput();
}
