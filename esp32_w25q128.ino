/* 
 *  Based on https://github.com/nopnop2002/esp-idf-w25q64
 *  esp-idf reference: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
 *  
 *  W25Q128JVSQ      ESP32 SPI/DSPI    ESP32 QSPI
 *  =============================================
 *  1 CS             GPIO5             GPIO5
 *  2 MISO/IO1       GPIO19            GPIO19
 *  3 WP/IO2         3.3V              GPIO22
 *  4 GND            GND               GND
 *  5 MOSI/IO0       GPIO23            GPIO23
 *  6 SCK|           GPIO18            GPIO18
 *  7 HOLD/IO3       3.3V              GPIO21
 *  8 VCC            3.3V              3.3V
 *  
 *  Note: the GPIO pins are based on ESP32 SPI2 (VSPI) pin configuration
 *  
 * ----------------------------------------------------------------------------------------------------------
 * |                                                     NOR FLASH                                          |
 * ----------------------------------------------------------------------------------------------------------
 * |  Model  | Block |   Sector  |      Page      |             Byte             |            Bit           |
 * | W25Q02  | 4096  | 4096 * 16 | 4096 * 16 * 16 | 4096 * 16 * 16 * 256 (256MB) | 4096 * 16 * 16 * 256 * 8 |
 * | W25Q01  | 2048  | 2048 * 16 | 2048 * 16 * 16 | 2048 * 16 * 16 * 256 (128MB) | 2048 * 16 * 16 * 256 * 8 |
 * | W25Q512 | 1024  | 1024 * 16 | 1024 * 16 * 16 | 1024 * 16 * 16 * 256  (64MB) | 1024 * 16 * 16 * 256 * 8 |
 * | W25Q256 |  512  |  512 * 16 |  512 * 16 * 16 |  512 * 16 * 16 * 256  (32MB) |  512 * 16 * 16 * 256 * 8 |
 * | W25Q128 |  256  |  256 * 16 |  256 * 16 * 16 |  256 * 16 * 16 * 256  (16MB) |  256 * 16 * 16 * 256 * 8 |
 * | W25Q64  |  128  |  128 * 16 |  128 * 16 * 16 |  128 * 16 * 16 * 256   (8MB) |  128 * 16 * 16 * 256 * 8 |
 * | W25Q32  |   64  |   64 * 16 |   64 * 16 * 16 |   64 * 16 * 16 * 256   (4MB) |   64 * 16 * 16 * 256 * 8 |
 * | W25Q16  |   32  |   32 * 16 |   32 * 16 * 16 |   32 * 16 * 16 * 256   (2MB) |   32 * 16 * 16 * 256 * 8 |
 * | W25Q80  |   16  |   16 * 16 |   16 * 16 * 16 |   16 * 16 * 16 * 256   (1MB) |   16 * 16 * 16 * 256 * 8 |
 * | W25Q40  |    8  |    8 * 16 |    8 * 16 * 16 |    8 * 16 * 16 * 256 (512KB) |    8 * 16 * 16 * 256 * 8 |
 * | W25Q20  |    4  |    4 * 16 |    4 * 16 * 16 |    4 * 16 * 16 * 256 (256KB) |    4 * 16 * 16 * 256 * 8 |
 * | W25X10  |    2  |    2 * 16 |    2 * 16 * 16 |    2 * 16 * 16 * 256 (128KB) |    2 * 16 * 16 * 256 * 8 |
 * | W25X05  |    1  |    1 * 16 |    1 * 16 * 16 |    1 * 16 * 16 * 256  (64KB) |    1 * 16 * 16 * 256 * 8 |
 * ----------------------------------------------------------------------------------------------------------
 * Chips greater than W25Q128 use 4Byte address mode, Q25Q128 and lower use 3Byte address mode
 */

#include "w25q128.h"

void dump(uint8_t *dt, int n) {
  
  uint8_t clm = 0;

  Serial.printf("------------------------------------------------------\n");
  for (int addr = 0; addr <= n-1; addr++) {
    uint8_t data = dt[addr];
    if (clm == 0) {
      Serial.printf("%05d: ", addr);
    }
    Serial.printf("%02x ", data);
    if (++clm == 16) {
      Serial.println();
      clm = 0;
    }
  }
  Serial.printf("------------------------------------------------------\n");
  
}

void setup() {

  Serial.begin(115200);
  delay(200);
  Serial.println();
  
  W25Q128_t dev;
  W25Q128_init(&dev);

  esp_err_t ret;

  // Get Status Register1
  uint8_t reg1;
  ret = W25Q128_readStatusReg1(&dev, &reg1);
  if (ret != ESP_OK) {
    Serial.printf("readStatusReg1 fail %d\n",ret);
    while(1) { vTaskDelay(1); }
  } 
  Serial.printf("readStatusReg1 : %x\n", reg1);
  
  // Get Status Register2
  uint8_t reg2;
  ret = W25Q128_readStatusReg2(&dev, &reg2);
  if (ret != ESP_OK) {
    Serial.printf("readStatusReg2 fail %d\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("readStatusReg2 : %x\n", reg2);

  // Get Unique ID
  uint8_t uid[8];
  ret = W25Q128_readUniqieID(&dev, uid);
  if (ret != ESP_OK) {
    Serial.printf("readUniqieID fail %d\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("readUniqieID : %x-%x-%x-%x-%x-%x-%x-%x\n",
     uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]
  );

  // Get JDEEC Info
  uint8_t jid[4];
  ret = W25Q128_readJEDEC(&dev, jid);
  if (ret != ESP_OK) {
    Serial.printf("readJEDEC Code fail %d\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("JEDEC Code %x %x %x\n", jid[0], jid[1], jid[2]);


  // Get Manufacture Code and Chip Capacity
  uint8_t mcode0[3];
  ret = W25Q128_readManufacturer(&dev, STD_IO, mcode0);
  if (ret != ESP_OK) {
    Serial.printf("readManufacturer(SPI) fail %d\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("readManufacturer(SPI): %x %x\n\n", mcode0[0], mcode0[1]);

//  // Get Manufacture Code and Chip Capacity with DSPI
//  uint8_t mcode1[3];
//  ret = W25Q128_readManufacturer(&dev, DUAL_IO, mcode1);
//  if (ret != ESP_OK) {
//    Serial.printf("readManufacturer(DSPI) fail %d\n", ret);
//    while(1) { vTaskDelay(1); }
//  }
//  Serial.printf("readManufacturer(DSPI): %x %x\n", mcode1[0], mcode1[1]);
  
//  // Get Manufacture Code and Chip Capacity with QSPI
//  uint8_t mcode2[3];
//  ret = W25Q128_readManufacturer(&dev, QUAD_IO, mcode2);
//  if (ret != ESP_OK) {
//    Serial.printf("readManufacturer(QSPI) fail %d\n", ret);
//    while(1) { vTaskDelay(1); }
//  }
//  Serial.printf("readManufacturer(QSPI): %x %x\n", mcode2[0], mcode2[1]);


  // Read 256 byte data from Address=0
  uint8_t rbuf[256];
  int len;
  memset(rbuf, 0, 256);
  len =  W25Q128_fastread(&dev, 0, rbuf, 256);
  if (len != 256) {
    Serial.println("fastread fail");
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("Fast Read Data: len=%d\n", len);
  dump(rbuf, 256);
  
  // Erase data by Sector
  bool flag = W25Q128_eraseSector(&dev, 0, true);
  if (flag == false) {
    Serial.printf("eraseSector fail %d\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.println("Erase Sector 0");
  
  memset(rbuf, 0, 256);
  len =  W25Q128_read(&dev, 0, rbuf, 256);
  if (len != 256) {
    Serial.println("read fail");
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("Read Data: len=%d\n", len);
  dump(rbuf, 256);

  // Write data to Sector=0 Address=10
  uint8_t wdata[26];
  for (int i=0; i<26; i++) {
    wdata[i]='A'+ i; // A-Z     
  }  
  len =  W25Q128_pageWrite(&dev, 0, 10, wdata, 26);
  if (len != 26) {
    Serial.println("pageWrite fail");
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("Page Write(Sector=0 Address=10) len=%d\n", len);

  // First read 256 byte data from Address=0
  memset(rbuf, 0, 256);
  len =  W25Q128_fastread(&dev, 0, rbuf, 256);
  if (len != 256) {
    Serial.println("fastread fail");
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("Fast Read Data: len=%d\n", len);
  dump(rbuf, 256);

  // Write data to Sector=0 Address=0
  for (int i=0; i < 10;i++) {
    wdata[i]='0'+i; // 0-9     
  }  
  len =  W25Q128_pageWrite(&dev, 0, 0, wdata, 10);
  if (len != 10) {
    Serial.println("pageWrite fail");
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("Page Write(Sector=0 Address=0) len=%d\n", len);

  // First read 256 byte data from Address=0
  memset(rbuf, 0, 256);
  len =  W25Q128_fastread(&dev, 0, rbuf, 256);
  if (len != 256) {
    Serial.println("fastread fail");
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("Fast Read Data: len=%d\n", len);
  dump(rbuf, 256);

  Serial.println("Success All Test");

}

void loop() {

}
