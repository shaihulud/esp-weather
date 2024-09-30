#define humi_width 60
#define humi_height 60
PROGMEM const unsigned char humi_logo[] = {
  0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x01, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x8C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 
  0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x07, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x1C, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x80, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 
  0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x30, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xE0, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 
  0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x30, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 
  0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x80, 0x03, 0x00, 0x00, 
  0x00, 0x00, 0x1C, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 
  0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x06, 0x00, 0x00, 
  0x00, 0x00, 0x07, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 
  0x00, 0x0C, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x1C, 0x00, 0x00, 
  0x00, 0x80, 0x01, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xC0, 0xC1, 0x0F, 
  0xE0, 0x38, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0x1F, 0x60, 0x30, 0x00, 0x00, 
  0x00, 0xE0, 0x60, 0x18, 0x70, 0x70, 0x00, 0x00, 0x00, 0x60, 0x70, 0x38, 
  0x38, 0x60, 0x00, 0x00, 0x00, 0x70, 0x30, 0x30, 0x1C, 0xE0, 0x00, 0x00, 
  0x00, 0x30, 0x70, 0x38, 0x0C, 0xC0, 0x00, 0x00, 0x00, 0x38, 0x60, 0x18, 
  0x0E, 0xC0, 0x01, 0x00, 0x00, 0x18, 0xE0, 0x1F, 0x07, 0x80, 0x01, 0x00, 
  0x00, 0x18, 0x80, 0x8F, 0x03, 0x80, 0x01, 0x00, 0x00, 0x1C, 0x00, 0xC0, 
  0x01, 0x80, 0x03, 0x00, 0x00, 0x08, 0x00, 0xC0, 0x01, 0x00, 0x01, 0x00, 
  0x00, 0x1C, 0x00, 0xE0, 0x00, 0x80, 0x03, 0x00, 0x00, 0x0C, 0x00, 0x70, 
  0x00, 0x00, 0x03, 0x00, 0x00, 0x0C, 0x00, 0x38, 0x00, 0x00, 0x03, 0x00, 
  0x00, 0x0C, 0x00, 0x3C, 0x08, 0x80, 0x03, 0x00, 0x00, 0x18, 0x00, 0x0C, 
  0x3F, 0x00, 0x01, 0x00, 0x00, 0x18, 0x00, 0x8E, 0x7F, 0x80, 0x03, 0x00, 
  0x00, 0x18, 0x00, 0x87, 0x61, 0x80, 0x01, 0x00, 0x00, 0x18, 0x80, 0xC3, 
  0xC0, 0x80, 0x01, 0x00, 0x00, 0x38, 0x80, 0xC3, 0xC0, 0xC0, 0x00, 0x00, 
  0x00, 0x30, 0xC0, 0xC1, 0xC1, 0xC0, 0x00, 0x00, 0x00, 0x70, 0xE0, 0x80, 
  0x73, 0xE0, 0x00, 0x00, 0x00, 0xE0, 0x70, 0x80, 0x7F, 0x70, 0x00, 0x00, 
  0x00, 0xC0, 0x60, 0x00, 0x1E, 0x70, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x00, 
  0x00, 0x3C, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x1C, 0x00, 0x00, 
  0x00, 0x00, 0x0F, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 
  0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0xF0, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0xF0, 0xAF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 
  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x05, 0x00, 0x00, 0x00, 
  };
