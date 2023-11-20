/**
   Basic VNC Server for ESP32 with OLED Display SSD1306
  
  The MIT License (MIT)
    Copyright (c) 2023 by Hackerspace FFM, Lutz Lisseck
*/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <MyCreds.h>      // Define WIFI_SSID and WIFI_PASSWORD here
#include "SSD1306Wire.h"  // ESP8266 and ESP32 OLED driver for SSD1306 displays by ThingPulse

const char* hostname = "esp32-vnc";

#define LED 16
SSD1306Wire display(0x3c, 5, 4);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

WiFiServer server(5900);

#define HACKFFMLOGO_width 17
#define HACKFFMLOGO_height 24
const uint8_t HACKFFMLOGO_bits[] = {
  0x01, 0x00, 0x01, 0x05, 0x40, 0x01, 0x15, 0x50, 0x01, 0x55, 0x55, 0x01, 
  0x15, 0x50, 0x01, 0x05, 0x41, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 
  0x00, 0x01, 0x00, 0x60, 0x0C, 0x00, 0xF0, 0x1F, 0x00, 0x08, 0x20, 0x00, 
  0x4A, 0xA4, 0x00, 0x0B, 0xA0, 0x01, 0xF7, 0xDF, 0x01, 0xEB, 0xAF, 0x01, 
  0x13, 0x90, 0x01, 0xF0, 0x1F, 0x00, 0xE3, 0x8F, 0x01, 0x73, 0x9C, 0x01, 
  0x30, 0x18, 0x00, 0x48, 0x24, 0x00, 0x78, 0x3C, 0x00, 0x78, 0x3C, 0x00, 
 };

struct pixel_format_s {
  uint8_t bitsperpixel;
  uint8_t depth;
  uint8_t bigendianflag;
  uint8_t truecolorflag;
  uint16_t redmax;
  uint16_t greenmax;
  uint16_t bluemax;
  uint8_t redshift;
  uint8_t greenshift;
  uint8_t blueshift;
} pixelformat;

struct frame_buffer_update_request_s {
  uint8_t incremental;
  uint16_t xposition;
  uint16_t yposition;
  uint16_t width;
  uint16_t height;
} fbur;

struct key_event_s {
  uint8_t downflag;
  uint32_t key;
} keyevent;

struct pointer_event_s {
  uint8_t buttonmask;
  uint16_t xposition;
  uint16_t yposition;
} pointerevent;

uint8_t vncfbuf[128*64*4+32]; // Buffer for bytes to send over TCP to VNC Client
uint32_t FrameBufferUpdate(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint32_t len = 0;
 
  x = constrain(x, 0, display.width() - 1);
  y = constrain(y, 0, display.height() - 1);
  uint16_t endx = w - x; 
  uint16_t endy = h - y;

  vncfbuf[0] = 0;
  vncfbuf[1] = 0;
  vncfbuf[2] = 0;
  vncfbuf[3] = 1; // 1 rectangle

  vncfbuf[4] = x>>8; vncfbuf[5] = x & 0xff;
  vncfbuf[6] = y>>8; vncfbuf[7] = y & 0xff;

  vncfbuf[8] = w>>8; vncfbuf[9] = w & 0xff;
  vncfbuf[10] = h>>8; vncfbuf[11] = h & 0xff;

  vncfbuf[12] = 0; vncfbuf[13] = 0; vncfbuf[14] = 0; vncfbuf[15] = 0; // raw encoding

  if(pixelformat.bitsperpixel == 8) {
    len = 16;
    uint8_t setpix = (pixelformat.redmax << pixelformat.redshift) |
       (pixelformat.greenmax << pixelformat.greenshift) |
       (pixelformat.bluemax << pixelformat.blueshift);
    for(uint16_t iy = y; iy < endy; iy++) {
      uint16_t buf_y = (iy / 8) * display.width();
      uint8_t  mask_y = (1 << (iy & 7));
      for(uint16_t ix = x; ix < endx; ix++) {
        if(display.buffer[buf_y + ix] & mask_y) {
          vncfbuf[len++] = setpix;
        } else {
          vncfbuf[len++] = 0;
        }
      }
    }  
  } else if(pixelformat.bitsperpixel == 16) {
    len = 16;
    uint16_t setpix = (pixelformat.redmax << pixelformat.redshift) |
       (pixelformat.greenmax << pixelformat.greenshift) |
       (pixelformat.bluemax << pixelformat.blueshift);   
    uint8_t setpix0, setpix1;
    if(pixelformat.bigendianflag) {
      setpix0 = setpix >> 8;
      setpix1 = setpix & 0xff;
    } else {
      setpix1 = setpix >> 8;
      setpix0 = setpix & 0xff;     
    }
    for(uint16_t iy = y; iy < endy; iy++) {
      uint16_t buf_y = (iy / 8) * display.width();
      uint8_t  mask_y = (1 << (iy & 7));
      for(uint16_t ix = x; ix < endx; ix++) {
        if(display.buffer[buf_y + ix] & mask_y) {
          vncfbuf[len++] = setpix0;
          vncfbuf[len++] = setpix1;
        } else {
          vncfbuf[len++] = 0;
          vncfbuf[len++] = 0;
        }
      }
    }     
  } else if(pixelformat.bitsperpixel == 32) {
    len = 16;
    uint32_t setpix = ((uint32_t)pixelformat.redmax << pixelformat.redshift) |
       ((uint32_t)pixelformat.greenmax << pixelformat.greenshift) |
       ((uint32_t)pixelformat.bluemax << pixelformat.blueshift);   
    uint8_t setpix0, setpix1, setpix2, setpix3;
    if(pixelformat.bigendianflag) {
      setpix0 = setpix >> 24;
      setpix1 = setpix >> 16;
      setpix2 = setpix >> 8;
      setpix3 = setpix;
    } else {
      setpix3 = setpix >> 24;
      setpix2 = setpix >> 16;
      setpix1 = setpix >> 8;
      setpix0 = setpix;    
    }
    for(uint16_t iy = y; iy < endy; iy++) {
      uint16_t buf_y = (iy / 8) * display.width();
      uint8_t  mask_y = (1 << (iy & 7));
      for(uint16_t ix = x; ix < endx; ix++) {
        if(display.buffer[buf_y + ix] & mask_y) {
          vncfbuf[len++] = setpix0;
          vncfbuf[len++] = setpix1;
          vncfbuf[len++] = setpix2;
          vncfbuf[len++] = setpix3;
        } else {
          vncfbuf[len++] = 0;
          vncfbuf[len++] = 0;
          vncfbuf[len++] = 0;
          vncfbuf[len++] = 0;
        }
      }
    }     
  } else {
    Serial.println("Pixelformat.bitsperpixel not supported.\r\n");
    len = 0;
  }
  return(len);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booted.");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  display.init();
  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  //display.setFont(ArialMT_Plain_16);

  display.clear();
  display.drawString(10,0,"Wifi connecting...\r\n");
  display.display();

  Serial.println("\r\nStart-up...");
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Try to connect to a given WiFi network
  int tocount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  display.clear();
  display.drawString(0,0,WiFi.localIP().toString().c_str());
  display.drawXbm(34, 14, HACKFFMLOGO_width, HACKFFMLOGO_height, HACKFFMLOGO_bits);
  display.display();
  MDNS.begin(hostname);
  MDNS.addService("vnc", "tcp", 5900);

  server.begin();
}


uint32_t prevMillis = 0;

char buf[256];

void loop(void)
{
    display.setColor(BLACK); display.fillRect(110,0,10,10); display.setColor(WHITE);
    display.drawStringf(110,0,buf,"%d",((millis()/100)%10));
    display.display();

    // Check if a client has connected
    WiFiClient client = server.available();
    if (!client) {
        return;
    }
    Serial.println("");
    Serial.println("New client");

    client.print("RFB 003.008\n");

    // Wait for data from client to become available
    while(client.connected() && !client.available()){
        delay(1);
    }

    // Read the first line of HTTP request
    String req = client.readStringUntil('\r');
    Serial.print("Request: ");
    Serial.println(req);

    if(req.startsWith("RFB ")) {
      uint8_t data[128]; uint32_t send_len = 0;
      Serial.println("RFB found.");
      data[0] = 1; data[1] = 1;
      client.write(data, 2); 
      while(client.connected() && !client.available()){
          delay(1);
      }
      int len = 0;
      
      len = client.readBytes(data, 16);
      Serial.printf("Answer1 len: %d, val0 = %d\r\n",len,data[0]);

      if((len == 1) && (data[0]==1)) {
        data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 0;
        client.write(data, 4);
        Serial.println("OK, start init.");
        while(client.connected() && !client.available()){
          delay(10);
        }
        len = client.readBytes(data, 128);
        Serial.printf("Answer2 len: %d, val0 = %d\r\n",len,data[0]);   
        if(len == 1) {
          uint8_t svinit[] = {
            0,128, 0,64, // size X x Y
             32,24, 1,1, // 32 bits per pixel, but 24 bits
             0,255, 0,255, 0,255, // max 0...255
             16,8,0, 0,0,0,       // shift 16,8,0
            0,0,0,8, 
            'E','S','P','-', 'V','N','C',0 
          };
          client.write(svinit, 32);
          while(client.connected()){
            delay(10);
            if(client.available()) {
              uint16_t count16;
              data[0] = client.read();
              switch(data[0]) {
                case 0: // SetPixelFormat
                  len = client.readBytes(&data[1], 3+16);
                  if(len == 19) {
                    Serial.println("SetPixelFormat:");
                    //for(int i=4; i<3+16; i++) Serial.printf("%02x ", data[i]);
                    pixelformat.bitsperpixel = data[4];
                    pixelformat.depth = data[5];
                    pixelformat.bigendianflag = data[6];
                    pixelformat.truecolorflag = data[7];
                    pixelformat.redmax   = ((uint16_t)data[8]<<8) | (uint16_t)data[9];
                    pixelformat.greenmax = ((uint16_t)data[10]<<8) | (uint16_t)data[11];
                    pixelformat.bluemax  = ((uint16_t)data[12]<<8) | (uint16_t)data[13];
                    pixelformat.redshift = data[14];
                    pixelformat.greenshift = data[15];
                    pixelformat.blueshift = data[16];
                    Serial.printf(" BitPerPixel: %d, Depth: %d, BigEndian: %d, TrueColor: %d\r\n",
                      (int)pixelformat.bitsperpixel, (int)pixelformat.depth, 
                      (int)pixelformat.bigendianflag, (int)pixelformat.truecolorflag);
                    Serial.printf(" RedMax: %d, GreenMax: %d, BlueMax: %d\r\n",
                      (int)pixelformat.redmax, (int)pixelformat.greenmax, (int)pixelformat.bluemax); 
                    Serial.printf(" RedShift: %d, GreenShift: %d, BlueShift: %d\r\n",
                      (int)pixelformat.redshift, (int)pixelformat.greenshift, (int)pixelformat.blueshift);
                  } else {
                    Serial.println("Invalid SetPixelFormat.");
                  }
                  break;

                case 2: // SetEncodings
                  len = client.readBytes(&data[1], 3);
                  count16 = data[2]<<8 | data[3];
                  if((count16 > 0) && (len == 3)) {
                    Serial.print("SetEncodings:\r\n ");
                    for(int i=0; i<count16; i++) {
                      len = client.readBytes(data, 4);
                      uint32_t data32 = ((uint32_t)data[0]<<24) | ((uint32_t)data[1]<<16) 
                        | ((uint32_t)data[2]<<8) | ((uint32_t)data[3]);               
                      Serial.printf("%d ", data32);
                    }
                    Serial.println(" ");
                  } else {
                    Serial.println("Invalid SetEncodings.");
                  }
                  break;    

                case 3: // FramebufferUpdateRequest
                  len = client.readBytes(&data[1], 9);
                  if(len == 9) {
                    Serial.print("FramebufferUpdateRequest:\r\n ");
                    //for(int i=1; i<=9; i++) Serial.printf("%03d ", data[i]);
                    fbur.incremental = data[1];
                    fbur.xposition = ((uint16_t)data[2]<<8) | (uint16_t)data[3];
                    fbur.yposition = ((uint16_t)data[4]<<8) | (uint16_t)data[5];
                    fbur.width = ((uint16_t)data[6]<<8) | (uint16_t)data[7];
                    fbur.height = ((uint16_t)data[8]<<8) | (uint16_t)data[9];
                    Serial.printf(" Incremental: %d, X: %d, Y: %d, Width: %d, Height: %d\r\n",
                      (int)fbur.incremental, (int)fbur.xposition, (int)fbur.yposition,
                      (int)fbur.width,(int)fbur.height);
                    if(send_len = FrameBufferUpdate(0,0,128,64)) {
                      client.write(vncfbuf, send_len);
                    }
                  } else {
                    Serial.println("Invalid FramebufferUpdateRequest.");
                  }
                  break;  

                case 4: // KeyEvent
                  len = client.readBytes(&data[1], 7);
                  if(len == 7) {
                    Serial.print("KeyEvent: ");
                    //for(int i=1; i<=7; i++) Serial.printf("%02x ", data[i]);
                    keyevent.downflag = data[1];
                    keyevent.key = ((uint32_t)data[4]<<24) | ((uint32_t)data[5]<<16) 
                        | ((uint32_t)data[6]<<8) | ((uint32_t)data[7]);
                    Serial.printf("Downflag: %d, Key: %04x\r\n", (int)keyevent.downflag, keyevent.key);
                  } else {
                    Serial.println("Invalid KeyEvent.");
                  }
                  break;   

                case 5: // PointerEvent
                  len = client.readBytes(&data[1], 5);
                  if(len == 5) {
                    Serial.print("PointerEvent: ");
                    //for(int i=1; i<=5; i++) Serial.printf("%02x ", data[i]);
                    pointerevent.buttonmask = data[1];
                    pointerevent.xposition = ((uint16_t)data[2]<<8) | (uint16_t)data[3];
                    pointerevent.yposition = ((uint16_t)data[4]<<8) | (uint16_t)data[5];
                    Serial.printf("Button: %02x, (X:%d, Y:%d)\r\n", (int)pointerevent.buttonmask,
                      (int)pointerevent.xposition, (int)pointerevent.yposition);
                    if(pointerevent.buttonmask & 1) {
                      display.setPixel(pointerevent.xposition, pointerevent.yposition);
                    } else {
                      display.clearPixel(pointerevent.xposition, pointerevent.yposition);
                    } 
                    display.display();  
                  } else {
                    Serial.println("Invalid PointerEvent.");
                  }
                  break;   

                case 6: // ClientCutText
                  len = client.readBytes(&data[1], 7);
                  if(len == 7) {
                    Serial.print("ClientCutText:\r\n ");
                    for(int i=4; i<5; i++) Serial.printf("%02x ", data[i]);
                    Serial.println(" ");
                  } else {
                    Serial.println("Invalid ClientCutText.");
                  }
                  break;                                                            
              }
              
              // Serial.printf("Quest len: %d, val0 = %d\r\n",len,data[0]); 
            }
          }
         
        }     
      }
    
    }

    client.stop();
    Serial.println("Done with client");
}