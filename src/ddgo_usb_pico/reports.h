#pragma once

typedef union {
  struct TU_ATTR_PACKED {
    uint8_t b : 1;
    uint8_t a : 1;
    uint8_t c : 1;
    uint8_t d : 1;
    uint8_t select : 1;
    uint8_t start : 1;
    uint8_t : 1;
    uint8_t : 1;
  };
  uint8_t bButtons;
} densha_usb_buttons_t;

// generic report
typedef struct TU_ATTR_PACKED {
  uint8_t brake;
  uint8_t power;
  uint8_t hat; // not used
  densha_usb_buttons_t buttons;
} generic_report_t;

// One handle controller (PC)
typedef struct TU_ATTR_PACKED {
  uint8_t brake;
  uint8_t power;
  uint8_t hat;
  densha_usb_buttons_t buttons;
  uint8_t : 8;
  uint8_t : 8;
} onehandlepc_report_t;

// Two handle controller (PC)
typedef struct TU_ATTR_PACKED {
  uint8_t brake;
  uint8_t power;
  uint8_t : 8;
  densha_usb_buttons_t buttons;
  uint8_t : 8;
  uint8_t : 8;
} twohandlepc_report_t;

//One handle controller (SWITCH)
typedef struct TU_ATTR_PACKED {
  union {
   struct TU_ATTR_PACKED {
    uint8_t y : 1;
    uint8_t b : 1;
    uint8_t a : 1;
    uint8_t x : 1;
    uint8_t l : 1;
    uint8_t r : 1;
    uint8_t zl : 1;
    uint8_t zr : 1;
    uint8_t select : 1;
    uint8_t start  : 1;
    uint8_t l3 : 1;
    uint8_t r3 : 1;
    uint8_t home : 1;
    uint8_t capture : 1;
    uint8_t : 2;
   };
   uint16_t buttons;
  };

  uint8_t hat : 4;
  uint8_t : 4; 

  uint8_t lx; // x
  uint8_t ly; // y
  uint8_t rx; // z
  uint8_t ry; // rz
  uint8_t : 8; // 0x00 vendor
} onehandleswitch_report_t;
