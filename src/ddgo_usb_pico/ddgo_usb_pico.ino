/*********************************************************************
 Densha De Go USB Adapter
 by Matheus Fraguas (sonik-br)
 https://github.com/sonik-br/ddgo_usb_adapt

 For using on a RP2040 board

 Requires the arduino pico board definition from:
 https://github.com/earlephilhower/arduino-pico

 Requires Pico-PIO-USB lib
 https://github.com/sekigon-gonnoc/Pico-PIO-USB

 Can use tusb_xinput for xinput support.
 If not using, remove the callbacks.
 https://github.com/Ryzee119/tusb_xinput

 Special thanks to Marc Riera 
 https://marcriera.github.io/ddgo-controller-docs/
 
*********************************************************************/

// pio-usb is required for rp2040 host
#include "pio_usb.h"
#define HOST_PIN_DP   3 //2   // Pin used as D+ for host, D- = D+ + 1

#include "Adafruit_TinyUSB.h"
//#include "pico/stdlib.h"
#include "driver_densha_host.h"
#include "usb_descriptors.h"
#include "reports.h"

#include "usb_host.h"

// USB Host object
Adafruit_USBH_Host USBHost;

enum OutputType {
  OUTPUT_PC_GENERIC,        //                                                     NOT READY
  OUTPUT_PC_ONE_HANDLE,     // One handle controller (PC) DGC-255
  OUTPUT_PC_TWO_HANDLE,     // Two handle controller (PC) DGOC-44U
  OUTPUT_PC_RYOJOUHEN,      // Ryojōhen controller (PC) DYC-288 / DRC-184          NOT READY
  OUTPUT_SWITCH_ONE_HANDLE, // One handle controller (Nintendo Switch) ZKNS-001
  OUTPUT_HID_CLASSIC        // Classic controller map (layout of PS1 digital pad)  NOT READY
};

// To do:
// implement hid input for ps3 controller?
// PS2_TWO_HANDLE to PC_TWO_HANDLE, output dpad as select+button?
// Implement the full required Nintendo Switch descriptor
// Change Hori to Zuki 0x33DD 0x0002 "One Handle MasCon for Nintendo Switch Exclusive Edition" ZKNS-002

OutputType outputType = OUTPUT_PC_TWO_HANDLE;
bool outputClassicPS1 = true; //Forces UP and DOWN as pressed when in Classic mode.

//Classic controller definitions
#define C_A       0x0001 //SQUARE
#define C_B       0x0002 //CROSS
#define C_C       0x0004 //CIRCLE
#define C_POWER_1 0x0008 //TRIANGLE
#define C_POWER_2 0x0010 //LEFT
#define C_POWER_3 0x0020 //RIGHT
#define C_UP      0x0040 //UP
#define C_DOWN    0x0080 //DOWN
#define C_BRAKE_1 0x0100 //L1
#define C_BRAKE_2 0x0200 //L2
#define C_BRAKE_3 0x0400 //R1
#define C_BRAKE_4 0x0800 //R2
#define C_SELECT  0x1000 //SELECT
#define C_START   0x2000 //START



// Nintendo Switch handle notches
#define NSWITCH_E  0x00 // emergency
#define NSWITCH_B1 0x65
#define NSWITCH_B2 0x57
#define NSWITCH_B3 0x49
#define NSWITCH_B4 0x3C
#define NSWITCH_B5 0x2E
#define NSWITCH_B6 0x20
#define NSWITCH_B7 0x13
#define NSWITCH_B8 0x05
#define NSWITCH_N  0x80 // neutral
#define NSWITCH_P1 0x9F
#define NSWITCH_P2 0xB7
#define NSWITCH_P3 0xCE
#define NSWITCH_P4 0xE6
#define NSWITCH_P5 0xFF

#define NSWITCH_HAT_RELEAED 0x0F








// default report values
generic_report_t generic_input_report {
  .brake    = 0x00,
  .power    = 0x00,
  .hat      = 0x08,
  .buttons  = 0x00
};

onehandlepc_report_t out_onehandlepc_report {
  .brake    = 0x00,
  .power    = 0x00,
  .hat      = 0x08,
  .buttons  = 0x00
};

twohandlepc_report_t out_twohandlepc_report {
  .brake    = 0x00,
  .power    = 0x00,
  .buttons  = 0x00
};

onehandleswitch_report_t out_onehandleswitch_report {
  .buttons = 0x00,
  .hat    = NSWITCH_HAT_RELEAED,
  .lx     = 0x80,
  .ly     = 0x80,
  .rx     = 0x80,
  .ry     = 0x80
};


// USB HID object. For ESP32 these values cannot be changed after this declaration
Adafruit_USBD_HID usb_hid(NULL, 0, HID_ITF_PROTOCOL_NONE, 1, false);


typedef struct {
  uint8_t address;
  uint8_t instance;
} connected_device_t;

volatile bool connected {0};
volatile connected_device_t connected_device;

volatile bool rumblePowerEnabled {false};
volatile bool rumbleBrakeEnabled {false};
volatile bool lampEnabled {false};

// the setup function runs once when you press reset or power the board
void setup()
{
  const uint16_t TAITO_VID = 0x0AE4;
  const uint16_t HORI_VID = 0x0F0D;
  switch (outputType) {
    case OUTPUT_PC_ONE_HANDLE:
      TinyUSBDevice.setID(TAITO_VID, 0x0003); // same as the two handle?
      TinyUSBDevice.setManufacturerDescriptor("TAITO");
      TinyUSBDevice.setProductDescriptor(usb_onehandlepc_string_product);
      TinyUSBDevice.setSerialDescriptor(usb_onehandlepc_string_serial);
      TinyUSBDevice.setVersion(usb_onehandlepc_bcd_version);
      TinyUSBDevice.setDeviceVersion(usb_onehandlepc_bcd_device_version);
      usb_hid.setReportDescriptor(desc_onehandlepc_hid_report, sizeof(desc_onehandlepc_hid_report));
      break;
    case OUTPUT_PC_TWO_HANDLE:
      TinyUSBDevice.setID(TAITO_VID, 0x0003);
      TinyUSBDevice.setManufacturerDescriptor("TAITO");
      TinyUSBDevice.setProductDescriptor(usb_twohandlepc_string_product);
      TinyUSBDevice.setSerialDescriptor(usb_twohandlepc_string_serial);
      TinyUSBDevice.setVersion(usb_twohandlepc_bcd_version);
      TinyUSBDevice.setDeviceVersion(usb_twohandlepc_bcd_device_version);
      usb_hid.setReportDescriptor(desc_twohandlepc_hid_report, sizeof(desc_twohandlepc_hid_report));
      break;
    case OUTPUT_PC_RYOJOUHEN:
      TinyUSBDevice.setID(TAITO_VID, 0x0008);
      TinyUSBDevice.setManufacturerDescriptor("TAITO");
      TinyUSBDevice.setProductDescriptor("電車でGO! 旅情編 コットローラ"); //this is correct?
      TinyUSBDevice.setSerialDescriptor("TCPP20014"); //this is correct?
//      usb_hid.setReportDescriptor(desc_hid_report2, sizeof(desc_hid_report2));
      break;
    case OUTPUT_SWITCH_ONE_HANDLE:
//      TinyUSBDevice.setID(HORI_VID, 0x00C1);
//      TinyUSBDevice.setManufacturerDescriptor("Hori");
//      TinyUSBDevice.setProductDescriptor("One Handle MasCon for Nintendo Switch");
      //TinyUSBDevice.setSerialDescriptor("ZKNS001");
//      usb_hid.setReportDescriptor(desc_hid_report2, sizeof(desc_hid_report2));


      TinyUSBDevice.setID(HORI_VID, 0x00C1);
      TinyUSBDevice.setManufacturerDescriptor("Hori Co., Ltd"); // should be empty?
      TinyUSBDevice.setProductDescriptor(usb_onehandleswitch_string_product);
      TinyUSBDevice.setSerialDescriptor(usb_onehandleswitch_string_serial);
      TinyUSBDevice.setVersion(usb_onehandleswitch_bcd_version);
      TinyUSBDevice.setDeviceVersion(usb_onehandleswitch_bcd_device_version);
      usb_hid.setReportDescriptor(desc_onehandleswitch_hid_report, sizeof(desc_onehandleswitch_hid_report));

      break;
    case OUTPUT_HID_CLASSIC:
//      usb_hid.setReportDescriptor(desc_hid_report3, sizeof(desc_hid_report3));
      break;
    default:
      return;
  }

  // usb_hid.setPollInterval(1);
  // usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));

  usb_hid.begin();

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
  
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // wait for native usb

  printf("TinyUSB Densha De Go adapter\r\n");


  // USB Host via PIO

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while(1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;
 
 #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46 */
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
 #endif /* ARDUINO_RASPBERRY_PI_PICO_W */
 
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

void loop() {
  static generic_report_t last_report { 0x00 };

  // TinyUSBDevice.task(); // no need to call here. Arduino Core handles this
  USBHost.task();

//  if (Serial.available()) {
//    const char ch = Serial.read();
//    if (ch == 'r') {
//      //send commands to Densha De Go
//      //https://marcriera.github.io/ddgo-controller-docs/controllers/usb/tcpp20009/
//      static bool rumbleEnabled = false;
//      rumbleEnabled = !rumbleEnabled;
////      if(!tuh_densha_set_rumble_power_handle(connected_device.address, instance, rumbleEnabled)) {
////        Serial.println("RUMBLE POWER ERROR\r\n");
////      }
////      if(!tuh_densha_set_rumble_brake_handle(connected_device.address, instance, rumbleEnabled)) {
////        Serial.println("RUMBLE BRAKE ERROR\r\n");
////      }
//      if(!tuh_densha_set_lamp(connected_device.address, connected_device.instance, rumbleEnabled)) {
//        Serial.println("LAMP ERROR\r\n");
//      }
//      return;
//    }
//  }

  if (usb_hid.ready()) {
    // input report was updated?
    bool report_was_updated = false;
    if (memcmp(&last_report, &generic_input_report, sizeof(generic_input_report))) {
      report_was_updated = true;
      memcpy(&last_report, &generic_input_report, sizeof(generic_input_report));
    }
  
    // map generic_output to the output_mode
    if (report_was_updated)
      map_output();
    
    // send hid report to host
    switch (outputType) {
      case OUTPUT_PC_ONE_HANDLE:
        usb_hid.sendReport(0, &out_onehandlepc_report, sizeof(out_onehandlepc_report));
        break;
      case OUTPUT_PC_TWO_HANDLE:
        usb_hid.sendReport(0, &out_twohandlepc_report, sizeof(out_twohandlepc_report));
        break;
      case OUTPUT_SWITCH_ONE_HANDLE:
        usb_hid.sendReport(0, &out_onehandleswitch_report, sizeof(out_onehandleswitch_report));
        break;
    }
  }
  
}


void map_output() {
  switch (outputType) {
    case OUTPUT_PC_ONE_HANDLE:
      out_onehandlepc_report.brake    = generic_input_report.brake;
      out_onehandlepc_report.power    = generic_input_report.power;
      out_onehandlepc_report.hat      = generic_input_report.hat;
      out_onehandlepc_report.buttons  = generic_input_report.buttons;
      break;
    case OUTPUT_PC_TWO_HANDLE:
      out_twohandlepc_report.brake    = generic_input_report.brake;
      out_twohandlepc_report.power    = generic_input_report.power;
      out_twohandlepc_report.buttons  = generic_input_report.buttons;
      break;
    case OUTPUT_SWITCH_ONE_HANDLE:
      //brake takes priority.
      if (generic_input_report.brake != 0x79) {
        switch (generic_input_report.brake) {
          case 0x79: // Released
            out_onehandleswitch_report.ly = NSWITCH_N;
            break;
          case 0x8A: // B1
            out_onehandleswitch_report.ly = NSWITCH_B1;
            break;
          case 0x94: // B2
            out_onehandleswitch_report.ly = NSWITCH_B2;
            break;
          case 0x9A: // B3
            out_onehandleswitch_report.ly = NSWITCH_B3;
            break;
          case 0xA2: // B4
            out_onehandleswitch_report.ly = NSWITCH_B4;
            break;
          case 0xA8: // B5
            out_onehandleswitch_report.ly = NSWITCH_B5;
            break;
          case 0xAF: // B6
            out_onehandleswitch_report.ly = NSWITCH_B6;
            break;
          case 0xB2: // B7
            out_onehandleswitch_report.ly = NSWITCH_B7;
            break;
          case 0xB5: // B8
            out_onehandleswitch_report.ly = NSWITCH_B8;
            break;
          case 0xB9: // Emergency
            out_onehandleswitch_report.ly = NSWITCH_E;
            break;
//          case 0xFF: // Transition
//            break;
        }
      }
      //only set power if brake is released
      if (generic_input_report.brake == 0x79) {
        switch (generic_input_report.power) {
          case 0x81: // N
            out_onehandleswitch_report.ly = NSWITCH_N;
            break;
          case 0x6D: // P1
            out_onehandleswitch_report.ly = NSWITCH_P1;
            break;
          case 0x54: // P2
            out_onehandleswitch_report.ly = NSWITCH_P2;
            break;
          case 0x3F: // P3
            out_onehandleswitch_report.ly = NSWITCH_P3;
            break;
          case 0x21: // P4
            out_onehandleswitch_report.ly = NSWITCH_P4;
            break;
          case 0x00: // P5
            out_onehandleswitch_report.ly = NSWITCH_P5;
            break;
//          case 0xFF: // Transition
//            break;
        }
      }

      out_onehandleswitch_report.hat    = generic_input_report.hat > 0x07 ? NSWITCH_HAT_RELEAED : generic_input_report.hat;
      out_onehandleswitch_report.b      = generic_input_report.buttons.b;
      out_onehandleswitch_report.a      = generic_input_report.buttons.c;
      out_onehandleswitch_report.y      = generic_input_report.buttons.a;
      out_onehandleswitch_report.x      = generic_input_report.buttons.d;
      out_onehandleswitch_report.select = generic_input_report.buttons.select;
      out_onehandleswitch_report.start  = generic_input_report.buttons.start;
      
      out_onehandleswitch_report.zl = (out_onehandleswitch_report.ly == NSWITCH_E);
      
      break;
  }
}





//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// HID callbacks

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report_desc, uint16_t desc_len) {
  printf("HID mount\r\n");
  tuh_hid_receive_report(dev_addr, idx);
}
//void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
//  printf("HID unmount");
//}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len) {
////  printf("HID report: address %u, index %u\r\n", dev_addr, idx);
//
//  uint16_t vid, pid;
//  tuh_vid_pid_get(dev_addr, &vid, &pid);
//  //printf("VID = %04x, PID = %04x\r\n", vid, pid);
//
//  gp.x = 0x80;
//  gp.y = 0x80;
//  gp.z = 0x80;
//  gp.rz = 0x80;
//  gp.rx = 0x80;
//  gp.ry = 0x80;
//  gp.buttons = 0x00;
//
////dualshock4  .. lx ly rx ry dpbtn btn .. lt rt
////dualsense   .. lx ly rx ry lt rt .. dpbtn btn
//
//  const uint16_t VID_SONY = 0x054C;
//  const uint16_t PID_DUALSHOCK4 = 0x09CC;
//  const uint16_t PID_DUALSENSE  = 0x0CE6;
//
//  if (vid == VID_SONY && (pid == PID_DUALSHOCK4 || pid == PID_DUALSENSE)) {
//    uint8_t index_dpad; //dpad and face buttons
//    uint8_t index_btn;  //more buttons
//    uint8_t index_lt;   //Lt
//    uint8_t index_rt;   //Rt
//
//    if (pid == PID_DUALSHOCK4) {
//      index_dpad = 5;
//      index_btn = 6;
//      index_lt = 8;
//      index_rt = 9;
//    } else { //dualsense
//      index_dpad = 8;
//      index_btn = 9;
//      index_lt = 5;
//      index_rt = 6;
//    }
//    
//    if (report[index_dpad] & 0x20) //cross
//      gp.buttons |= C_B;
//    if (report[index_dpad] & 0x40) //circle
//      gp.buttons |= C_C;
//    if (report[index_dpad] & 0x10) //square
//      gp.buttons |= C_A;
//    if (report[index_dpad] & 0x80) //triangle
//      gp.buttons |= C_POWER_1;
//
//    if ((report[index_dpad] & 0x0F) == 0x00) //up
//      gp.buttons |= C_UP;
//    if ((report[index_dpad] & 0x0F) == 0x04) //down
//      gp.buttons |= C_DOWN;
//    if ((report[index_dpad] & 0x0F) == 0x06) //left
//      gp.buttons |= C_POWER_2;
//    if ((report[index_dpad] & 0x0F) == 0x02) //right
//      gp.buttons |= C_POWER_3;
//
//    if (report[index_btn] & 0x01) //L1
//      gp.buttons |= C_BRAKE_1;
//    if (report[index_btn] & 0x02) //R1
//      gp.buttons |= C_BRAKE_3;
//
//    if (report[index_lt] > 128) //L2
//      gp.buttons |= C_BRAKE_2;
//    if (report[index_rt] > 128) //R2
//      gp.buttons |= C_BRAKE_4;
//
//    if (report[index_btn] & 0x10)
//      gp.buttons |= C_SELECT;
//    if (report[index_btn] & 0x20)
//      gp.buttons |= C_START;
//
//    usb_hid.sendReport(0, &gp, sizeof(gp));
//
////    for (uint16_t i = 0; i < len; ++i)
////      printf("0x%x, ", report[i]);
////    printf("\r\n");
////    delay(100);
//  }
//
//  tuh_hid_receive_report(dev_addr, idx);
}

// XINPUT callbacks

//void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t *xinput_itf)
//{
////    printf("XINPUT MOUNTED %02x %d\n", dev_addr, instance);
//    // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
//    // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
//    if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false)
//    {
//        tuh_xinput_receive_report(dev_addr, instance);
//        return;
//    }
//    tuh_xinput_set_led(dev_addr, instance, 0, true);
//    tuh_xinput_set_led(dev_addr, instance, 1, true);
//    tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
//    tuh_xinput_receive_report(dev_addr, instance);
//}
//
//void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
////  printf("XINPUT RECEIVED %02x %d\n", dev_addr, instance);
//  if (outputType == OUTPUT_HID_CLASSIC) {
//    xinputh_interface_t *xid_itf = (xinputh_interface_t *)report;
//    xinput_gamepad_t *p = &xid_itf->pad;
//  
//    if (xid_itf->connected && xid_itf->new_pad_data && usb_hid.ready())
//    {
//      TU_LOG1("[%02x, %02x], Type: %s, Buttons %04x, LT: %02x RT: %02x, LX: %d, LY: %d, RX: %d, RY: %d\n",
//           dev_addr, instance, type_str, p->wButtons, p->bLeftTrigger, p->bRightTrigger, p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);
//
//      gp.x = 0x80;
//      gp.y = 0x80;
//      gp.z = 0x80;
//      gp.rz = 0x80;
//      gp.rx = 0x80;
//      gp.ry = 0x80;
//      gp.buttons = 0x00;
//
//      if (p->wButtons & XINPUT_GAMEPAD_A) //cross
//        gp.buttons |= C_B;
//      if (p->wButtons & XINPUT_GAMEPAD_B) //circle
//        gp.buttons |= C_C;
//      if (p->wButtons & XINPUT_GAMEPAD_X) //square
//        gp.buttons |= C_A;
//      if (p->wButtons & XINPUT_GAMEPAD_Y) //triangle
//        gp.buttons |= C_POWER_1;
//
//      if (p->wButtons & XINPUT_GAMEPAD_DPAD_UP)
//        gp.buttons |= C_UP;
//      if (p->wButtons & XINPUT_GAMEPAD_DPAD_DOWN)
//        gp.buttons |= C_DOWN;
//      if (p->wButtons & XINPUT_GAMEPAD_DPAD_LEFT)
//        gp.buttons |= C_POWER_2;
//      if (p->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)
//        gp.buttons |= C_POWER_3;
//
//      if (p->wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) //L1
//        gp.buttons |= C_BRAKE_1;
//      if (p->wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) //R1
//        gp.buttons |= C_BRAKE_3;
//
//      if (p->bLeftTrigger > 128) //L2
//        gp.buttons |= C_BRAKE_2;
//      if (p->bRightTrigger > 128) //R2
//        gp.buttons |= C_BRAKE_4;
//
//      if (p->wButtons & XINPUT_GAMEPAD_BACK)
//        gp.buttons |= C_SELECT;
//      if (p->wButtons & XINPUT_GAMEPAD_START)
//        gp.buttons |= C_START;
//
//      usb_hid.sendReport(0, &gp, sizeof(gp));
//    }
//  }
//  tuh_xinput_receive_report(dev_addr, instance);
//}

// DenshaDeGo callbacks

void tuh_densha_mount_cb(uint8_t dev_addr, uint8_t instance, const denshah_interface_t *densha_itf)
{
    printf("DENSHA MOUNTED %02x %d\n", dev_addr, instance);
    //start receiving report
    connected_device.address = dev_addr;
    connected_device.instance = instance;
    connected = true;
    tuh_densha_receive_report(dev_addr, instance);
}

void tuh_densha_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    connected = false;
    connected_device.address = 0;
    connected_device.instance = 0;
    printf("DENSHA UNMOUNTED %02x %d\n", dev_addr, instance);
}

void tuh_densha_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  //printf("DENSHA RECEIVED %x %x %x \n", dev_addr, instance, len);

  denshah_interface_t *xid_itf = (denshah_interface_t *)report;
  densha_gamepad_t *p = &xid_itf->pad;
  const char* type_str;
  switch (xid_itf->type)
  {
      case 1: type_str = "PS2 TYPE 2"; break;
      default: type_str = "Unknown";
  }

  if (xid_itf->connected && xid_itf->new_pad_data)
  {
    printf("[%02x, %02x], Type: %s, Buttons: %02x, DPad: %02x, Pedal: %02x, Power: %02x, Brake: %02x\n",
         dev_addr, instance, type_str, p->bButtons, p->bDpad, p->bPedal, p->bPower, p->bBrake);

    //How to check specific buttons
//    if (p->bButtons & DENSHA_GAMEPAD_A) printf("You are pressing A\n");
//    if (p->bButtons & DENSHA_GAMEPAD_B) printf("You are pressing B\n");
//    if (p->bButtons & DENSHA_GAMEPAD_C) printf("You are pressing C\n");
//    if (p->bButtons & DENSHA_GAMEPAD_D) printf("You are pressing D\n");
//    if (p->bButtons & DENSHA_GAMEPAD_SELECT) printf("You are pressing SELECT\n");
//    if (p->bButtons & DENSHA_GAMEPAD_START) printf("You are pressing START\n");

//      if (p->bButtons & DENSHA_GAMEPAD_A) {
//        rumblePowerEnabled = !rumblePowerEnabled;
//        tuh_densha_set_rumble_power_handle(dev_addr, instance, rumblePowerEnabled);
//      }
//      if (p->bButtons & DENSHA_GAMEPAD_B) {
//        rumbleBrakeEnabled = !rumbleBrakeEnabled;
//        tuh_densha_set_rumble_brake_handle(dev_addr, instance, rumbleBrakeEnabled);
//      }
//      if (p->bButtons & DENSHA_GAMEPAD_C) {
//        lampEnabled = !lampEnabled;
//        tuh_densha_set_lamp(dev_addr, instance, lampEnabled);
//      }

//      tuh_densha_set_rumble_power_handle(1, 0, p->bPower == 0xff);
//      tuh_densha_set_rumble_brake_handle(1, 0, p->bBrake == 0xff);

      generic_input_report.brake            = p->bBrake;
      generic_input_report.power            = p->bPower;
      generic_input_report.hat              = p->bDpad; //p->bDpad == 0x08 ? 0x00 : p->bDpad + 1;
      generic_input_report.buttons.bButtons = p->bButtons;
      //memcpy(&generic_report.buttons, &p->bButtons, sizeof(generic_report.buttons));

  
//    switch (outputType) {
//      case OUTPUT_PC_ONE_HANDLE:
//        generic_report.brake    = p->bBrake;
//        generic_report.power    = p->bPower;
//        generic_report.hat      = p->bDpad == 0x08 ? 0x00 : p->bDpad + 1;
//        generic_report.buttons  = (p->bButtons);
//        break;
//      case OUTPUT_PC_TWO_HANDLE:
//        generic_report.brake    = p->bBrake;
//        generic_report.power    = p->bPower;
//        //generic_report.hat      = p->bDpad == 0x08 ? 0x00 : p->bDpad + 1;
//        generic_report.buttons  = (p->bButtons);
//        break;
//      case OUTPUT_PC_RYOJOUHEN:
//        generic_report.brake       = p->bBrake; // need to implement
//        //gp.y       = p->bPower;
//        generic_report.buttons = (p->bPedal == 0x00) | (p->bButtons << 1);
//        //gp.buttons |= (p->bButtons);
//        generic_report.hat     = p->bDpad == 0x08 ? 0x00 : p->bDpad + 1;
//  
//        switch (p->bPower) {
//          case 0x81: // N
//            gp.y = 0x00;
//            break;
//          case 0x6D: // P1
//            gp.y = 0x3C;
//            break;
//          case 0x54: // P2
//            gp.y = 0x78;
//            break;
//          case 0x3F: // P3
//            gp.y = 0xB4;
//            break;
//          case 0x21: // P4
//            gp.y = 0xF0;
//            break;
//          case 0x00: // P5
//            gp.y = 0xF0;
//            break;
//  //        case 0xFF: // Transition
//  //          break;
//        }
//  //      switch (p->bBrake) {
//  //        case 0x79: // Released
//  //          gp.x = 0x23;
//  //          break;
//  //        case 0x8A: // B1
//  //          gp.x = 0x2B;
//  //          break;
//  //        case 0x94: // B2
//  //          gp.x = 0x3D;
//  //          break;
//  //        case 0x9A: // B3
//  //          gp.x = 0x4F;
//  //          break;
//  //        case 0xA2: // B4
//  //          gp.x = 0x64;
//  //          break;
//  //        case 0xA8: // B5
//  //          gp.x = 0x8B;
//  //          break;
//  //        case 0xAF: // B6
//  //          gp.x = 0xB1;
//  //          break;
//  //        case 0xB2: // B7
//  //          gp.x = 0xD7;
//  //          break;
//  //        case 0xB5: // B8
//  //          gp.x = 0xD7;
//  //          break;
//  //        case 0xB9: // Emergency
//  //          gp.x = 0xD7;
//  //          break;
//  ////        case 0xFF: // Transition
//  ////          break;
//  //      }
//        break;
//       case OUTPUT_SWITCH_ONE_HANDLE:
//        gp.x       = 0x80;
//        gp.buttons = (p->bButtons);
//        gp.hat     = p->bDpad == 0x08 ? 0x00 : p->bDpad + 1;
//        
//        //brake takes priority.
//        
//        if (p->bBrake != 0x79) {
//          switch (p->bBrake) {
//            case 0x79: // Released
//              gp.y = 0x80;
//              break;
//            case 0x8A: // B1
//              gp.y = 0x65;
//              break;
//            case 0x94: // B2
//              gp.y = 0x57;
//              break;
//            case 0x9A: // B3
//              gp.y = 0x49;
//              break;
//            case 0xA2: // B4
//              gp.y = 0x3C;
//              break;
//            case 0xA8: // B5
//              gp.y = 0x2E;
//              break;
//            case 0xAF: // B6
//              gp.y = 0x20;
//              break;
//            case 0xB2: // B7
//              gp.y = 0x13;
//              break;
//            case 0xB5: // B8
//              gp.y = 0x05;
//              break;
//            case 0xB9: // Emergency
//              gp.y = 0x00;
//              //todo: also ZL pressed
//              break;
//  //          case 0xFF: // Transition
//  //            break;
//          }
//        }
//        //only set power if brake is released
//        if (p->bBrake == 0x79) {
//          switch (p->bPower) {
//            case 0x81: // N
//              gp.y = 0x80;
//              break;
//            case 0x6D: // P1
//              gp.y = 0x9F;
//              break;
//            case 0x54: // P2
//              gp.y = 0xB7;
//              break;
//            case 0x3F: // P3
//              gp.y = 0xCE;
//              break;
//            case 0x21: // P4
//              gp.y = 0xE6;
//              break;
//            case 0x00: // P5
//              gp.y = 0xFF;
//              break;
//  //          case 0xFF: // Transition
//  //            break;
//          }
//        }
//        break;
//       case OUTPUT_HID_CLASSIC:
//        gp.x = 0x80;
//        gp.y = 0x80;
//        gp.z = 0x80;
//        gp.rz = 0x80;
//        gp.rx = 0x80;
//        gp.ry = 0x80;
//        gp.buttons = 0x00;
//
//        if (outputClassicPS1)
//          gp.buttons |= C_UP | C_DOWN;
//          
////        if (p->bDpad == 0x00) //UP
////          gp.buttons |= C_UP;
////        else if (p->bDpad == 0x02) //RIGHT
////          gp.buttons |= C_POWER_3;
////        else if (p->bDpad == 0x04) //DOWN
////          gp.buttons |= C_DOWN;
////        else if (p->bDpad == 0x06) //LEFT
////          gp.buttons |= C_POWER_2;
//
//        if (p->bButtons & DENSHA_GAMEPAD_A)
//          gp.buttons |= C_A;
//        if (p->bButtons & DENSHA_GAMEPAD_B)
//          gp.buttons |= C_B;
//        if (p->bButtons & DENSHA_GAMEPAD_C)
//          gp.buttons |= C_C;
//        //if (p->bButtons & DENSHA_GAMEPAD_D)
//        //  gp.buttons |= C_D;
//        if (p->bButtons & DENSHA_GAMEPAD_SELECT)
//          gp.buttons |= C_SELECT;
//        if (p->bButtons & DENSHA_GAMEPAD_START)
//          gp.buttons |= C_START;
//
//        if (p->bPower == 0x81)      // N
//          gp.buttons |= C_POWER_2 | C_POWER_3;
//        else if (p->bPower == 0x6D) // P1
//          gp.buttons |= C_POWER_1 | C_POWER_3;
//        else if (p->bPower == 0x54) // P2
//          gp.buttons |= C_POWER_3;
//        else if (p->bPower == 0x3F) // P3
//          gp.buttons |= C_POWER_1 | C_POWER_2;
//        else if (p->bPower == 0x21) // P4
//          gp.buttons |= C_POWER_2;
//        else if (p->bPower == 0x00) // P5
//          gp.buttons |= C_POWER_1;
//      //  else if (p->bPower == 0xFF) // Transition
//      //    gp.buttons |= 0x00;
//          
//        if (p->bBrake == 0x79)      // Released
//          gp.buttons |= C_BRAKE_2 | C_BRAKE_3 | C_BRAKE_4;
//        else if (p->bBrake == 0x8A) // B1
//          gp.buttons |= C_BRAKE_1 | C_BRAKE_3 | C_BRAKE_4;
//        else if (p->bBrake == 0x94) // B2
//          gp.buttons |= C_BRAKE_3 | C_BRAKE_4;
//        else if (p->bBrake == 0x9A) // B3
//          gp.buttons |= C_BRAKE_1 | C_BRAKE_2 | C_BRAKE_4;
//        else if (p->bBrake == 0xA2) // B4
//          gp.buttons |= C_BRAKE_2 | C_BRAKE_4;
//        else if (p->bBrake == 0xA8) // B5
//          gp.buttons |= C_BRAKE_1 | C_BRAKE_4;
//        else if (p->bBrake == 0xAF) // B6
//          gp.buttons |= C_BRAKE_4;
//        else if (p->bBrake == 0xB2) // B7
//          gp.buttons |= C_BRAKE_1 | C_BRAKE_2 | C_BRAKE_3;
//        else if (p->bBrake == 0xB5) // B8
//          gp.buttons |= C_BRAKE_2 | C_BRAKE_3;
//      //  else if (p->bBrake == 0xB9) // Emergency
//      //    gp.buttons |= 0x00;
//        else if (p->bBrake == 0xFF) // Transition
//          gp.buttons |= C_BRAKE_1 | C_BRAKE_2 | C_BRAKE_3 | C_BRAKE_4;
//        
//        break;
////       default:
////        return;
//    }
  
    
    
  
  //  if (p->bPedal == 0x00)
  //    gp.buttons |= GAMEPAD_BUTTON_6;
  
  //  switch (p->bPower) {
  //    case 0x81:
  //      gp.buttons |= GAMEPAD_BUTTON_7;
  //      break;
  //    case 0x6D:
  //      gp.buttons |= GAMEPAD_BUTTON_8;
  //      break;
  //    case 0x54:
  //      gp.buttons |= GAMEPAD_BUTTON_9;
  //      break;
  //    case 0x3F:
  //      gp.buttons |= GAMEPAD_BUTTON_10;
  //      break;
  //    case 0x21:
  //      gp.buttons |= GAMEPAD_BUTTON_11;
  //      break;
  //    case 0x00:
  //      gp.buttons |= GAMEPAD_BUTTON_12;
  //      break;
  //  }
  //  
  //  switch (p->bBrake) {
  //    case 0x79:
  //      gp.buttons |= GAMEPAD_BUTTON_13;
  //      break;
  //    case 0x8A:
  //      gp.buttons |= GAMEPAD_BUTTON_14;
  //      break;
  //    case 0x94:
  //      gp.buttons |= GAMEPAD_BUTTON_15;
  //      break;
  //    case 0x9A:
  //      gp.buttons |= GAMEPAD_BUTTON_16;
  //      break;
  //    case 0xA2:
  //      gp.buttons |= GAMEPAD_BUTTON_17;
  //      break;
  //    case 0xA8:
  //      gp.buttons |= GAMEPAD_BUTTON_18;
  //      break;
  //    case 0xAF:
  //      gp.buttons |= GAMEPAD_BUTTON_19;
  //      break;
  //    case 0xB2:
  //      gp.buttons |= GAMEPAD_BUTTON_20;
  //      break;
  //    case 0xB5:
  //      gp.buttons |= GAMEPAD_BUTTON_21;
  //      break;
  //    case 0xB9:
  //      gp.buttons |= GAMEPAD_BUTTON_22;
  //      break;
  //  }
  
//      usb_hid.sendReport(0, &gp, sizeof(gp));
  }
  
  //receive next report
  tuh_densha_receive_report(dev_addr, instance);
}

void tuh_densha_report_sent_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  printf("DENSHA SENT\n");
}
