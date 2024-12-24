// DenshaDeGo tinyusb code by sonik-br
// https://github.com/sonik-br
//
// Based on documentation by marcriera
// https://marcriera.github.io/ddgo-controller-docs
//
// Tinyusb host based on xinput host by Ryzee119
// https://github.com/Ryzee119/tusb_xinput

#ifndef _TUSB_DENSHA_HOST_H_
#define _TUSB_DENSHA_HOST_H_

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef CFG_TUH_DENSHA
#define CFG_TUH_DENSHA 1
#endif

#include "host/usbh_pvt.h"

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

#ifndef CFG_TUH_DENSHA_EPIN_BUFSIZE
#define CFG_TUH_DENSHA_EPIN_BUFSIZE 64
#endif

#ifndef CFG_TUH_DENSHA_EPOUT_BUFSIZE
#define CFG_TUH_DENSHA_EPOUT_BUFSIZE 64
#endif

#define DENSHA_VID_TAITO    0x0AE4
#define DENSHA_PID_PS2TYPE2 0x0004

#define DENSHA_GAMEPAD_B      0x01
#define DENSHA_GAMEPAD_A      0x02
#define DENSHA_GAMEPAD_C      0x04
#define DENSHA_GAMEPAD_D      0x08
#define DENSHA_GAMEPAD_SELECT 0x10
#define DENSHA_GAMEPAD_START  0x20

#define MAX_PACKET_SIZE 32

typedef struct densha_gamepad
{
    uint8_t bButtons;
    uint8_t bDpad;
    uint8_t bPedal;
    uint8_t bPower;
    uint8_t bBrake;
} densha_gamepad_t;

typedef enum
{
    TAITO_DENSYA_UNKNOWN = 0,
    TAITO_DENSYA_CON_T01
} densha_type_t;

typedef enum
{
    LEFT_RUMBLE = 1,
    RIGHT_RUMBLE,
    DOOR_LAMP,
} densha_function_t;

typedef struct
{
    densha_type_t type;
    densha_gamepad_t pad;
    uint8_t connected;
    uint8_t new_pad_data;
    uint8_t itf_num;
    uint8_t ep_in;
    uint8_t ep_out;

    uint16_t epin_size;
    uint16_t epout_size;

    uint8_t epin_buf[CFG_TUH_DENSHA_EPIN_BUFSIZE];
    uint8_t epout_buf[CFG_TUH_DENSHA_EPOUT_BUFSIZE];
} denshah_interface_t;

extern usbh_class_driver_t const usbh_densha_driver;

//--------------------------------------------------------------------+
// Callbacks
//--------------------------------------------------------------------+

void tuh_densha_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);
TU_ATTR_WEAK void tuh_densha_report_sent_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);
TU_ATTR_WEAK void tuh_densha_umount_cb(uint8_t dev_addr, uint8_t instance);
TU_ATTR_WEAK void tuh_densha_mount_cb(uint8_t dev_addr, uint8_t instance, const denshah_interface_t *densha_itf);

//--------------------------------------------------------------------+
// Interface API
//--------------------------------------------------------------------+

bool tuh_densha_receive_report(uint8_t dev_addr, uint8_t instance);
bool tuh_densha_send_report(uint8_t dev_addr, uint8_t instance, uint8_t function, bool state);
bool tuh_densha_set_rumble_power_handle(uint8_t dev_addr, uint8_t instance, bool state);
bool tuh_densha_set_rumble_brake_handle(uint8_t dev_addr, uint8_t instance, bool state);
bool tuh_densha_set_lamp(uint8_t dev_addr, uint8_t instance, bool state);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
bool denshah_init       (void);
bool denshah_open       (uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len);
bool denshah_set_config (uint8_t dev_addr, uint8_t itf_num);
bool denshah_xfer_cb    (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
void denshah_close      (uint8_t dev_addr);

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_DENSHA_HOST_H_ */
