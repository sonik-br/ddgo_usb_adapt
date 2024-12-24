#include "tusb_option.h"

#if (TUSB_OPT_HOST_ENABLED)

#include "host/usbh.h"
//#include "host/usbh_classdriver.h"
#include "host/usbh_pvt.h"
#include "class/hid/hid.h"
#include "driver_densha_host.h"

typedef struct
{
    uint8_t inst_count;
    denshah_interface_t instances[CFG_TUH_DENSHA];
} denshah_device_t;

static denshah_device_t _denshah_dev[CFG_TUH_DEVICE_MAX];

TU_ATTR_ALWAYS_INLINE static inline denshah_device_t *get_dev(uint8_t dev_addr)
{
    return &_denshah_dev[dev_addr - 1];
}

TU_ATTR_ALWAYS_INLINE static inline denshah_interface_t *get_instance(uint8_t dev_addr, uint8_t instance)
{
    return &_denshah_dev[dev_addr - 1].instances[instance];
}

static uint8_t get_instance_id_by_epaddr(uint8_t dev_addr, uint8_t ep_addr)
{
    for (uint8_t inst = 0; inst < CFG_TUH_DENSHA; inst++)
    {
        denshah_interface_t *hid = get_instance(dev_addr, inst);

        if ((ep_addr == hid->ep_in) || (ep_addr == hid->ep_out))
            return inst;
    }

    return 0xff;
}

static uint8_t get_instance_id_by_itfnum(uint8_t dev_addr, uint8_t itf)
{
    for (uint8_t inst = 0; inst < CFG_TUH_DENSHA; inst++)
    {
        denshah_interface_t *hid = get_instance(dev_addr, inst);

        if ((hid->itf_num == itf) && (hid->ep_in || hid->ep_out))
            return inst;
    }

    return 0xff;
}

bool tuh_densha_receive_report(uint8_t dev_addr, uint8_t instance)
{
    denshah_interface_t *densha_itf = get_instance(dev_addr, instance);
    TU_VERIFY(usbh_edpt_claim(dev_addr, densha_itf->ep_in));

    if ( !usbh_edpt_xfer(dev_addr, densha_itf->ep_in, densha_itf->epin_buf, densha_itf->epin_size) )
    {
        usbh_edpt_claim(dev_addr, densha_itf->ep_in);
        return false;
    }
    return true;
}


bool tuh_densha_set_rumble_power_handle(uint8_t dev_addr, uint8_t instance, bool state)
{
    //switch (densha_itf->type)
    //case TAITO_DENSYA_CON_T01:
    return tuh_densha_send_report(dev_addr, instance, LEFT_RUMBLE, state);
}

bool tuh_densha_set_rumble_brake_handle(uint8_t dev_addr, uint8_t instance, bool state)
{
    return tuh_densha_send_report(dev_addr, instance, RIGHT_RUMBLE, state);
}

bool tuh_densha_set_lamp(uint8_t dev_addr, uint8_t instance, bool state)
{
    return tuh_densha_send_report(dev_addr, instance, DOOR_LAMP, state);
}

bool tuh_densha_send_report(uint8_t dev_addr, uint8_t instance, densha_function_t function, bool state)
{
    uint8_t txbuf[2] = {
        function, // 1: left rumble, 2: right rumble, 3: door lamp
        state     // 0: off, 1: on
    };
    uint16_t len = sizeof(txbuf);

    tusb_control_request_t const request = {
        .bmRequestType_bit = {
            .recipient = TUSB_REQ_RCPT_INTERFACE,//TUSB_REQ_RCPT_INTERFACE, TUSB_REQ_RCPT_OTHER
            .type      = TUSB_REQ_TYPE_VENDOR,
            .direction = TUSB_DIR_OUT
        },
        .bRequest = HID_REQ_CONTROL_SET_REPORT, // ??? HID_REQ_CONTROL_SET_REPORT   = 0x09, ///< Set Report ???
        .wValue   = tu_htole16(TU_U16(2, 1)), //0x0201, tu_htole16(TU_U16(desc_type, index))
        .wIndex   = 0,
        .wLength  = len//0x0002
    };
    tuh_xfer_t xfer = {
        .daddr       = dev_addr, // from mount callback
        .ep_addr     = 0, // control endpoint has address 0
        .setup       = &request,
        .buffer      = txbuf,
        //.complete_cb = complete_cb
    };

    return tuh_control_xfer(&xfer);
}

//--------------------------------------------------------------------+
// USBH API
//--------------------------------------------------------------------+
bool denshah_init(void)
{
    tu_memclr(_denshah_dev, sizeof(_denshah_dev));
    return true;
}

bool denshah_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
    TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX);

    densha_type_t type = TAITO_DENSYA_UNKNOWN;

    uint16_t PID, VID;
    tuh_vid_pid_get(dev_addr, &VID, &PID);

    if (VID == DENSHA_VID_TAITO)
    {
        if (PID == DENSHA_PID_PS2TYPE2)
            type = TAITO_DENSYA_CON_T01;
    }

    if (type == TAITO_DENSYA_UNKNOWN)
    {
        TU_LOG2("DENSHA: not implemented\n");
        return false;
    }

    TU_LOG2("DENSHA opening Interface %u (addr = %u)\r\n", desc_itf->bInterfaceNumber, dev_addr);

    denshah_device_t *densha_dev = get_dev(dev_addr);
    TU_ASSERT(densha_dev->inst_count < CFG_TUH_DENSHA, 0);

    denshah_interface_t *densha_itf = get_instance(dev_addr, densha_dev->inst_count);
    densha_itf->itf_num = desc_itf->bInterfaceNumber;
    densha_itf->type = type;

    //Parse descriptor for all endpoints and open them
    uint8_t const *p_desc = (uint8_t const *)desc_itf;
    int endpoint = 0;
    int pos = 0;
    while (endpoint < desc_itf->bNumEndpoints && pos < max_len)
    {
        if (tu_desc_type(p_desc) != TUSB_DESC_ENDPOINT)
        {
            pos += tu_desc_len(p_desc);
            p_desc = tu_desc_next(p_desc);
            continue;
        }
        tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *)p_desc;
        TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType);
        TU_ASSERT(tuh_edpt_open(dev_addr, desc_ep));
        if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_OUT)
        {
            densha_itf->ep_out = desc_ep->bEndpointAddress;
            densha_itf->epout_size = tu_edpt_packet_size(desc_ep);
        }
        else
        {
            densha_itf->ep_in = desc_ep->bEndpointAddress;
            densha_itf->epin_size = tu_edpt_packet_size(desc_ep);
        }
        endpoint++;
        pos += tu_desc_len(p_desc);
        p_desc = tu_desc_next(p_desc);
    }

    densha_dev->inst_count++;
    return true;
}

bool denshah_set_config(uint8_t dev_addr, uint8_t itf_num)
{
    uint8_t instance = get_instance_id_by_itfnum(dev_addr, itf_num);
    denshah_interface_t *densha_itf = get_instance(dev_addr, instance);
    densha_itf->connected = true;

    if (tuh_densha_mount_cb)
    {
        tuh_densha_mount_cb(dev_addr, instance, densha_itf);
    }

    usbh_driver_set_config_complete(dev_addr, densha_itf->itf_num);
    return true;
}

bool denshah_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    if (result != XFER_RESULT_SUCCESS)
    {
        TU_LOG1("Error: %d\n", result);
        return false;
    }

    uint8_t const dir = tu_edpt_dir(ep_addr);
    uint8_t const instance = get_instance_id_by_epaddr(dev_addr, ep_addr);
    denshah_interface_t *densha_itf = get_instance(dev_addr, instance);
    densha_gamepad_t *pad = &densha_itf->pad;
    uint8_t *rdata = densha_itf->epin_buf;

    if (dir == TUSB_DIR_IN)
    {
        TU_LOG2("Get Report callback (%u, %u, %u bytes)\r\n", dev_addr, instance, xferred_bytes);
        TU_LOG2_MEM(densha_itf->epin_buf, xferred_bytes, 2);
        if (densha_itf->type == TAITO_DENSYA_CON_T01)
        {
             //0x00 is not a valid value for Brake. Ignore this report
             //Can happen on first report right when the controller is connected
            if (rdata[0] == 0x01 && rdata[1] != 0x00)
            {
                tu_memclr(pad, sizeof(densha_gamepad_t));
                //uint16_t wButtons = rdata[5] << 8 | rdata[4];

                pad->bBrake   = rdata[1];
                pad->bPower   = rdata[2];
                pad->bPedal   = rdata[3];
                pad->bDpad    = rdata[4];
                pad->bButtons = rdata[5];

                densha_itf->new_pad_data = true;
            }
        } 
        tuh_densha_report_received_cb(dev_addr, instance, (const uint8_t *)densha_itf, sizeof(denshah_interface_t));
        densha_itf->new_pad_data = false;
    }
    else
    {
        if (tuh_densha_report_sent_cb)
        {
            tuh_densha_report_sent_cb(dev_addr, instance, densha_itf->epout_buf, xferred_bytes);
        }
    }

    return true;
}

void denshah_close(uint8_t dev_addr)
{
    TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );
    denshah_device_t *densha_dev = get_dev(dev_addr);

    for (uint8_t inst = 0; inst < densha_dev->inst_count; inst++)
    {
        if (tuh_densha_umount_cb)
        {
            tuh_densha_umount_cb(dev_addr, inst);
        }
    }
    tu_memclr(densha_dev, sizeof(denshah_device_t));
}

usbh_class_driver_t const usbh_densha_driver =
{
    #if CFG_TUSB_DEBUG >= CFG_TUH_LOG_LEVEL
      .name = "DENSHADEGO_HOST",
    #endif
    .init       = denshah_init,
    .open       = denshah_open,
    .set_config = denshah_set_config,
    .xfer_cb    = denshah_xfer_cb,
    .close      = denshah_close
};

#endif
