#include "serial_to_can.h"
#include "AP_Periph.h"
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

uavcan_equipment_power_BatteryInfo pkt{};
bool battery_packet = false;

/*
Start the serial driver
*/
void serial2can_init()
{

}

/*
Update CAN packets from the serial packets
*/
void serial2can_update()
{
    battery_packet = true;
}

/*
Send out the CAN packets
*/
void AP_Periph_FW::serial_to_can_update()
{

    if (battery_packet)
    {
        uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE]{};
        const uint16_t total_size = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                         UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
    }
}