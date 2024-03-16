#include "serial_to_can.h"
#include "AP_Periph.h"
#include <dronecan_msgs.h>




extern const AP_HAL::HAL &hal;

uavcan_equipment_power_BatteryInfo batt_pkt{};
bool battery_packet_update = false;
uavcan_equipment_range_sensor_Measurement range_pkt{};
bool rangefinder_packet_update = false;
uavcan_equipment_device_Temperature temp_pkt{};
bool temperature_packet_update = false;
uavcan_equipment_ice_reciprocating_Status efi_pkt{};
bool efi_packet_update = false;
uavcan_equipment_air_data_RawAirData arspd_pkt {};
bool airspeed_packet_update = false;

AP_HAL::UARTDriver *uart;

/*
Start the serial driver
*/
void serial2can_init()
{
    uart = hal.serial(1);
    uart->begin(115200);
}

/*
Update CAN packets from the serial packets
*/
void serial2can_update()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    while (uart->available() > 0)
    {
        uint8_t c = uart->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {

            }

            case MAVLINK_MSG_ID_AIRSPEED:
            {
                mavlink_airspeed_t airspeed;
                mavlink_msg_airspeed_decode(&msg, &airspeed);
                arspd_pkt.flags = 0;
                arspd_pkt.differential_pressure = airspeed.raw_press;
                airspeed_packet_update = true;
                break;
            }

            default:
            {
                break;
            }
            }
        }
    }
}

/*
Send out the CAN packets
*/
void AP_Periph_FW::serial_to_can_update()
{

    if (battery_packet_update)
    {
        uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE]{};
        const uint16_t total_size = uavcan_equipment_power_BatteryInfo_encode(&batt_pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                         UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
        battery_packet_update = false;
    }

    if (rangefinder_packet_update)
    {
        uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE]{};
        uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&range_pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                         UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
        rangefinder_packet_update = false;
    }

    if (temperature_packet_update)
    {
        uint8_t buffer[UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE]{};
        const uint16_t total_size = uavcan_equipment_device_Temperature_encode(&temp_pkt, buffer, !canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE,
                         UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
        temperature_packet_update = false;
    }

    if (efi_packet_update)
    {
        uint8_t buffer[UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_MAX_SIZE]{};
        const uint16_t total_size = uavcan_equipment_ice_reciprocating_Status_encode(&efi_pkt, buffer, !canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_SIGNATURE,
                         UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
        efi_packet_update = false;
    }

    if (airspeed_packet_update)
    {
        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE]{};
        uint16_t total_size = uavcan_equipment_air_data_RawAirData_encode(&arspd_pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
                         UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
        airspeed_packet_update = false;
    }
}