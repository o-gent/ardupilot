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
    uart = hal.serial(2);
    uart->begin(57600);
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
                arspd_pkt.flags = airspeed.flags;
                arspd_pkt.differential_pressure = airspeed.raw_press;
                arspd_pkt.differential_pressure_sensor_temperature = airspeed.temperature;
                airspeed_packet_update = true;
                break;
            }

            case MAVLINK_MSG_ID_HYGROMETER_SENSOR:
            {
                mavlink_hygrometer_sensor_t hygrometer;
                mavlink_msg_hygrometer_sensor_decode(&msg, &hygrometer);
                temp_pkt.temperature = hygrometer.temperature;
                temp_pkt.device_id = hygrometer.id;
                temperature_packet_update = true;
                break;
            }

            case MAVLINK_MSG_ID_BATTERY_STATUS:
            {
                mavlink_battery_status_t battery;
                mavlink_msg_battery_status_decode(&msg, &battery);
                batt_pkt.average_power_10sec = 0;
                batt_pkt.battery_id = battery.id;
                batt_pkt.current = battery.current_battery;
                batt_pkt.full_charge_capacity_wh = -1;
                batt_pkt.hours_to_full_charge = -1;
                batt_pkt.model_instance_id = 0;
                //batt_pkt.model_name;
                batt_pkt.remaining_capacity_wh = -1;
                batt_pkt.state_of_charge_pct = battery.battery_remaining;
                batt_pkt.state_of_charge_pct_stdev = -1;
                batt_pkt.state_of_health_pct = -1;
                // batt_pkt.status_flags
                batt_pkt.temperature = battery.temperature;
                batt_pkt.voltage = battery.voltages[0];
                battery_packet_update = true;
                break;
            }

            case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            {
                mavlink_distance_sensor_t distance;
                mavlink_msg_distance_sensor_decode(&msg, &distance);
                range_pkt.sensor_id = distance.id;
                range_pkt.range = distance.current_distance;
                rangefinder_packet_update = true;
                break;
            }

            case MAVLINK_MSG_ID_EFI_STATUS:
            {
                mavlink_efi_status_t engine;
                mavlink_msg_efi_status_decode(&msg, &engine);
                efi_pkt.atmospheric_pressure_kpa = engine.barometric_pressure;
                // efi_pkt.coolant_temperature;
                // efi_pkt.cylinder_status;
                efi_pkt.ecu_index = engine.ecu_index;
                efi_pkt.engine_load_percent = engine.engine_load;
                efi_pkt.engine_speed_rpm = engine.rpm;
                efi_pkt.estimated_consumed_fuel_volume_cm3 = engine.fuel_consumed;
                efi_pkt.flags = engine.health;
                efi_pkt.fuel_consumption_rate_cm3pm = engine.fuel_flow;
                efi_pkt.fuel_pressure = engine.fuel_pressure;
                efi_pkt.intake_manifold_pressure_kpa = engine.intake_manifold_pressure;
                efi_pkt.intake_manifold_temperature = engine.intake_manifold_temperature;
                // efi_pkt.oil_pressure;
                // efi_pkt.oil_temperature;
                efi_pkt.spark_dwell_time_ms = engine.spark_dwell_time;
                // efi_pkt.spark_plug_usage;
                // efi_pkt.state;
                efi_pkt.throttle_position_percent = engine.throttle_position;
                efi_packet_update = true;
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