//
// Created by lenny on 06/01/24.
//

#include "Gps.h"
#include "mqtt_config.h"

Gps::Gps() {
    
}

Gps::~Gps() {

}

int8_t Gps::init() {
    uart.init(GPS_UART_PORT, GPS_UART_TX, GPS_UART_RX, GPS_UART_BAUD);
    // ntripClient.init();
    //sendGpsConfig();
    return 0;
}

int8_t Gps::deinit() {
    return 0;
}

int8_t Gps::updateAndGetData(positionData &values) {
    char frame[100];
    double lat, lon;
    char c = 0;
    uint8_t count = 0;
    int8_t ret = 0;

    memset(frame, '\0', sizeof(frame));

    while(uart.numBytesAvailable() > 0) {
        if(c == '*' || count >= 99)
            break;
        c = uart.readByte();
        frame[count] = c;
        count++;
    }

    if(NMEA_GGA_decode(frame, lat, lon) == 0) {   
        values.lat = lat;
        values.lon = lon;
    }

    // ntripClient.process();

    // if(ntripClient.getMessageLen()) {
    //     uart.writeBytes((char *)ntripClient.getMessage(), ntripClient.getMessageLen());
    // }
    uart.flush();

    return ret;
}

void Gps::sendGpsConfig() {
    char packet[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x40, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x06};
    uart.writeBytes(packet, sizeof(packet));
}

int8_t Gps::NMEA_GGA_decode(char* nmeaFrame, double &lat, double &lon) {
    int32_t lat_gps_actual, lon_gps_actual;
    
    // 0 Not fix - 1 GPS Fix - 2 DGPS Fix - 3 PPS Fix - 4 RTK fix - 5 Float RTK Fix
    if (nmeaFrame[4] == 'G' && nmeaFrame[5] == 'A' && (nmeaFrame[44] == '1' || nmeaFrame[44] == '2')) { 
        lat_gps_actual = ((int)nmeaFrame[19] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[20] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[22] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[23] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[24] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[25] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[26] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
        lat_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
        lat_gps_actual += ((int)nmeaFrame[17] - 48) *  (long)100000000;                            //Add the degrees multiplied by 10.
        lat_gps_actual += ((int)nmeaFrame[18] - 48) *  (long)10000000;                             //Add the degrees multiplied by 10.

        lon_gps_actual = ((int)nmeaFrame[33] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[34] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[36] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[37] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[38] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[39] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[40] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
        lon_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
        lon_gps_actual += ((int)nmeaFrame[30] - 48) * (long)1000000000;                            //Add the degrees multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[31] - 48) * (long)100000000;                             //Add the degrees multiplied by 10.
        lon_gps_actual += ((int)nmeaFrame[32] - 48) * (long)10000000;                              //Add the degrees multiplied by 10.

        lat = (double)lat_gps_actual / 1e7;
        lon = (double)lon_gps_actual / 1e7;
        return 0;
    }
    return -1;
}