/* File Name: TFMPlus.cpp
 * Version: 1.5.0
 * Described: Arduino Library for the Benewake TFMini-Plus Lidar sensor
 *            The TFMini-Plus is a unique product, and the various
 *            TFMini Libraries are not compatible with the Plus.
 * Developer: Bud Ryerson
 * Inception: v0.2.0 - 31 JAN 2019 
 * v1.0.0 - 25FEB19 - Initial release
 * v1.0.1 - 09MAR19 - 'build()' function always returned TRUE.
      Corrected to return FALSE if serial data is not available.
      And other minor corrections to textual descriptions.
 * v1.1.0 - 13MAR19 - To simplify, all interface functions now
      return boolean.  Status code is still set and can be read.
      'testSum()' is deleted and 'makeSum()' is used instead.
      Example code is updated.
 * v1.1.1 - 14MAR19 - Two commands: RESTORE_FACTORY_SETTINGS
      and SAVE_SETTINGS were not defined correctly.
 * v1.2.1 - 02APR19 - Rewrote 'getData()' function to make it faster
      and more robust when serial read skips a byte or fails entirely.
 * v1.3.1 - 08APR19 - Redefined commands to include response length
   **********************     IMPORTANT    ************************
   ****  Changed name of 'buildCommand()' to 'sendCommand()'.  ****
   ****************************************************************
 * v.1.3.2 - Added a line to getData() to flush the serial buffer
        of all but last frame of data before reading.  This does not
        effect usage, but will ensure that the latest data is read.
 * v.1.3.3 - 20MAY19 - Changed 'sendCommand()' to add a second byte,
        to the HEADER recognition routine, the reply length byte.
        This makes recognition of the command reply more robust.
        Zeroed out 'data frame' snd  'command reply' buffer arrays
        completely before reading from device.  Added but did not
        implement some I2C command codes.
 * v.1.3.4 - 07JUN19 - Added 'TFMP_' to all error status defines.
        The ubiquitous 'Arduino.h' also contains a 'SERIAL' define.
 * v.1.3.5 - 25OCT19 - Added missing underscore to parameter
        in header file TFMPlus.h:
        Line 138   #define   BAUD_14400   0x003840
 * v.1.3.6 - 27APR20 - a little cleanup in 'getData()'
 * v.1.4.0 - 15JUN20 - Changed all data variables from unsigned
             to signed integers.  Defined abnormal data codes
             as per TFMini-S Producut Manual
           -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            Dist | Strength    |  Comment
             -1    Other value   Strength ≤ 100
             -2    -1            Signal strength saturation
             -4    Other value   Ambient light saturation
           -  -  -  -  -  -  -  -  -  -  -  -  -  -  -   
 * v.1.4.1 - 22JUL20 - Fixed bug in sendCommand() checksum calculation
           - Changed two printf()s to Serial.print()s
	   - Fixed printReply() to show data from 'reply' rather than 'frame'
 * v.1.4.2 - 19MAY21 - Changed command paramter 'FRAME_5' to correct value.
             It was set to 0x0003.  Now it's set to 0x0005
 * v.1.5.0 - 06SEP21 - Corrected (reversed) Enable/Disable commands in 'TFMPlus.h'
             Changed three command names
               OBTAIN_FIRMWARE_VERSION is now GET_FIRMWARE_VERSION
               RESTORE_FACTORY_SETTINGS is now HARD_RESET
               SYSTEM_RESET is now SOFT_RESET
 *
 * Default settings for the TFMini-Plus are a 115200 serial baud rate
 * and a 100Hz measurement frame rate. The device will begin returning
 * measurement data immediately on power up.
 *
 * 'begin()' function passes a serial stream to library and
 *  returns TRUE/FALSE whether serial data is available.
 *  Function also sets a public one byte status code.
 *  Status codes are defined within the library.
 *
 * 'getData( dist, flux, temp)' passes back measurement data
 *  • dist = distance in centimeters,
 *  • flux = signal strength in arbitrary units, and
 *  • temp = an encoded number in degrees centigrade
 *  Function returns TRUE/FALSE whether completed without error.
 *  Error, if any, is saved as a one byte 'status' code.
 *
 * 'sendCommand( cmnd, param)' sends a 32bit command code (cmnd)
 *  and a 32bit parameter value (param). Returns TRUE/FALSE and
 *  sets a one byte status code.
 *  Commands are selected from the library's list of defined commands.
 *  Parameter values can be entered directly (115200, 250, etc) but
 *  for safety, they should be chosen from the Library's defined lists.
 *  An incorrect value can render the device uncommunicative.
 *
 */

#include "TF02Pro.h"
#include "../../../../utils/utils.h"
#include "string.h"
//#include <Wire.h>          //  Future I2C Implementation

// Constructor
TF02Pro::TF02Pro(UartDevice *streamPtr){
    pStream = streamPtr;          // Save reference to stream/serial object.
}
TF02Pro::~TF02Pro(){}

bool TF02Pro::getData( int16_t &dist, int16_t &flux, int16_t &temp)
{
    static char i = 0;
    char j = 0;
    int checksum = 0; 
    int rx[9];

    memset(rx, 0, 9);

    while(pStream->numBytesAvailable()) {  
        rx[i] = pStream->readByte();
        if(rx[0] != 0x59) {
            i = 0;
        } else if(i == 1 && rx[1] != 0x59) {
            i = 0;
        } else if(i == 8) {
            for(j = 0; j < 8; j++) {
                checksum += rx[j];
            }
            if(rx[8] == (checksum % 256)) {
                dist = rx[2] + rx[3] * 256;
                flux = rx[4] + rx[5] * 256;
                return true;
            }
            i = 0;
        } else {
            i++;
        } 
    }  
    return false;
}

int8_t TF02Pro::init() {
    return 0;
}

int8_t TF02Pro::deinit() {
    return 0;
}

int8_t TF02Pro::updateAndGetData(altitudeData &values) {
    int16_t tfDist = 0;    // Distance to object in centimeters
    int16_t tfFlux = 0;    // Strength or quality of return signal
    int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

    if(!getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
        return -1;

    if(tfDist >= 0) {
        values.alt = (float)tfDist;
        return 0;
    }

    return -1;
}
