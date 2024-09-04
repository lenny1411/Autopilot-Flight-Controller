//
// Created by lenny on 15/04/2024.
//

#include "NtripClient.h"
#include "config.h"
#include "utils/utils.h"

NtripClient::NtripClient() {

}

NtripClient::~NtripClient() {

}

int8_t NtripClient::init() {
    while(!WiFi.isConnected());
    return connect();
}

int8_t NtripClient::connect()
{
    if(!ntripClient.connected()) {
        char request[120];
        memset(request, '\0', sizeof(request));
        sprintf(
            request,
            "GET /%s HTTP/1.1\r\nAccept: */*\r\nUser-Agent: NTRIP Client/1.0\r\nConnection: close\r\n\r\n",
            NTRIP_CASTER_MOUNTPOINT
        );

        if(!ntripClient.connect(NTRIP_CASTER_ADDR, NTRIP_CASTER_PORT))
            return -1;

        ntripClient.write(request, strlen(request));

        unsigned long tempTimeout = get_ms_count();
        while (ntripClient.available() == 0) {
            if (get_ms_count() - tempTimeout > 2000) {
                ntripClient.stop();
                return -1;
            }
            delay_milis(1);
        }

        String status = ntripClient.readStringUntil('\n');

        if(status.indexOf("200") == -1)
            return -1;

        timeout = NTRIP_TIMEOUT;

        return 0;
    }
    return 0;
}

void NtripClient::process() {
    if(connect() != 0) {
        return;
    }

    messageSize = ntripClient.available();

    if(messageSize != 0)
    {
        ntripClient.readBytes(buf, messageSize);
    } else {
        if (timeout <= 0) {
            ntripClient.stop();
            ntripClient = WiFiClient();
        }
        timeout--;
    }
}

int NtripClient::getMessageLen() {
    return messageSize;
}

uint8_t *NtripClient::getMessage() {
    return buf;
}
