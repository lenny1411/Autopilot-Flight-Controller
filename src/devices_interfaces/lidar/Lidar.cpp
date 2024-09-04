//
// Created by lenny on 12/01/24.
//

#include "Lidar.h"

Lidar::Lidar() {
    lidar = TF02Pro(&uart);
}

Lidar::~Lidar() {
    deinit();
}

int8_t Lidar::

init() {
    uart.init(LIDAR_UART_PORT, LIDAR_UART_TX, LIDAR_UART_RX, LIDAR_UART_BAUD);
    return lidar.init();
}

int8_t Lidar::deinit() {
    return 0;
}

int8_t Lidar::updateAndGetData(altitudeData &values) {
    return lidar.updateAndGetData(values);
}