/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(WIFISUPPORT)

#include "../../core/serial.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include "wifi.h"

AsyncWebServer server(80);

#ifndef WIFI_HOSTNAME
  #define WIFI_HOSTNAME DEFAULT_WIFI_HOSTNAME
#endif

void wifi_init() {

  SERIAL_ECHO_MSG("Starting WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    SERIAL_ERROR_MSG("Unable to connect to WiFi with SSID '" WIFI_SSID "', restarting.");
    delay(5000);
    ESP.restart();
  }

  delay(10);
  if (!MDNS.begin(WIFI_HOSTNAME)) {
    SERIAL_ERROR_MSG("Unable to start mDNS with hostname '" WIFI_HOSTNAME "', restarting.");
    delay(5000);
    ESP.restart();
  }

  MDNS.addService("http", "tcp", 80);

  SERIAL_ECHOLNPGM("Successfully connected to WiFi with SSID '" WIFI_SSID "', hostname: '" WIFI_HOSTNAME "', IP address: ", WiFi.localIP().toString().c_str());
}

#endif // WIFISUPPORT
#endif // ARDUINO_ARCH_ESP32
