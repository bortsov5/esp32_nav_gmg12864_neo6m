#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <SPI.h>
#include <WebServer.h>
#include "ble.h"
#include <TinyGPS++.h>

/*
15  //si
2   //scl
5   //rs
23  //RSE
13  //cs
*/

#define PIN_SI 15  // Data (MOSI)
#define PIN_SCL 2  // Clock (SCK)
#define PIN_RS 5   // Register Select (DC)
#define PIN_RSE 23 // Reset (RST)
#define PIN_CS 13  // Chip Select

#define start_addr 0x390000

U8G2_ST7565_ERC12864_1_4W_HW_SPI u8g2(
  U8G2_R0,           // Rotation
  PIN_CS,            // Chip Select
  PIN_RS,            // Data/Command
  PIN_RSE            // Reset
);

extern void drawRoute();
extern void startRouteSimulation(uint16_t pointpos);
extern String getTimeString();

// Структура для хранения точки трека
struct TrackPoint {
  double lat;
  double lng;
  int x;
  int y;
};

// Хранилище последних 2 точеки
TrackPoint track[2];
int trackCount = 0;
double zoom = 1.0;  //100 м

// Текущая позиция
double centerLat = 0;
double centerLng = 0;
const double METER_PER_PIXEL = 2.0;  // 1 пиксель = 2 метра
const double MIN_DISTANCE_METERS = 2.0;  // Минимальное расстояние для новой точки

const char *ssid = "RockotAP";
const char *password = "12345678";
bool apStarted = false;

int pointA = -1;
double angle = 0;
bool bleConnected = false;
unsigned long lastCoordTime = 0;
int secondsSinceLastCoord = 99;
unsigned long lastDrawTime = 0;
const unsigned long DRAW_INTERVAL = 5000; // 5 секунд

static const uint8_t ble_icon[] = {
    0b00000000,
    0b00010100,
    0b00101000,
    0b01010100,
    0b01010100,
    0b00101000,
    0b00010100,
    0b00000000
};

// Иконка "нет связи" - крестик ни BLE ни GPS
static const uint8_t no_signal[] = {
    0b10000001,
    0b01000010,
    0b00100100,
    0b00011000,
    0b00011000,
    0b00100100,
    0b01000010,
    0b10000001
};

// Иконка "сигнал"
static const uint8_t gps_icon[] = {
    0b01100011,
    0b00010001,
    0b10001001,
    0b11001111,
    0b10001001,
    0b00010001,
    0b01100001,
    0b00000011
};

int timezoneOffset = 3;//Для Минска UTC+3 (летом и зимой одинаково)

WebServer server8(80);
TinyGPSPlus gps;

double getLastSegmentAngle() {
    if (track[0].lng > 0 && track[1].lng > 0 && track[0].lat > 0 && track[1].lat > 0)  {    
        double latAvg = (track[0].lat + track[1].lat) / 2.0 * PI / 180.0;
        double cosLat = cos(latAvg);
        
        double dx = (track[1].lng - track[0].lng) * cosLat;
        double dy = track[1].lat - track[0].lat;
        
        // 0° = вверх (север), угол по часовой стрелке
        double angle1 = atan2(dx, dy) * 180.0 / PI;
        if (angle1 < 0) angle1 += 360;
        return angle1;
     } else {
      return 0;
     }
}

//расчет расстояния между двумя точками в метрах
double distanceMeters(double lat1, double lng1, double lat2, double lng2) {
  double latAvg = (lat1 + lat2) / 2.0;
  double metersPerDegLng = 111320.0 * cos(latAvg * PI / 180.0);
  
  double dx = (lng2 - lng1) * metersPerDegLng;
  double dy = (lat2 - lat1) * 111320.0;
  
  return sqrt(dx*dx + dy*dy);
}

// Добавление новой точки в трек
void addTrackPoint(double lat, double lng) { 

  if (trackCount==0)  {
    track[0].lat = lat;
    track[0].lng = lng;   
    trackCount = 1; 
    return;  
  } 

  // Проверяем расстояние от последней точки
  double dist = distanceMeters(track[0].lat, track[0].lng, lat, lng);

  if (dist >= MIN_DISTANCE_METERS) {
    track[1].lat = track[0].lat;
    track[1].lng = track[0].lng;
    track[0].lat = lat;
    track[0].lng = lng;    
    trackCount++;
    if (trackCount>1) {
      trackCount = 1;
    }
  }
 
}

//Расчет полной длины маршрута из flash:
double getTotalRouteLength(uint32_t pointCount) {    
    
    if (pointCount == 0 || pointCount > 10000) {
        return 0;
    }
    
    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;

    double totalLength = 0;
    double lat1, lon1, lat2, lon2;
    uint32_t data_addr = base_addr + 4;
    
    Serial.println("");

    // Читаем первую точку
    spi_flash_read(data_addr, (uint32_t*)&lat1, 8);
    spi_flash_read(data_addr + 8, (uint32_t*)&lon1, 8);

    centerLat = lat1;
    centerLng = lon1;
    
    // Суммируем расстояния между последовательными точками
    for (uint32_t i = 1; i < pointCount; i++) {
        uint32_t point_addr = data_addr + (i * 16);
        spi_flash_read(point_addr, (uint32_t*)&lat2, 8);
        spi_flash_read(point_addr + 8, (uint32_t*)&lon2, 8);
        
        totalLength += distanceMeters(lat1, lon1, lat2, lon2);
        lat1 = lat2;
        lon1 = lon2;
    }
    
    return totalLength;
}


///-------------- WEB --------------

 uint32_t totaluploaded = 0;
 String uploadedFileName = "";
 uint32_t uploadedFileBaseAddr = 0;
 size_t uploadedFileSize = 0;

 uint32_t readUploadedFileSize() {
    uint32_t size = 0;
    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;
    spi_flash_read(base_addr, (uint32_t*)&size, 4);
    
    if (size>999999) {
        return 0;
    } else {
        return size;
    }

}

// Обработчик изменения zoom
void handleSetZoom() {
    if (server8.hasArg("zoom")) {
        zoom = server8.arg("zoom").toDouble();
        server8.send(200, "text/html", "Zoom set to: " + String(zoom));
    } else {
        server8.send(400, "text/html", "No zoom value");
    }
}

void handleRoot() {
 
    Serial.println("handleRoot");
    
    String html = "<b>Only  *.gpx</b><hr><form method='POST' action='/upload' enctype='multipart/form-data'>"
                  "<input type='file' name='file' accept='.gpx'><br>"
                  "<input type='submit' value='Upload'>"
                  "</form>"
                  "<hr>"
                  "<form method='POST' action='/setzoom'>"
                  "Zoom: <input type='number' step='0.1' name='zoom' value='" + String(zoom) + "'><br>"
                  "<input type='submit' value='Set Zoom'>"
                  "</form>";
    
    // Проверяем, есть ли сохраненный файл
    uint32_t savedSize = readUploadedFileSize();
    if (savedSize > 0) {
        html += "<br><a href='/view'>View uploaded file content</a>";
        html += "<br><a href='/simulate'>Simulate uploaded file</a>";
        html += "<br>File size: " + String(savedSize) + " points";
        html += "<br>Storage address: 0x510000 + SPI_FLASH_SEC_SIZE";
    }
    
    server8.send(200, "text/html", html);
}


void handleSimulate() {

    Serial.println("handleSimulate");    
    
    String html = "<a href='/view'>View points</a><br>";
        html += "<a href='/simulate'>Simulate points</a><br>";
        html += "<a href='/'>Home</a>";
    
    server8.send(200, "text/html", html);

    startRouteSimulation(0);
}


// Функция вычисления расстояния между двумя точками (в метрах)
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // радиус Земли в метрах
    double dlat = radians(lat2 - lat1);
    double dlon = radians(lon2 - lon1);
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(radians(lat1)) * cos(radians(lat2)) *
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

double minDistance = 0;

void handleViewFile() {
    Serial.println("handleViewFile");   

    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;
    
    // Читаем количество точек
    uint32_t pointCount = 0;
    spi_flash_read(base_addr, (uint32_t*)&pointCount, 4);
    
    if (pointCount == 0 || pointCount > 10000) {
        server8.send(200, "text/plain", "No points found or file corrupted");
        return;
    }
    
    // Поиск ближайшей точки к центру (GPS)
    minDistance = 1e9;
    uint32_t data_addr = base_addr + 4;
    int nearestIndex = -1;
    
    for (uint32_t i = 0; i < pointCount; i++) {
        double lat, lon;
        uint32_t point_addr = data_addr + (i * 16);
        
        spi_flash_read(point_addr, (uint32_t*)&lat, 8);
        spi_flash_read(point_addr + 8, (uint32_t*)&lon, 8);
        
        // Вычисляем расстояние по формуле гаверсинуса
        double dist = haversine(centerLat, centerLng, lat, lon);
        if (dist < minDistance) {
            minDistance = dist;
            nearestIndex = i;
        }
    }
    
    String content = "Total points: " + String(pointCount) + "\n";
    content += "Current GPS: " + String(centerLat, 6) + ", " + String(centerLng, 6) + "\n";
    content += "Nearest point distance: " + String(minDistance, 1) + " meters\n\n";
    content += "First 20 points:\n----------------\n";
    
    uint32_t pointsToShow = (pointCount < 300) ? pointCount : 300;
    
    for (uint32_t i = 0; i < pointsToShow; i++) {
        double lat, lon;
        uint32_t point_addr = data_addr + (i * 16);
        
        spi_flash_read(point_addr, (uint32_t*)&lat, 8);
        spi_flash_read(point_addr + 8, (uint32_t*)&lon, 8);
        
        content += "Point " + String(i + 1);
        if (i == nearestIndex) content += " [NEAREST]";
        content += ":\n  Lat: " + String(lat, 10) + "\n  Lon: " + String(lon, 10) + "\n\n";
    }
    
    server8.send(200, "text/plain", content);
    drawRoute();
}


void handleViewFile2() {

    Serial.println("handleRoot");   

    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;
    
    // Читаем количество точек
    uint32_t pointCount = 0;
    spi_flash_read(base_addr, (uint32_t*)&pointCount, 4);
    
    if (pointCount == 0 || pointCount > 10000) {
        server8.send(200, "text/plain", "No points found or file corrupted");
        return;
    }
    
    String content = "Total points: " + String(pointCount) + "\n\n";
    content += "First 20 points:\n";
    content += "----------------\n";
    
    uint32_t data_addr = base_addr + 4;
    uint32_t pointsToShow = (pointCount < 300) ? pointCount : 300;
    
    for (uint32_t i = 0; i < pointsToShow; i++) {
        double lat, lon;
        uint32_t point_addr = data_addr + (i * 16);
        
        spi_flash_read(point_addr, (uint32_t*)&lat, 8);
        spi_flash_read(point_addr + 8, (uint32_t*)&lon, 8);
        
        content += "Point " + String(i + 1) + ":\n";
        content += "  Lat: " + String(lat, 10) + "\n";
        content += "  Lon: " + String(lon, 10) + "\n\n";
    }
    
    server8.send(200, "text/plain", content);
    drawRoute();
}

 void handleFileUpload() {
    static size_t base_addr;
    static uint32_t pointCount;
    static bool inRte = false;
    static bool inTrkseg = false;
    static String buffer;
    static String pendingTag;  // Добавляем буфер для незаконченного тега
    static int lastPos = 0;
    static int chunkNumber = 0;
  
    HTTPUpload& upload = server8.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        Serial.println("\n=== UPLOAD START ===");
        Serial.println("Filename: " + upload.filename);
        
        base_addr = start_addr + SPI_FLASH_SEC_SIZE;
        pointCount = 0;
        buffer = "";
        pendingTag = "";  // Очищаем
        lastPos = 0;
        inRte = false;
        inTrkseg = false;
        chunkNumber = 0;
        uploadedFileName = upload.filename;
        totaluploaded = 0;
        
        for (int i = 0; i < 30; i++) {
            spi_flash_erase_sector(base_addr / SPI_FLASH_SEC_SIZE + i);
        }
  
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (totaluploaded >= 0x70000) return;
  
        size_t data_len = upload.currentSize;
        uint8_t* data_buf = upload.buf;
        
        Serial.printf("\n=== CHUNK %d ===\n", chunkNumber++);
        Serial.printf("Received %d bytes\n", data_len);
        Serial.println("");

        if (data_len > 2 && data_buf[0] == 0x0D && data_buf[1] == 0x0A) {
            data_len -= 2;
            data_buf += 2;
            Serial.println("Removed CRLF at start");
        }
  
        if (data_len > 0) {
            // Сначала добавляем сохраненный pendingTag к новым данным
            if (pendingTag.length() > 0) {
                Serial.printf("Adding pending tag (%d bytes): %s\n", pendingTag.length(), pendingTag.c_str());
                buffer = pendingTag;
                pendingTag = "";
            }
            
            // Добавляем новые данные в буфер
            size_t oldBufferLen = buffer.length();
            for (size_t i = 0; i < data_len; i++) {
                buffer += (char)data_buf[i];
            }
            Serial.printf("Buffer size: %d -> %d bytes\n", oldBufferLen, buffer.length());
            Serial.println("");
            
            int pos = 0;  // Всегда начинаем с 0, т.к. pendingTag уже добавлен
            int processedPoints = 0;
            
            while (true) {
                // Ищем начало rte или trkseg если еще не внутри
                if (!inRte && !inTrkseg) {
                    int rtePos = buffer.indexOf("<rte", pos);
                    int trksegPos = buffer.indexOf("<trkseg", pos);
                    
                    if (rtePos != -1 && (trksegPos == -1 || rtePos < trksegPos)) {
                        inRte = true;
                        pos = rtePos;
                        Serial.printf("Found <rte at position %d\n", rtePos);
                    } else if (trksegPos != -1) {
                        inTrkseg = true;
                        pos = trksegPos;
                        Serial.printf("Found <trkseg at position %d\n", trksegPos);
                    } else {
                        break;
                    }
                }
                
                // Ищем точки в rte
                if (inRte) {
                    int rteptPos = buffer.indexOf("<rtept", pos);
                    if (rteptPos != -1) {
                        int closeTag = buffer.indexOf("/>", rteptPos);
                        if (closeTag != -1 && closeTag < buffer.indexOf(">", rteptPos)) {
                            int latPos = buffer.indexOf("lat=\"", rteptPos);
                            int lonPos = buffer.indexOf("lon=\"", rteptPos);
                            
                            if (latPos != -1 && lonPos != -1 && latPos < closeTag && lonPos < closeTag) {
                                int latEnd = buffer.indexOf("\"", latPos + 5);
                                int lonEnd = buffer.indexOf("\"", lonPos + 5);
                                
                                if (latEnd != -1 && lonEnd != -1 && latEnd < closeTag && lonEnd < closeTag) {
                                    String latStr = buffer.substring(latPos + 5, latEnd);
                                    String lonStr = buffer.substring(lonPos + 5, lonEnd);
                                    
                                    double lat = latStr.toDouble();
                                    double lon = lonStr.toDouble();
                                    
                                    uint32_t data_addr = base_addr + 4 + (pointCount * 16);
                                    spi_flash_write(data_addr, (uint32_t*)&lat, 8);
                                    spi_flash_write(data_addr + 8, (uint32_t*)&lon, 8);
                                    
                                    pointCount++;
                                    processedPoints++;
                                    
                                    if (pointCount % 100 == 0) {
                                        Serial.println("Saved point " + String(pointCount) + ": " + 
                                                     String(lat, 8) + ", " + String(lon, 8));
                                    }
                                }
                            }
                            pos = closeTag + 2;
                        } else {
                            // Не нашли закрытие тега - возможно разрыв
                            pendingTag = buffer.substring(rteptPos);
                            buffer = buffer.substring(0, rteptPos);
                            Serial.printf("Incomplete rtept tag detected, saved %d bytes to pending\n", pendingTag.length());
                            goto finish;
                        }
                    } else {
                        int endRte = buffer.indexOf("</rte>", pos);
                        if (endRte != -1) {
                            inRte = false;
                            pos = endRte + 6;
                            Serial.println("Found </rte>");
                        } else {
                            break;
                        }
                    }
                }
                
                // Ищем точки в trkseg
                if (inTrkseg) {
                    int trkptPos = buffer.indexOf("<trkpt", pos);
                    if (trkptPos != -1) {
                        int closeTag = buffer.indexOf("/>", trkptPos);
                        if (closeTag != -1 && closeTag < buffer.indexOf(">", trkptPos)) {
                            int latPos = buffer.indexOf("lat=\"", trkptPos);
                            int lonPos = buffer.indexOf("lon=\"", trkptPos);
                            
                            if (latPos != -1 && lonPos != -1 && latPos < closeTag && lonPos < closeTag) {
                                int latEnd = buffer.indexOf("\"", latPos + 5);
                                int lonEnd = buffer.indexOf("\"", lonPos + 5);
                                
                                if (latEnd != -1 && lonEnd != -1 && latEnd < closeTag && lonEnd < closeTag) {
                                    String latStr = buffer.substring(latPos + 5, latEnd);
                                    String lonStr = buffer.substring(lonPos + 5, lonEnd);
                                    
                                    double lat = latStr.toDouble();
                                    double lon = lonStr.toDouble();
                                    
                                    uint32_t data_addr = base_addr + 4 + (pointCount * 16);
                                    spi_flash_write(data_addr, (uint32_t*)&lat, 8);
                                    spi_flash_write(data_addr + 8, (uint32_t*)&lon, 8);
                                    
                                    pointCount++;
                                    processedPoints++;
                                    
                                    if (pointCount % 100 == 0) {
                                        Serial.println("Saved point " + String(pointCount) + ": " + 
                                                     String(lat, 8) + ", " + String(lon, 8));
                                    }
                                }
                            }
                            pos = closeTag + 2;
                        } else {
                            // Не нашли закрытие тега - возможно разрыв
                            pendingTag = buffer.substring(trkptPos);
                            buffer = buffer.substring(0, trkptPos);
                            Serial.printf("Incomplete trkpt tag detected, saved %d bytes to pending\n", pendingTag.length());
                            goto finish;
                        }
                    } else {
                        int endTrkseg = buffer.indexOf("</trkseg>", pos);
                        if (endTrkseg != -1) {
                            inTrkseg = false;
                            pos = endTrkseg + 9;
                            Serial.println("Found </trkseg>");
                        } else {
                            break;
                        }
                    }
                }
                
                lastPos = pos;
                
                if (pos >= buffer.length()) {
                    break;
                }
            }
            
            finish:
            
            Serial.printf("Processed %d points in this chunk\n", processedPoints);
            Serial.printf("Current total points: %d\n", pointCount);
            Serial.println("");

            // Сохраняем остаток буфера
            if (lastPos < buffer.length()) {
                String remainder = buffer.substring(lastPos);
                Serial.printf("Remaining buffer (%d bytes): ", remainder.length());
                if (remainder.length() > 0) {
                    String preview = remainder.substring(0, min(remainder.length(), (size_t)80));
                    Serial.println(preview);
                }
                buffer = remainder;
                lastPos = 0;
            } else {
                Serial.println("Buffer fully processed, cleared");
                buffer = "";
                lastPos = 0;
            }
            
            totaluploaded += data_len;
            Serial.printf("Total uploaded: %d bytes\n", totaluploaded);
        }
  
    } else if (upload.status == UPLOAD_FILE_END) {
        Serial.println("\n=== UPLOAD END ===");
        
        // Обрабатываем оставшийся pendingTag
        if (pendingTag.length() > 0) {
            Serial.printf("Processing pending tag (%d bytes): %s\n", pendingTag.length(), pendingTag.c_str());
            buffer = pendingTag;
            pendingTag = "";
            
            // Пытаемся обработать последний тег
            int closeTag = buffer.indexOf("/>");
            if (closeTag != -1) {
                int latPos = buffer.indexOf("lat=\"");
                int lonPos = buffer.indexOf("lon=\"");
                
                if (latPos != -1 && lonPos != -1 && latPos < closeTag && lonPos < closeTag) {
                    int latEnd = buffer.indexOf("\"", latPos + 5);
                    int lonEnd = buffer.indexOf("\"", lonPos + 5);
                    
                    if (latEnd != -1 && lonEnd != -1 && latEnd < closeTag && lonEnd < closeTag) {
                        String latStr = buffer.substring(latPos + 5, latEnd);
                        String lonStr = buffer.substring(lonPos + 5, lonEnd);
                        
                        double lat = latStr.toDouble();
                        double lon = lonStr.toDouble();
                        
                        uint32_t data_addr = base_addr + 4 + (pointCount * 16);
                        spi_flash_write(data_addr, (uint32_t*)&lat, 8);
                        spi_flash_write(data_addr + 8, (uint32_t*)&lon, 8);
                        
                        pointCount++;
                        Serial.println("Saved final point " + String(pointCount) + ": " + 
                                     String(lat, 8) + ", " + String(lon, 8));
                    }
                }
            }
        }
        
        Serial.printf("Final buffer content (%d bytes):\n", buffer.length());
        Serial.println("=== BUFFER START ===");
        Serial.println(buffer);
        Serial.println("=== BUFFER END ===");
        
        spi_flash_write(base_addr, (uint32_t*)&pointCount, 4);
        
        Serial.println("\n=== RESULTS ===");
        Serial.println("Upload complete!");
        Serial.println("Total points: " + String(pointCount));
        Serial.println("Memory used: " + String(4 + pointCount * 16) + " bytes");
        
        String response = "File uploaded successfully<br>";
        response += "Total points: " + String(pointCount) + "<br>";
        response += "Memory used: " + String(4 + pointCount * 16) + " bytes<br>";
        response += "<a href='/view'>View points</a><br>";
        response += "<a href='/simulate'>Simulate points</a><br>";
        response += "<a href='/'>Home</a>";
        
        server8.send(200, "text/html", response);
        
        // Очистка
        buffer = "";
        pendingTag = "";
        pointCount = 0;
        inRte = false;
        inTrkseg = false;
        lastPos = 0;
    }
}

/// <<< ---- WEB

void drawRotatedTriangle(int x, int y, double angle) {
    int height = 10;  // Высота треугольника
    int base = 6;    // Ширина основания

    x = constrain(x, 10, 122);  // 15-113 по X (128-15=113)
    y = constrain(y, 10, 58);   // 15-49 по Y (64-15=49)

    angle = angle + 90;
    
    // Вершина (острие) - вытянута вперед
    int tipX = x + height * cos(angle * PI / 180);
    int tipY = y + height * sin(angle * PI / 180);
    
    // Углы основания (сзади)
    double backAngle = angle + 180;
    int baseLeftX = x + (base/2) * cos((backAngle - 90) * PI / 180);
    int baseLeftY = y + (base/2) * sin((backAngle - 90) * PI / 180);
    int baseRightX = x + (base/2) * cos((backAngle + 90) * PI / 180);
    int baseRightY = y + (base/2) * sin((backAngle + 90) * PI / 180);
    
    u8g2.drawLine(tipX, tipY, baseLeftX, baseLeftY);
    u8g2.drawLine(tipX, tipY, baseRightX, baseRightY);
    u8g2.drawLine(baseLeftX, baseLeftY, baseRightX, baseRightY);
}

void updateBleStatus()
{
    if ((bleConnected)||gps.location.isValid()) {
        unsigned long diff = (millis() - lastCoordTime) / 1000;
        secondsSinceLastCoord = (diff > 99) ? 99 : diff;
    } else {
        secondsSinceLastCoord = 99;
    }
}

void drawIcon(int x, int y, const uint8_t* icon, int width = 8, int height = 8) {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (icon[i] & (1 << (7 - j))) {
                u8g2.drawPixel(x + j, y + i);
            }
        }
    }
}

void drawStatusBar()
{
    updateBleStatus();
    
    // Рисуем фон статусной строки
    u8g2.setDrawColor(0);
    u8g2.drawBox(95, 0, 33, 12);  // Черный фон для контраста
    
    u8g2.setDrawColor(1);

    u8g2.setFont(u8g2_font_6x13_t_cyrillic);
    
    // Иконка BLE
    if (bleConnected) {
        // Рисуем символ типа BLE 
        drawIcon(96, 2, ble_icon);
    } else {
        if (gps.location.isValid()) {
            drawIcon(96, 2, gps_icon);
        } else
        {
          drawIcon(96, 2, no_signal);
        }
    }
    
    // Цифра секунд
    u8g2.setCursor(113, 10);
    if (secondsSinceLastCoord < 10) u8g2.print("0");
    u8g2.print(secondsSinceLastCoord);   
}


// Дополнительная функция для рисования стрелки направления
void drawDirectionArrow(int x, int y, double angle) {
    int arrowSize = 8;
    int arrowX = x, arrowY = y;
    
    // Вычисляем концы стрелки
    double rad = angle * PI / 180;
    
    // Основная линия стрелки
    int tipX = arrowX + (int)(arrowSize * cos(rad));
    int tipY = arrowY + (int)(arrowSize * sin(rad));
    
    // "Усы" стрелки
    double angle1 = rad + 140 * PI / 180;
    double angle2 = rad - 140 * PI / 180;
    
    int wing1X = tipX + (int)(arrowSize * 0.6 * cos(angle1));
    int wing1Y = tipY + (int)(arrowSize * 0.6 * sin(angle1));
    int wing2X = tipX + (int)(arrowSize * 0.6 * cos(angle2));
    int wing2Y = tipY + (int)(arrowSize * 0.6 * sin(angle2));
    
    // Рисуем стрелку
    u8g2.drawLine(arrowX, arrowY, tipX, tipY);
    u8g2.drawLine(tipX, tipY, wing1X, wing1Y);
    u8g2.drawLine(tipX, tipY, wing2X, wing2Y);
    
    // Рисуем кружок в центре стрелки
    u8g2.drawCircle(arrowX, arrowY, 2);
}

void drawRoute()
{
    bool anyPointVisible = false;      // Флаг: видна ли хоть одна точка маршрута
    int32_t nearestX = 0, nearestY = 0;
    int32_t nearestDist = 999999;  

    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;
    uint32_t pointCount = 0;
    spi_flash_read(base_addr, (uint32_t*)&pointCount, 4);
    
    if (pointCount < 2 || pointCount > 10000) return;
    uint32_t data_addr = base_addr + 4;
    
    // Лямбда для перевода координат в экранные
    auto toScreen = [=](double lat, double lon) -> std::pair<int16_t, int16_t> {
        // ВНИМАНИЕ: результат может выходить за пределы int16_t!
        int32_t x = (int32_t)(64 + (lon - centerLng) * 60000 * zoom);
        int32_t y = (int32_t)(32 - (lat - centerLat) * 60000 * zoom);
        
        // Клипинг значений к допустимому диапазону int16_t
        x = constrain(x, -32768, 32767);
        y = constrain(y, -32768, 32767);
        
        return {(int16_t)x, (int16_t)y};
    };
    
    // Используем int32_t для промежуточных вычислений
    int32_t* xs = new int32_t[pointCount];
    int32_t* ys = new int32_t[pointCount];

    int32_t finishX = 0, finishY = 0;

    for (uint32_t i = 0; i < pointCount; i++) {
        uint32_t point_addr = data_addr + (i * 16);
        double lat, lon;
        spi_flash_read(point_addr, (uint32_t*)&lat, 8);
        spi_flash_read(point_addr + 8, (uint32_t*)&lon, 8);
        
        auto [x, y] = toScreen(lat, lon);
        xs[i] = x;
        ys[i] = y;

        // Проверяем видимость точки
        if (x >= 0 && x < 128 && y >= 0 && y < 64) {
            anyPointVisible = true;
        }
        
        // Ищем ближайшую точку к центру экрана (для указателя)
        int32_t distToCenter = abs(x - 64) + abs(y - 32); // Манхэттенское расстояние
        if (i == 0 || distToCenter < nearestDist) {
            nearestDist = distToCenter;
            nearestX = x;
            nearestY = y;
        }        

        if (i == pointCount - 1) {
            finishX = x;
            finishY = y;
        }

        if (pointA > 0) {
            if (pointA == i) {
                addTrackPoint(lat, lon);
                angle = getLastSegmentAngle(); 
                Serial.println(angle, 8);                     
            }
        }
    }
    
    // Отрисовка
    u8g2.firstPage();
    do {
        for (uint32_t i = 1; i < pointCount; i++) {
            // Рисуем линии, если хотя бы одна точка видима
            // (ваш старый код рисовал только если xs[i] видима, но xs[i-1] могла быть невидима)
            bool visible1 = (xs[i-1] >= 0 && xs[i-1] < 128 && ys[i-1] >= 0 && ys[i-1] < 64);
            bool visible2 = (xs[i] >= 0 && xs[i] < 128 && ys[i] >= 0 && ys[i] < 64);
            
            if (visible1 && visible2) {
                u8g2.drawLine(xs[i-1], ys[i-1], xs[i], ys[i]);
            } else if (visible1 || visible2) {
                // Если одна точка видима, а другая нет - рисуем линию до края экрана
                // (опционально: улучшенная обрезка линии)
                int16_t x1 = constrain(xs[i-1], 0, 127);
                int16_t y1 = constrain(ys[i-1], 0, 63);
                int16_t x2 = constrain(xs[i], 0, 127);
                int16_t y2 = constrain(ys[i], 0, 63);
                u8g2.drawLine(x1, y1, x2, y2);
            }
            
            if (pointA > 0 && pointA + 1 == i) {                                       
                drawRotatedTriangle(xs[i-1], ys[i-1], angle);
            }
        }
        
        // РИСУЕМ КРУЖОК НА ФИНИШЕ
        if (pointCount > 0 && finishX >= 0 && finishX < 128 && finishY >= 0 && finishY < 64) {
            // Рисуем кружок радиусом 3 пикселя
            u8g2.drawCircle(finishX, finishY, 3);
            // Можно сделать двойной кружок для большей заметности
            u8g2.drawCircle(finishX, finishY, 2);
            
            // Или закрашенный кружок (если нужен)
            // u8g2.drawDisc(finishX, finishY, 3);
        }
        
        drawStatusBar();
        String timeString = getTimeString();
        int textWidth = u8g2.getStrWidth(timeString.c_str());
         int x = 128 - textWidth - 2;  // 128 - ширина экрана
        int y = 64 - 2;               // 64 - высота экрана, отступ снизу
        
        // Рисуем фон (опционально, для лучшей читаемости)
        u8g2.setDrawColor(0);  // Цвет фона (0 = черный/очистка)
        u8g2.drawBox(x - 1, y - 7, textWidth + 2, 8);  // Очищаем область
        u8g2.setDrawColor(1);  // Цвет рисования (1 = белый)
        
        // Выводим время
        u8g2.setCursor(x, y);
        u8g2.print(timeString);
        
            if (!anyPointVisible) {
                double dx = nearestX - 64;
                double dy = nearestY - 32;
                double directionAngle = atan2(dy, dx) * 180 / PI;

                // Рисуем указатель направления (стрелку) на краю экрана
                int arrowX = 64, arrowY = 32;
                int arrowSize = 10;
                
                // Размещаем стрелку на соответствующем краю экрана
                if (abs(dx) > abs(dy)) {
                    // Горизонтальное направление
                    arrowY = 32;
                    if (dx > 0) arrowX = 128 - 15;  // Правый край
                    else arrowX = 15;                // Левый край
                } else {
                    // Вертикальное направление
                    arrowX = 64;
                    if (dy > 0) arrowY = 64 - 15;   // Нижний край
                    else arrowY = 15;                // Верхний край
                }
                
                // Рисуем стрелку с поворотом
                drawDirectionArrow(arrowX, arrowY, directionAngle);
            }

        // БАГ ИСПРАВЛЕНИЕ: = это присваивание, нужно == для сравнения
        if (pointA == -1) {  // <-- ИСПРАВЛЕНО: было pointA=-1
            drawRotatedTriangle(64, 32, angle); // Я всегда в центре           
        }
    } while (u8g2.nextPage());
    
    delete[] xs;
    delete[] ys;
}

void drawRoute2() //старый вариант (не используется)
{
    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;
    uint32_t pointCount = 0;
    spi_flash_read(base_addr, (uint32_t*)&pointCount, 4);
    
    if (pointCount == 0 || pointCount > 10000) {
        return;
    }

    uint32_t data_addr = base_addr + 4;
    Serial.println("---");
    
    // Читаем первую точку (центр)
    double centerLat, centerLon;
    spi_flash_read(data_addr, (uint32_t*)&centerLat, 8);
    spi_flash_read(data_addr + 8, (uint32_t*)&centerLon, 8);
    
    auto toScreen = [&](double lat, double lon, int16_t &x, int16_t &y) {
        x = 64 + (lon - centerLon) * 60000;  // подберите множитель
        y = 32 - (lat - centerLat) * 60000;
    };
    
    // Рисуем маршрут
    u8g2.firstPage();
    do {
        double lat1, lon1, lat2, lon2;
        int16_t x1, y1, x2, y2;
        
        data_addr = base_addr + 4;

        // Первая точка
        spi_flash_read(data_addr, (uint32_t*)&lat1, 8);
        spi_flash_read(data_addr + 8, (uint32_t*)&lon1, 8);
        toScreen(lat1, lon1, x1, y1);
        u8g2.drawPixel(x1, y1);
        
        Serial.println("-1"); 

        // Остальные точки
        for (uint32_t i = 1; i < pointCount; i++) {
            uint32_t point_addr = data_addr + (i * 16);
            spi_flash_read(point_addr, (uint32_t*)&lat2, 8);
            spi_flash_read(point_addr + 8, (uint32_t*)&lon2, 8);
            
            toScreen(lat2, lon2, x2, y2);
            //u8g2.drawPixel(x2, y2);
            if (x2<128 && x2>0 && y2<64 && y2>0)
            {
              u8g2.drawLine(x1, y1, x2, y2);
            }

            Serial.println(i);
            
            if (pointA+1==i) {          
                addTrackPoint(lat2, lon2);
                angle = getLastSegmentAngle();
                Serial.print(angle); Serial.print(" ");
                drawRotatedTriangle(x1, y1, angle);
            }

            lat1 = lat2; lon1 = lon2;
            x1 = x2; y1 = y2;
        }
    } while (u8g2.nextPage());
}

void onCharacteristicWrite(const String &uuid, uint8_t *data, size_t length)
{

     String value = (uuid != CHA_NAV_TBT_ICON) ? String((char *)(data)) : String();

    if (uuid == CHA_GPS_POSITIONS) {
      int firstComma = value.indexOf(',');
      int secondComma = value.indexOf(',', firstComma + 1); 

      if (firstComma != -1 && secondComma != -1) {
            String latStr = value.substring(0, firstComma);
            String lonStr = value.substring(firstComma + 1, secondComma);
            String speedStr = value.substring(secondComma + 1);
            
            double latitude = latStr.toDouble();
            double longitude = lonStr.toDouble();
            float speed = speedStr.toFloat();
            
            centerLat = latitude;
            centerLng = longitude;  //Центр теперь тут (я тут)

            addTrackPoint(latitude, longitude);
            angle = getLastSegmentAngle(); 
            pointA = -1;
            lastCoordTime = millis();
            drawRoute();
      }

    }

    Serial.print("onCharacteristicWrite: ");
    Serial.print(uuid);
    Serial.print(" ");
    Serial.print(length);
    Serial.print(" bytes: ");
    Serial.println((char*)data);

}

void onConnectionChange(bool connected)
{
 // connectionChanged = true;
    bleConnected = connected;
    drawRoute();
}

/*
void sendUBX(uint8_t *msg, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        Serial2.write(msg[i]);
    }
    Serial.println("UBX command sent");
}

void enableAllNMEA() {
    // Включаем GGA
    uint8_t enableGGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x24, 0x7E};
    sendUBX(enableGGA, sizeof(enableGGA));
    delay(100);
    
    // Включаем RMC
    uint8_t enableRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x27, 0x7E};
    sendUBX(enableRMC, sizeof(enableRMC));
    delay(100);
    
    // Включаем GSA
    uint8_t enableGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x25, 0x7E};
    sendUBX(enableGSA, sizeof(enableGSA));
    delay(100);
    
    // Включаем GSV
    uint8_t enableGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x28, 0x7E};
    sendUBX(enableGSV, sizeof(enableGSV));
    delay(100);
    
    // Включаем VTG
    uint8_t enableVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x2A, 0x7E};
    sendUBX(enableVTG, sizeof(enableVTG));
}*/

void setup(void)
{
  SPI.begin(PIN_SCL, -1, PIN_SI, PIN_CS);  // SCK, MISO, MOSI, CS
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setContrast(10);
  Serial.begin(115200);
  Serial2.begin(9600);

 // delay(2000);
 // enableAllNMEA();

  // Проверяем, есть ли сохраненный файл маршрута
  uint32_t savedSize = readUploadedFileSize();
  double totalLength = getTotalRouteLength(savedSize);

  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_6x13_t_cyrillic);//u8g2_font_5x7_t_cyrillic
    u8g2.drawUTF8(0, 10, "GPS Навигатор");
    if (savedSize > 0) {
      u8g2.drawUTF8(0, 35, "Маршрут");  
      u8g2.setCursor(45, 35);   
      u8g2.print(savedSize);     
      u8g2.drawUTF8(90, 35, "точек");

      u8g2.drawUTF8(0, 50, "Длина "); 
      u8g2.setCursor(45, 50);   
      u8g2.print(totalLength);   
      
    } else {
      u8g2.drawUTF8(0, 35, "Маршрут не сохранён");
    }
  } while (u8g2.nextPage());

  Serial.println("Display ready!");

  Serial.println("Starting WiFi AP...");
  WiFi.mode(WIFI_AP);
  apStarted = WiFi.softAP(ssid, password);

  if (apStarted)
  {
    Serial.println("AP started");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    server8.on("/", HTTP_GET, handleRoot);
    server8.on("/view", HTTP_GET, handleViewFile);
    server8.on("/simulate", HTTP_GET, handleSimulate);
    server8.on("/setzoom", handleSetZoom);
    server8.on("/upload", HTTP_POST, []() {
    server8.send(200, "text/plain", "File upload completed"); }, handleFileUpload);   
    server8.begin();
    server8.sendHeader("Connection", "close");
    server8.client().setTimeout(120000); // 120 секунд в миллисекундах
    
  }
  else
  {
    Serial.println("AP failed to start");
  }

  delay(2000);

  if (savedSize == 0)
  {
    // Тестовый паттерн
    u8g2.firstPage();
    do
    {
      u8g2.drawUTF8(0, 20, "Загрузите маршрут GPX");
      u8g2.drawUTF8(0, 35, "WIFI AP RockotAP");
      u8g2.drawUTF8(0, 50, "pass 12345678");
    } while (u8g2.nextPage());

  } else {
    drawRoute();
  }

  initBle();

}

uint32_t simDataAddr = 0;
void startRouteSimulation(uint16_t pointpos = 0) {

    Serial.println("startRouteSimulation");

    uint32_t base_addr = start_addr + SPI_FLASH_SEC_SIZE;
    uint32_t pointCount = 0;
    spi_flash_read(base_addr, (uint32_t*)&pointCount, 4);
    
    if (pointCount >= 2) {

        simDataAddr = base_addr + 4;
       for (uint32_t i = 0; i < pointCount; i++) {
            
            
            // Устанавливаем центр на первую точку маршрута
            spi_flash_read(simDataAddr, (uint32_t*)&centerLat, 8);
            spi_flash_read(simDataAddr + 8, (uint32_t*)&centerLng, 8);
            addTrackPoint(centerLat, centerLng);
            angle = getLastSegmentAngle(); 
            simDataAddr = simDataAddr + 16;
            
            if (((pointpos>0)&&(i==pointpos))||(pointpos==0)) {
              Serial.println(i);
              drawRoute();
              delay(200);    
            }
            
            
        }
    }
}

void  handleUartInput()
{
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if ((input.length() > 0) && (input.length() < 10)) {
           uint16_t pointV = atoi(input.c_str());
           startRouteSimulation(pointV);
        }       
    }
  }


bool needRedraw() {
    if (!bleConnected) return false;
    if (gps.location.isValid()) return true;
    
    unsigned long diff = (millis() - lastCoordTime) / 1000;
    // Если данные не приходили больше 5 секунд
    if (diff >= 5) {
        // Проверяем, не рисовали ли мы уже давно
        if (millis() - lastDrawTime >= DRAW_INTERVAL) {
            lastDrawTime = millis();
            return true;
        }
    }
    return false;
}


String getTimeString() {
    if (!gps.time.isValid()) return "--:--:--";
    
    int hour = gps.time.hour() + timezoneOffset;
    
    // Корректировка при переходе через 0 и 24
    if (hour >= 24) hour -= 24;
    if (hour < 0) hour += 24;

      // Количество спутников
    uint32_t sat = 0;  
    if (gps.satellites.isValid()) {
        sat = gps.satellites.value();
    }
    
    char timeStr[20];
    sprintf(timeStr, "%02d:%02d:%02d (%d)", 
            hour, 
            gps.time.minute(), 
            gps.time.second(), 
            sat);
    return String(timeStr);
}

// В начало файла
#define GPS_BUFFER_SIZE 256
char gpsBuffer[GPS_BUFFER_SIZE];
int gpsBufIdx = 0;

void loop(void)
{

    while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.print(c);  // Отладка
        
        if (gpsBufIdx < GPS_BUFFER_SIZE - 1) {
            gpsBuffer[gpsBufIdx++] = c;
        }
    }

 // 2. Обрабатываем накопленные данные
    if (gpsBufIdx > 0) {
        for (int i = 0; i < gpsBufIdx; i++) {
            if (gps.encode(gpsBuffer[i])) {
                if (gps.location.isUpdated()) {

                    if (!bleConnected) {                        
                    
                        centerLat = gps.location.lat();
                        centerLng = gps.location.lng();
                        addTrackPoint(centerLat, centerLng);
                        angle = getLastSegmentAngle(); 
                        pointA = -1;
                        lastCoordTime = millis();
                        drawRoute();
                    }
                }
                if (gps.time.isValid()) {
                   if (!bleConnected) {
                     drawRoute();
                   }
                }
            }
        }
        gpsBufIdx = 0;  // Очищаем буфер
    }

   if (apStarted) {
     server8.handleClient();
   } 
   handleUartInput();

   // Проверяем, нужно ли перерисовать экран
   if (needRedraw()) {
     drawRoute();
   }

   

}
