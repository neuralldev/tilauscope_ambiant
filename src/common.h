// common header file

#include <esp_system.h> // Include this header at the top of main.cpp

// utility features 
inline uint8_t calculateChecksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}
