#pragma once

#include <pix_rover_driver/Byte.hpp>
#include <iostream>

class LinVelXy {
public:
    static const uint32_t ID = 0xE;
    LinVelXy();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int lin_vel_x_;
    int lin_vel_y_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 32, 'name': 'lin_vel_x', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'm/s', 'precision': 1.0, 'type': 'int'}
  int linvelx();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 32, 'name': 'lin_vel_y', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'm/s', 'precision': 1.0, 'type': 'int'}
  int linvely();
};



