#pragma once

#include <queue>
#include <string>

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/uart_multi/uart_multi.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_TEXT
#include "esphome/components/text/text.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

namespace esphome {
namespace dooya {

static const uint8_t START_CODE = 0x55;
static const uint8_t UNKNOWN_POSITION = 0xFF;

enum Command : uint8_t {
  READ = 0x01,
  WRITE = 0x02,
  CONTROL = 0x03,
};

enum ReadType : uint8_t {
  GET_POSITION = 0x02,
  INVERT_DIRECTION = 0x03,
  PULL_TO_START = 0x04,
  GET_STATUS = 0x05,
};

enum ControlType : uint8_t {
  OPEN = 0x01,
  CLOSE = 0x02,
  STOP = 0x03,
  SET_POSITION = 0x04,
  CLEAR_POSITIONING = 0x07,
  FACTORY_RESET = 0x08,
};

class DooyaCover : public cover::Cover, public Component, public uart_multi::UARTMultiDevice {
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(positioning)
#endif
#ifdef USE_BUTTON
  SUB_BUTTON(get_status)
  SUB_BUTTON(clear_positioning)
  SUB_BUTTON(factory_reset)
  SUB_BUTTON(change_address)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(address_change_status)
#endif
#ifdef USE_SWITCH
  SUB_SWITCH(invert_direction)
  SUB_SWITCH(pull_to_start)
#endif

 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_address(uint16_t address) {
    this->address_[0] = (uint8_t)(address >> 8);
    this->address_[1] = (uint8_t)(address & 0xFF);
  }
  void send_update() override;
  void on_uart_multi_byte(uint8_t byte) override;
  cover::CoverTraits get_traits() override;
  void send_command(const uint8_t *data, uint8_t len);
  void set_status_update_interval(uint32_t interval) { this->status_update_interval_ = interval; }
  void handle_change_address_button_press();
#ifdef USE_TEXT
  void set_address_change_text(text::Text *text) { this->address_change_text_ = text; }
#endif
  std::queue<std::tuple<uint8_t, uint32_t>> read_requests;
  uint8_t current_write_payload;

 protected:
  void control(const cover::CoverCall &call) override;
  void process_read_response_();
  void process_write_response_();
  void process_control_response_();
  void request_status_and_position_();
  bool parse_address_change_input_(uint8_t &id_l, uint8_t &id_h, std::string &error) const;
  void send_address_change_command_(uint8_t id_l, uint8_t id_h);
  void publish_address_change_status_(const std::string &state);

  enum AddressChangeState : uint8_t {
    ADDRESS_CHANGE_IDLE = 0,
    ADDRESS_CHANGE_ARMED,
    ADDRESS_CHANGE_WAITING_RESPONSE,
  };

  uint8_t address_[2] = {0xFE, 0xFE};
  std::vector<uint8_t> rx_buffer_;
  float target_position_;
  uint32_t status_update_interval_{10000};
  uint32_t last_status_update_{0};
  AddressChangeState address_change_state_{ADDRESS_CHANGE_IDLE};
  uint8_t address_change_pending_[2] = {0x00, 0x00};
  uint32_t address_change_started_at_{0};
#ifdef USE_TEXT
  text::Text *address_change_text_{nullptr};
#endif
};

}  // namespace dooya
}  // namespace esphome
