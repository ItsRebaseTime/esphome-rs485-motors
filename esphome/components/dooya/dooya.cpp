#include "dooya.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace esphome {
namespace dooya {

static const char *const TAG = "dooya.cover";

using namespace esphome::cover;

bool validate_crc(const std::vector<uint8_t> &frame) {
  size_t len = frame.size();
  return crc16(&frame[0], len - 2) == ((uint16_t) frame[len - 1] << 8 | frame[len - 2]);
}

bool parse_hex_byte(const std::string &value, uint8_t &out) {
  if (value.size() != 2)
    return false;
  if (!std::isxdigit(static_cast<unsigned char>(value[0])) || !std::isxdigit(static_cast<unsigned char>(value[1])))
    return false;
  char *end = nullptr;
  unsigned long parsed = std::strtoul(value.c_str(), &end, 16);
  if (end == nullptr || *end != '\0' || parsed > 0xFF)
    return false;
  out = static_cast<uint8_t>(parsed);
  return true;
}

std::vector<std::string> split_tokens(const std::string &value) {
  std::vector<std::string> tokens;
  std::string current;
  for (char c : value) {
    if (std::isspace(static_cast<unsigned char>(c)) || c == ',' || c == ';' || c == ':') {
      if (!current.empty()) {
        tokens.push_back(current);
        current.clear();
      }
      continue;
    }
    current.push_back(c);
  }
  if (!current.empty())
    tokens.push_back(current);
  return tokens;
}

std::string normalize_hex(const std::string &value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (std::isxdigit(static_cast<unsigned char>(c)))
      out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  }
  return out;
}

CoverTraits DooyaCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  return traits;
}

void DooyaCover::setup() {
  for (uint8_t read_type : {GET_STATUS, GET_POSITION, INVERT_DIRECTION, PULL_TO_START}) {
    uint8_t data[3] = {READ, read_type, 0x01};
    this->send_command(data, 3);
    this->read_requests.push({read_type, millis()});
  }
  this->publish_address_change_status_(
      "Idle. Enter target address as ID_L ID_H (for example: 42 43 or 4243), then press 'Change Address'.");
}

void DooyaCover::loop() {
  if (!this->read_requests.empty() && millis() - std::get<1>(this->read_requests.front()) > 1000)
    this->read_requests.pop();

  if (this->address_change_state_ == ADDRESS_CHANGE_ARMED && millis() - this->address_change_started_at_ > 120000) {
    this->address_change_state_ = ADDRESS_CHANGE_IDLE;
    this->publish_address_change_status_(
        "Armed state expired. Press 'Change Address' again after motor setup (hold setup button for 5s until LED flashes twice)."
    );
  } else if (this->address_change_state_ == ADDRESS_CHANGE_WAITING_RESPONSE &&
             millis() - this->address_change_started_at_ > 2000) {
    this->address_change_state_ = ADDRESS_CHANGE_IDLE;
    this->publish_address_change_status_(
        "No response from motor. Ensure exactly one motor is connected and press 'Change Address' again within 10s after LED flashes twice."
    );
  }
}

void DooyaCover::control(const CoverCall &call) {
  if (call.get_stop()) {
    uint8_t data[2] = {CONTROL, STOP};
    this->send_command(data, 2);
  } else if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if ((uint8_t) (pos * 100) != (uint8_t) (this->position * 100)) {
      if (pos == COVER_OPEN) {
        uint8_t data[2] = {CONTROL, OPEN};
        this->send_command(data, 2);
      } else if (pos == COVER_CLOSED) {
        uint8_t data[2] = {CONTROL, CLOSE};
        this->send_command(data, 2);
      } else {
        uint8_t data[3] = {CONTROL, SET_POSITION, (uint8_t) (pos * 100)};
        this->send_command(data, 3);
      }
    }
  }
}

void DooyaCover::send_update() {
  if (this->current_operation != COVER_OPERATION_IDLE) {
    for (uint8_t read_type : {GET_STATUS, GET_POSITION}) {
      uint8_t data[3] = {READ, read_type, 0x01};
      this->send_command(data, 3);
      this->read_requests.push({read_type, millis()});
    }
  }
}

void DooyaCover::on_uart_multi_byte(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  switch (at) {
    case 0:
      if (byte == START_CODE)
        this->rx_buffer_.push_back(byte);
      break;
    case 1:
      if (byte == this->address_[0] ||
          (this->address_change_state_ == ADDRESS_CHANGE_WAITING_RESPONSE && byte == this->address_change_pending_[0]))
        this->rx_buffer_.push_back(byte);
      else
        this->rx_buffer_.clear();
      break;
    case 2:
      if ((this->rx_buffer_[1] == this->address_[0] && byte == this->address_[1]) ||
          (this->address_change_state_ == ADDRESS_CHANGE_WAITING_RESPONSE && this->rx_buffer_[1] == this->address_change_pending_[0] &&
           byte == this->address_change_pending_[1]))
        this->rx_buffer_.push_back(byte);
      else
        this->rx_buffer_.clear();
      break;
    case 3:
      if (byte == READ || byte == WRITE || byte == CONTROL)
        this->rx_buffer_.push_back(byte);
      else
        this->rx_buffer_.clear();
      break;
    case 6:
      this->rx_buffer_.push_back(byte);
      if (this->rx_buffer_[3] == CONTROL && this->rx_buffer_[4] != SET_POSITION) {
        if (validate_crc(this->rx_buffer_))
          this->process_control_response_();
        else
          ESP_LOGE(TAG, "Incoming data CRC check failed");
        this->rx_buffer_.clear();
        this->parent_->ready_to_tx = true;
      }
      break;
    case 7:
      this->rx_buffer_.push_back(byte);
      if (validate_crc(this->rx_buffer_)) {
        switch (this->rx_buffer_[3]) {
          case READ:
            if (!this->read_requests.empty())
              this->process_read_response_();
            break;
          case WRITE:
            this->process_write_response_();
            break;
          case CONTROL:
            this->process_control_response_();
            break;
          default:
            ESP_LOGE(TAG, "Invalid response type received");
            break;
        }
      } else {
        ESP_LOGE(TAG, "Incoming data CRC check failed");
      }
      this->rx_buffer_.clear();
      this->parent_->ready_to_tx = true;
      break;
    default:
      this->rx_buffer_.push_back(byte);
      break;
  }
}

void DooyaCover::process_read_response_() {
  switch (std::get<0>(this->read_requests.front())) {
    case GET_POSITION:
      if (this->rx_buffer_[5] != UNKNOWN_POSITION) {
        if ((uint8_t) (this->position * 100) != this->rx_buffer_[5])
          this->position = clamp((float) this->rx_buffer_[5] / 100, 0.0f, 1.0f);
      } else {
        this->position = 0.5f;
      }
      this->publish_state(false);
#ifdef USE_BINARY_SENSOR
      if (this->positioning_binary_sensor_ != nullptr)
        this->positioning_binary_sensor_->publish_state(this->rx_buffer_[5] == UNKNOWN_POSITION);
#endif
      break;
#ifdef USE_SWITCH
    case INVERT_DIRECTION:
      if (this->invert_direction_switch_ != nullptr)
        this->invert_direction_switch_->publish_state(this->rx_buffer_[5] == 0x01);
      break;
    case PULL_TO_START:
      if (this->pull_to_start_switch_ != nullptr)
        this->pull_to_start_switch_->publish_state(this->rx_buffer_[5] == 0x00);
      break;
#endif
    case GET_STATUS:
      switch (this->rx_buffer_[5]) {
        case 0:
          if (this->current_operation != COVER_OPERATION_IDLE) {
            this->current_operation = COVER_OPERATION_IDLE;
            this->publish_state(false);
          }
          break;
        case 1:
          if (this->current_operation != COVER_OPERATION_OPENING) {
            this->current_operation = COVER_OPERATION_OPENING;
            this->publish_state(false);
          }
          break;
        case 2:
          if (this->current_operation != COVER_OPERATION_CLOSING) {
            this->current_operation = COVER_OPERATION_CLOSING;
            this->publish_state(false);
          }
          break;
        case 3:
          ESP_LOGW(TAG, "Device is in setting mode");
          break;
        default:
          ESP_LOGE(TAG, "Invalid status operation received");
          break;
      }
      break;
    default:
      ESP_LOGE(TAG, "Invalid read response received");
      break;
  }
  this->read_requests.pop();
}

void DooyaCover::process_write_response_() {
  if (this->address_change_state_ == ADDRESS_CHANGE_WAITING_RESPONSE && this->rx_buffer_[4] == 0x00 && this->rx_buffer_[5] == 0x02) {
    if (this->rx_buffer_[1] == this->address_change_pending_[0] && this->rx_buffer_[2] == this->address_change_pending_[1]) {
      this->address_[0] = this->address_change_pending_[0];
      this->address_[1] = this->address_change_pending_[1];
      this->address_change_state_ = ADDRESS_CHANGE_IDLE;
      this->publish_address_change_status_(str_sprintf(
          "Address changed successfully to 0x%02X%02X (ID_L=0x%02X, ID_H=0x%02X).",
          this->address_[0], this->address_[1], this->address_[0], this->address_[1]));
    }
    return;
  }

  switch (this->rx_buffer_[4]) {
#ifdef USE_SWITCH
    case INVERT_DIRECTION:
      if (this->invert_direction_switch_ != nullptr)
        this->invert_direction_switch_->publish_state(this->current_write_payload == 0x01);
      break;
    case PULL_TO_START:
      if (this->pull_to_start_switch_ != nullptr)
        this->pull_to_start_switch_->publish_state(this->current_write_payload == 0x00);
      break;
#endif
    default:
      ESP_LOGE(TAG, "Invalid write response received");
      break;
  }
}

void DooyaCover::process_control_response_() {
  switch (this->rx_buffer_[4]) {
    case OPEN:
      this->current_operation = COVER_OPERATION_OPENING;
      this->publish_state(false);
      break;
    case CLOSE:
      this->current_operation = COVER_OPERATION_CLOSING;
      this->publish_state(false);
      break;
    case STOP:
      this->current_operation = COVER_OPERATION_IDLE;
      this->publish_state(false);
      break;
    case SET_POSITION:
      if (this->rx_buffer_[5] != UNKNOWN_POSITION) {
        if (this->rx_buffer_[5] > (uint8_t) (this->position * 100))
          this->current_operation = COVER_OPERATION_OPENING;
        else
          this->current_operation = COVER_OPERATION_CLOSING;
        this->publish_state(false);
      }
#ifdef USE_BINARY_SENSOR
      if (this->positioning_binary_sensor_ != nullptr)
        this->positioning_binary_sensor_->publish_state(this->rx_buffer_[5] == UNKNOWN_POSITION);
#endif
      break;
    case CLEAR_POSITIONING:
      ESP_LOGI(TAG, "Positioning cleared");
      break;
    case FACTORY_RESET:
      ESP_LOGI(TAG, "Factory reset successful");
      break;
    default:
      ESP_LOGE(TAG, "Invalid control response received");
      break;
  }
}

void DooyaCover::send_command(const uint8_t *data, uint8_t len) {
  std::vector<uint8_t> frame = {START_CODE, this->address_[0], this->address_[1]};
  frame.insert(frame.end(), data, data + len);
  uint16_t crc = crc16(&frame[0], frame.size());
  frame.push_back(crc >> 0);
  frame.push_back(crc >> 8);

  this->send(frame);
}

bool DooyaCover::parse_address_change_input_(uint8_t &id_l, uint8_t &id_h, std::string &error) const {
#ifdef USE_TEXT
  if (this->address_change_text_ == nullptr) {
    error = "Address text entity is not configured";
    return false;
  }

  std::string input = this->address_change_text_->state;
  if (input.empty()) {
    error = "Address field is empty";
    return false;
  }

  auto tokens = split_tokens(input);
  std::vector<uint8_t> bytes;
  if (tokens.size() == 2) {
    bytes.reserve(tokens.size());
    for (auto token : tokens) {
      std::transform(token.begin(), token.end(), token.begin(),
                     [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
      if (token.rfind("0X", 0) == 0)
        token = token.substr(2);
      uint8_t parsed;
      if (!parse_hex_byte(token, parsed)) {
        error = "Address field contains invalid hex byte";
        return false;
      }
      bytes.push_back(parsed);
    }
  } else {
    auto compact = normalize_hex(input);
    if (compact.size() != 4) {
      error = "Expected ID_L+ID_H (4 hex chars)";
      return false;
    }
    for (size_t i = 0; i < compact.size(); i += 2) {
      uint8_t parsed;
      if (!parse_hex_byte(compact.substr(i, 2), parsed)) {
        error = "Address field contains invalid hex byte";
        return false;
      }
      bytes.push_back(parsed);
    }
  }

  if (bytes.size() == 2) {
    id_l = bytes[0];
    id_h = bytes[1];
  } else {
    error = "Unexpected address format";
    return false;
  }

  if (id_l == 0x00 || id_l == 0xFF || id_h == 0x00 || id_h == 0xFF) {
    error = "ID_L and ID_H cannot be 0x00 or 0xFF";
    return false;
  }
  return true;
#else
  error = "Text component support is not available";
  return false;
#endif
}

void DooyaCover::send_address_change_command_(uint8_t id_l, uint8_t id_h) {
  std::vector<uint8_t> frame = {START_CODE, 0x00, 0x00, WRITE, 0x00, 0x02, id_l, id_h};
  uint16_t crc = crc16(&frame[0], frame.size());
  frame.push_back(crc >> 0);
  frame.push_back(crc >> 8);
  this->send(frame);
}

void DooyaCover::publish_address_change_status_(const std::string &state) {
#ifdef USE_TEXT_SENSOR
  if (this->address_change_status_text_sensor_ != nullptr)
    this->address_change_status_text_sensor_->publish_state(state);
#else
  (void) state;
#endif
}

void DooyaCover::handle_change_address_button_press() {
  if (this->address_change_state_ == ADDRESS_CHANGE_WAITING_RESPONSE) {
    this->publish_address_change_status_("Waiting for motor response. Retry if timeout occurs.");
    return;
  }

  uint8_t id_l;
  uint8_t id_h;
  std::string error;
  if (!this->parse_address_change_input_(id_l, id_h, error)) {
    this->address_change_state_ = ADDRESS_CHANGE_IDLE;
    this->publish_address_change_status_(
        str_sprintf("Invalid address input: %s. Enter ID_L ID_H (for example: 42 43 or 4243).", error.c_str()));
    return;
  }

  if (this->address_change_state_ != ADDRESS_CHANGE_ARMED || this->address_change_pending_[0] != id_l ||
      this->address_change_pending_[1] != id_h) {
    this->address_change_pending_[0] = id_l;
    this->address_change_pending_[1] = id_h;
    this->address_change_state_ = ADDRESS_CHANGE_ARMED;
    this->address_change_started_at_ = millis();
    this->publish_address_change_status_(
        str_sprintf("Armed for ID_L=0x%02X ID_H=0x%02X. Power on motor, hold setup button for 5s until LED flashes twice, then press 'Change Address' again within 10 seconds to send.",
                    id_l, id_h));
    return;
  }

  this->send_address_change_command_(id_l, id_h);
  this->address_change_state_ = ADDRESS_CHANGE_WAITING_RESPONSE;
  this->address_change_started_at_ = millis();
  this->publish_address_change_status_(
      str_sprintf("Address change command sent for ID_L=0x%02X ID_H=0x%02X. Waiting for response...", id_l, id_h));
}

void DooyaCover::dump_config() {
  ESP_LOGCONFIG(TAG, "Dooya:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", this->address_[0], this->address_[1]);
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Positioning", this->positioning_binary_sensor_);
#endif
#ifdef USE_BUTTON
  LOG_BUTTON("  ", "Get Status Button", this->get_status_button_);
  LOG_BUTTON("  ", "Clear Positioning Button", this->clear_positioning_button_);
  LOG_BUTTON("  ", "Factory Reset Button", this->factory_reset_button_);
  LOG_BUTTON("  ", "Change Address Button", this->change_address_button_);
#endif
#ifdef USE_SWITCH
  LOG_SWITCH("  ", "Invert Direction Switch", this->invert_direction_switch_);
  LOG_SWITCH("  ", "Pull to Start Switch", this->pull_to_start_switch_);
#endif
#ifdef USE_TEXT
  LOG_TEXT("  ", "Address Text", this->address_change_text_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Address Change Status", this->address_change_status_text_sensor_);
#endif
}

}  // namespace dooya
}  // namespace esphome
