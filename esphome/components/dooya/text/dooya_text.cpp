#include "dooya_text.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dooya {

static const char *const TAG = "dooya.text";

void DooyaAddressText::control(const std::string &value) {
  ESP_LOGD(TAG, "Address text updated: %s", value.c_str());
  this->publish_state(value);
}

}  // namespace dooya
}  // namespace esphome
