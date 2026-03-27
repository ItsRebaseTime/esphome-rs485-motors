#pragma once

#include "esphome/components/text/text.h"
#include "esphome/core/component.h"

#include "../dooya.h"

namespace esphome {
namespace dooya {

class DooyaAddressText : public text::Text, public Component, public Parented<DooyaCover> {
 public:
  void control(const std::string &value) override;
};

}  // namespace dooya
}  // namespace esphome
