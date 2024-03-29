/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief hysteresis filter
 */

#ifndef MODULES_CONTROL_COMMON_HYSTERESIS_FILTER_H_
#define MODULES_CONTROL_COMMON_HYSTERESIS_FILTER_H_

/**
 * @namespace jmcauto::control
 * @brief jmcauto::control
 */
namespace jmcauto {
namespace control {

class HysteresisFilter {
 public:
  HysteresisFilter() = default;
  void filter(const double input_value, const double threshold,
              const double hysteresis_upper, const double hysteresis_lower,
              int *state, double *output_value);

 private:
  int previous_state_ = 0;
};

}  // namespace control
}  // namespace jmcauto
#endif  // MODULES_CONTROL_COMMON_HYSTERESIS_FILTER_H_
