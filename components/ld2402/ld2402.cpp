#include "ld2402.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include <cctype>
#include <strings.h>

/*
Configure commands - little endian

No command can exceed 64 bytes, otherwise they would need be to be split up into multiple sends.

All send command frames will have:
  Header = FD FC FB FA, Bytes 0 - 3, uint32_t 0xFAFBFCFD
  Length, bytes 4 - 5, uint16_t 0x0002, must be at least 2 for the command byte if no addon data.
  Command bytes 6 - 7, uint16_t
  Footer = 04 03 02 01 - uint32_t 0x01020304, Always last 4 Bytes.
Receive
  Error bytes 8-9 uint16_t, 0 = success, all other positive values = error

Enable config mode:
Send:
  UART Tx: FD FC FB FA 04 00 FF 00 02 00 04 03 02 01
  Command = FF 00 - uint16_t 0x00FF
  Protocol version = 02 00, can be 1 or 2 - uint16_t 0x0002
Reply:
  UART Rx: FD FC FB FA 08 00 FF 01 00 00 02 00 20 00 04 03 02 01

Disable config mode:
Send:
  UART Tx: FD FC FB FA 02 00 FE 00 04 03 02 01
  Command = FE 00 - uint16_t 0x00FE
Receive:
  UART Rx: FD FC FB FA 04 00 FE 01 00 00 04 03 02 01

Configure system parameters:

UART Tx: FD FC FB FA 08 00 12 00 00 00 64 00 00 00 04 03 02 01  Set system parms
Command = 12 00 - uint16_t 0x0012, Param
There are three documented parameters for modes:
  00 64 = Basic text mode (default)
    No target: "OFF" + CRLF (4F 46 46 0D 0A)
    Target:    "distance:" + decimal cm + CRLF (e.g. distance:436)
  00 04 = Energy output mode (not used)


Configure gate sensitivity parameters:
UART Tx: FD FC FB FA 0E 00 07 00 10 00 60 EA 00 00 20 00 60 EA 00 00 04 03 02 01
Command = 12 00 - uint16_t 0x0007
Gate 0 high thresh = 10 00 uint16_t 0x0010, Threshold value = 60 EA 00 00 uint32_t 0x0000EA60
Gate 0 low thresh = 20 00 uint16_t 0x0020, Threshold value = 60 EA 00 00 uint32_t 0x0000EA60
*/

namespace esphome::ld2402 {

static const char *const TAG = "ld2402";

// Local const's
static constexpr uint16_t REFRESH_RATE_MS = 1000;

// Command sets
static constexpr uint16_t CMD_DISABLE_CONF = 0x00FE;
static constexpr uint16_t CMD_ENABLE_CONF = 0x00FF;
static constexpr uint16_t CMD_PROTOCOL_VER = 0x0002;
static constexpr uint16_t CMD_READ_ABD_PARAM = 0x0008;
static constexpr uint16_t CMD_READ_VERSION = 0x0000;
static constexpr uint16_t CMD_SYSTEM_MODE = 0x0000;
static constexpr uint16_t CMD_SYSTEM_MODE_BASIC = 0x0064;
static constexpr uint16_t CMD_WRITE_ABD_PARAM = 0x0007;
static constexpr uint16_t CMD_WRITE_SYS_PARAM = 0x0012;
static constexpr uint16_t CMD_AUTO_THRESHOLD_START = 0x0009;
static constexpr uint16_t CMD_AUTO_THRESHOLD_PROGRESS = 0x000A;
static constexpr uint16_t CMD_AUTO_THRESHOLD_INTERFERENCE = 0x0014;
static constexpr uint16_t CMD_SAVE_PARAMS = 0x00FD;
static constexpr uint32_t CALIBRATION_POLL_INTERVAL_MS = 5000;

static constexpr uint8_t CMD_ABD_DATA_REPLY_SIZE = 0x04;
static constexpr uint8_t CMD_ABD_DATA_REPLY_START = 0x0A;
static constexpr uint8_t CMD_MAX_BYTES = 0x64;

static constexpr uint8_t LD2402_ERROR_TIMEOUT = 0x02;
static constexpr uint8_t LD2402_ERROR_UNKNOWN = 0x01;

// Register address values
static constexpr uint16_t CMD_MAX_GATE_REG = 0x0001;
static constexpr uint16_t CMD_TIMEOUT_REG = 0x0004;
static constexpr uint16_t CMD_GATE_MOVE_THRESH[TOTAL_GATES] = {0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015,
                                                               0x0016, 0x0017, 0x0018, 0x0019, 0x001A, 0x001B,
                                                               0x001C, 0x001D, 0x001E, 0x001F};
// Motion-low / hold threshold parameter IDs (0x0020–0x002F); not used by current write path yet.
static constexpr uint16_t CMD_GATE_HOLD_THRESH[TOTAL_GATES] = {0x0020, 0x0021, 0x0022, 0x0023, 0x0024, 0x0025,
                                                               0x0026, 0x0027, 0x0028, 0x0029, 0x002A, 0x002B,
                                                               0x002C, 0x002D, 0x002E, 0x002F};
static constexpr uint16_t CMD_GATE_STILL_THRESH[TOTAL_GATES] = {0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035,
                                                                0x0036, 0x0037, 0x0038, 0x0039, 0x003A, 0x003B,
                                                                0x003C, 0x003D, 0x003E, 0x003F};
//static constexpr uint32_t FACTORY_MOVE_THRESH[TOTAL_GATES] = {89125, 50119, 6310, 2512, 1995, 1995, 1995, 1995, 1995,  1995,  1995, 1995, 1995, 1995, 1995, 1995};
//static constexpr uint32_t FACTORY_STILL_THRESH[TOTAL_GATES] = {158489, 63096, 10000, 3162, 3162, 2512, 1995, 1585, 1585,   1585,  1585,  1585, 1585, 1585, 1585, 1585};

static constexpr uint32_t FACTORY_MOVE_THRESH[TOTAL_GATES] = {141254, 79433, 100000, 3981, 3162, 3162, 3162, 3162, 3162,  3162,  3162, 3162, 3162, 3162, 3162, 3162};
static constexpr uint32_t FACTORY_STILL_THRESH[TOTAL_GATES] = {251189, 100000, 15849, 5012, 5012, 3981, 3162, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512};
static constexpr uint16_t FACTORY_TIMEOUT = 30;
static constexpr uint16_t FACTORY_MAX_GATE = 85;

// COMMAND_BYTE Header & Footer
static constexpr uint32_t CMD_FRAME_FOOTER = 0x01020304;
static constexpr uint32_t CMD_FRAME_HEADER = 0xFAFBFCFD;
static constexpr uint32_t BUFFER_FULL_LOG_INTERVAL_MS = 10000;
static constexpr uint8_t CMD_FRAME_COMMAND = 6;
static constexpr uint8_t CMD_FRAME_DATA_LENGTH = 4;
static constexpr uint8_t CMD_ERROR_WORD = 8;

static constexpr const char *const ERR_MESSAGE[] = {
    "None",
    "Unknown",
    "Timeout",
};

static uint8_t calc_checksum(void *data, size_t size) {
  uint8_t checksum = 0;
  uint8_t *data_bytes = (uint8_t *) data;
  for (size_t i = 0; i < size; i++) {
    checksum ^= data_bytes[i];  // XOR operation
  }
  return checksum;
}

float LD2402Component::get_setup_priority() const { return setup_priority::BUS; }

void LD2402Component::dump_config() {
  ESP_LOGCONFIG(TAG,
                "LD2402:\n"
                "  Firmware version: %7s\n"
                "  Output mode: basic text\n"
                "  Presence timeout: %u s\n"
                "  Max gate distance: %u",
                this->firmware_ver_, this->config_presence_timeout_, this->config_max_gate_distance_);
#ifdef USE_NUMBER
  ESP_LOGCONFIG(TAG, "Number:");
  LOG_NUMBER("  ", "Gate Timeout:", this->gate_timeout_number_);
  LOG_NUMBER("  ", "Gate Max Distance:", this->max_gate_distance_number_);
  LOG_NUMBER("  ", "Gate Select:", this->gate_select_number_);
  for (uint8_t gate = 0; gate < TOTAL_GATES; gate++) {
    LOG_NUMBER("  ", "Gate Move Threshold:", this->gate_move_threshold_numbers_[gate]);
    LOG_NUMBER("  ", "Gate Still Threshold:", this->gate_still_threshold_numbers_[gate]);
  }
#endif
#ifdef USE_BUTTON
  LOG_BUTTON("  ", "Apply & Save Config:", this->apply_config_button_);
  LOG_BUTTON("  ", "Auto Calibrate:", this->auto_calibrate_button_);
  LOG_BUTTON("  ", "Factory Reset:", this->factory_reset_button_);
#endif
}

void LD2402Component::setup() {
  delay(500);
  this->drain_uart_();

  uint8_t error = LD2402_ERROR_TIMEOUT;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(500);
      this->drain_uart_();
    }
    error = this->set_config_mode(true);
    if (error == 0) {
      break;
    }
    ESP_LOGW(TAG, "Config mode enable attempt %u timed out", attempt + 1);
  }
  if (error != 0) {
    ESP_LOGE(TAG, "LD2402 config mode enable failed after retries");
    this->mark_failed();
    return;
  }
  this->get_max_distance_and_timeout_();
  this->get_firmware_version_();
  const char *pfw = this->firmware_ver_;
  std::string fw_str(pfw);

  for (auto &listener : this->listeners_) {
    listener->on_fw_version(fw_str);
  }

  for (uint8_t gate = 0; gate < TOTAL_GATES; gate++) {
    delay_microseconds_safe(125);
    this->get_gate_threshold_(gate);
  }

  this->current_config.max_gate = this->config_max_gate_distance_;
  this->current_config.timeout = this->config_presence_timeout_;
  this->set_max_distance_and_timeout(this->config_max_gate_distance_, this->config_presence_timeout_);

  memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
  this->set_mode_(CMD_SYSTEM_MODE_BASIC);
#ifdef USE_NUMBER
  this->init_gate_config_numbers();
#endif
  this->set_system_mode(this->system_mode_);
  this->set_config_mode(false);
}

void LD2402Component::apply_config_action() {
  const uint8_t checksum = calc_checksum(&this->new_config, sizeof(this->new_config));
  if (checksum == calc_checksum(&this->current_config, sizeof(this->current_config))) {
    ESP_LOGD(TAG, "No configuration change detected");
    return;
  }
  ESP_LOGD(TAG, "Applying and saving configuration");
  if (this->set_config_mode(true) == LD2402_ERROR_TIMEOUT) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
    return;
  }
  this->set_max_distance_and_timeout(this->new_config.max_gate, this->new_config.timeout);
  for (uint8_t gate = 0; gate < TOTAL_GATES; gate++) {
    delay_microseconds_safe(125);
    this->set_gate_threshold(gate);
  }
  memcpy(&current_config, &new_config, sizeof(new_config));
#ifdef USE_NUMBER
  this->init_gate_config_numbers();
#endif
  this->set_system_mode(this->system_mode_);
  if (this->save_params_to_flash_() != 0) {
    ESP_LOGE(TAG, "Failed to save parameters to flash");
  }
  this->set_config_mode(false);
}

void LD2402Component::factory_reset_action() {
  ESP_LOGD(TAG, "Setting factory defaults");
  if (this->set_config_mode(true) == LD2402_ERROR_TIMEOUT) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
    return;
  }
  this->set_max_distance_and_timeout(FACTORY_MAX_GATE, FACTORY_TIMEOUT);
#ifdef USE_NUMBER
  this->gate_timeout_number_->state = FACTORY_TIMEOUT;
  this->max_gate_distance_number_->state = FACTORY_MAX_GATE;
#endif
  for (uint8_t gate = 0; gate < TOTAL_GATES; gate++) {
    this->new_config.move_thresh[gate] = FACTORY_MOVE_THRESH[gate];
    this->new_config.still_thresh[gate] = FACTORY_STILL_THRESH[gate];
    delay_microseconds_safe(125);
    this->set_gate_threshold(gate);
  }
  memcpy(&this->current_config, &this->new_config, sizeof(this->new_config));
  this->set_system_mode(this->system_mode_);
  this->set_config_mode(false);
#ifdef USE_NUMBER
  this->init_gate_config_numbers();
  this->refresh_gate_config_numbers();
#endif
}

void LD2402Component::auto_calibrate_action() {
  if (this->calibrating_) {
    ESP_LOGW(TAG, "Auto-calibration already in progress");
    return;
  }
  ESP_LOGI(TAG, "Starting auto-threshold calibration (trigger=%.1f hold=%.1f micro=%.1f)",
           this->auto_trigger_coefficient_, this->auto_hold_coefficient_, this->auto_micro_coefficient_);
  if (this->set_config_mode(true) == LD2402_ERROR_TIMEOUT) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
    return;
  }
  if (this->start_auto_threshold_() != 0) {
    ESP_LOGE(TAG, "Failed to start auto-threshold calibration");
    this->set_config_mode(false);
    return;
  }
  this->calibrating_ = true;
  this->calibration_interference_ = false;
  this->calibration_progress_ = 0;
  this->last_calibration_poll_ms_ = 0;
  this->publish_calibration_progress_(0);
}

void LD2402Component::loop() {
  // If there is a active send command do not process it here, the send command call will handle it.
  if (this->cmd_active_) {
    return;
  }
  this->read_batch_(this->buffer_data_);
  this->poll_auto_calibration_();
}

namespace {

const char *trim_line(const char *line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  return line;
}

const char *first_digit(const char *line) {
  for (; *line != '\0'; line++) {
    if (isdigit(static_cast<unsigned char>(*line))) {
      return line;
    }
  }
  return nullptr;
}

}  // namespace

void LD2402Component::process_text_lines_(uint8_t *buffer, uint8_t &buffer_pos) {
  while (buffer_pos > 0) {
    void *newline = memchr(buffer, '\n', buffer_pos);
    if (newline == nullptr) {
      return;
    }

    auto *line_end = static_cast<uint8_t *>(newline);
    *line_end = '\0';
    if (line_end > buffer && line_end[-1] == '\r') {
      line_end[-1] = '\0';
    }

    this->handle_text_line_(reinterpret_cast<char *>(buffer));

    const uint8_t consumed = static_cast<uint8_t>((line_end - buffer) + 1);
    const uint8_t remainder = buffer_pos - consumed;
    if (remainder > 0) {
      memmove(buffer, &buffer[consumed], remainder);
    }
    buffer_pos = remainder;
    buffer[buffer_pos] = '\0';
  }
}

void LD2402Component::handle_text_line_(const char *line) {
  line = trim_line(line);
  if (*line == '\0') {
    return;
  }

  if (static_cast<uint8_t>(line[0]) < 0x20) {
    return;
  }

  ESP_LOGV(TAG, "Text: %s", line);

  auto publish_presence = [this](bool presence) {
    if (this->presence_ == presence) {
      return;
    }
    this->set_presence_(presence);
    for (auto &listener : this->listeners_) {
      listener->on_presence(presence);
    }
  };

  // Factory basic mode: "OFF" (4F 46 46 0D 0A)
  if (strcasecmp(line, "OFF") == 0) {
    publish_presence(false);
    return;
  }

  // Factory basic mode: "distance:436" (64 69 73 74 61 6E 63 65 3A + digits + 0D 0A)
  if (strncasecmp(line, "distance", 8) != 0) {
    return;
  }

  const char *num = first_digit(line + 8);
  if (num == nullptr) {
    return;
  }

  const optional<uint16_t> distance = parse_number<uint16_t>(num);
  if (!distance.has_value()) {
    return;
  }

  publish_presence(true);
  this->set_distance_(distance.value());

  const int32_t current_millis = App.get_loop_component_start_time();
  if (current_millis - this->last_periodic_millis >= REFRESH_RATE_MS) {
    this->last_periodic_millis = current_millis;
    for (auto &listener : this->listeners_) {
      listener->on_distance(this->get_distance_());
    }
  }
}

void LD2402Component::trim_or_hunt_cmd_header_(uint8_t *buffer, uint8_t &buffer_pos, uint8_t search_from) {
  for (uint8_t i = search_from; i + 3 < buffer_pos; i++) {
    if (buffer[i] != 0xFD) {
      continue;
    }
    if (memcmp(&buffer[i], &CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER)) != 0) {
      continue;
    }
    const uint8_t remainder = buffer_pos - i;
    memmove(buffer, &buffer[i], remainder);
    buffer_pos = remainder;
    buffer[buffer_pos] = 0;
    return;
  }

  const uint8_t keep = std::min<uint8_t>(buffer_pos, 3);
  if (keep > 0 && keep < buffer_pos) {
    memmove(buffer, &buffer[buffer_pos - keep], keep);
  }
  buffer_pos = keep;
  buffer[buffer_pos] = 0;
}

void LD2402Component::drain_uart_() {
  while (this->available()) {
    this->read();
  }
  this->detection_buffer_pos_ = 0;
  this->cmd_buffer_pos_ = 0;
}

bool LD2402Component::validate_cmd_frame_(const uint8_t *buffer, int len) const {
  if (len < 12) {
    return false;
  }
  if (memcmp(buffer, &CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER)) != 0) {
    return false;
  }
  uint16_t data_len;
  memcpy(&data_len, &buffer[CMD_FRAME_DATA_LENGTH], sizeof(data_len));
  if (data_len < 2 || data_len > CMD_MAX_BYTES) {
    return false;
  }
  const int expected_len = 4 + 2 + data_len + 4;
  return len == expected_len;
}

void LD2402Component::resync_buffer_(uint8_t *buffer, uint8_t &buffer_pos) {
  if (buffer_pos < 4) {
    return;
  }

  for (uint8_t i = 0; i + 3 < buffer_pos; i++) {
    if (buffer[i] == '\n') {
      const uint8_t remainder = buffer_pos - (i + 1);
      if (remainder > 0) {
        memmove(buffer, &buffer[i + 1], remainder);
      }
      buffer_pos = remainder;
      buffer[buffer_pos] = '\0';
      return;
    }
    if (buffer[i] == 0xFD && memcmp(&buffer[i], &CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER)) == 0) {
      if (i == 0) {
        return;
      }
      const uint8_t remainder = buffer_pos - i;
      memmove(buffer, &buffer[i], remainder);
      buffer_pos = remainder;
      buffer[buffer_pos] = '\0';
      return;
    }
  }

  const uint8_t keep = std::min<uint8_t>(buffer_pos, 3);
  if (keep > 0 && keep < buffer_pos) {
    memmove(buffer, &buffer[buffer_pos - keep], keep);
  }
  buffer_pos = keep;
  buffer[buffer_pos] = '\0';
}

void LD2402Component::append_rx_byte_(int rx_data, uint8_t *buffer, int len, uint8_t &buffer_pos) {
  if (buffer_pos >= len - 1) {
    const uint32_t now = millis();
    if (now - this->last_buffer_full_log_ms_ >= BUFFER_FULL_LOG_INTERVAL_MS) {
      this->last_buffer_full_log_ms_ = now;
      ESP_LOGW(TAG, "UART buffer full (%u bytes); resyncing", buffer_pos);
    }
    if (this->cmd_active_) {
      this->trim_or_hunt_cmd_header_(buffer, buffer_pos, 1);
    } else {
      this->process_text_lines_(buffer, buffer_pos);
    }
    if (buffer_pos >= len - 1) {
      buffer_pos = 0;
      buffer[0] = '\0';
    }
  }
  buffer[buffer_pos++] = static_cast<uint8_t>(rx_data);
  buffer[buffer_pos] = '\0';
}

void LD2402Component::readline_(int rx_data, uint8_t *buffer, int len, uint8_t &buffer_pos) {
  if (rx_data < 0) {
    return;
  }

  this->append_rx_byte_(rx_data, buffer, len, buffer_pos);

  if (buffer_pos >= 4 && memcmp(&buffer[buffer_pos - 4], &CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER)) == 0) {
    if (this->validate_cmd_frame_(buffer, buffer_pos)) {
      uint16_t cmd_word;
      memcpy(&cmd_word, &buffer[CMD_FRAME_COMMAND], sizeof(cmd_word));
      if (cmd_word == CMD_AUTO_THRESHOLD_INTERFERENCE) {
        this->handle_calibration_interference_report_(buffer, buffer_pos);
        buffer_pos = 0;
        return;
      }
      this->cmd_active_ = false;
      this->handle_ack_data_(buffer, buffer_pos);
      buffer_pos = 0;
    } else {
      ESP_LOGW(TAG, "Misaligned command frame (%u bytes); resyncing", buffer_pos);
      this->resync_buffer_(buffer, buffer_pos);
    }
    return;
  }

  if (this->cmd_active_) {
    this->trim_or_hunt_cmd_header_(buffer, buffer_pos, 0);
    return;
  }

  this->process_text_lines_(buffer, buffer_pos);
}

void LD2402Component::read_batch_(std::span<uint8_t, MAX_LINE_LENGTH> buffer) {
  // Read all available bytes in batches to reduce UART call overhead.
  size_t avail = this->available();
  uint8_t buf[MAX_LINE_LENGTH];
  while (avail > 0) {
    size_t to_read = std::min(avail, sizeof(buf));
    if (!this->read_array(buf, to_read)) {
      break;
    }
    avail -= to_read;

    for (size_t i = 0; i < to_read; i++) {
      this->readline_(buf[i], buffer.data(), buffer.size(), this->detection_buffer_pos_);
    }
  }
}

void LD2402Component::handle_ack_data_(uint8_t *buffer, int len) {
  memcpy(&this->cmd_reply_.command, &buffer[CMD_FRAME_COMMAND], sizeof(this->cmd_reply_.command));
  this->cmd_reply_.length = buffer[CMD_FRAME_DATA_LENGTH];
  uint16_t data_pos = 0;
  if (this->cmd_reply_.length > CMD_MAX_BYTES) {
    ESP_LOGW(TAG, "Reply frame too long");
    return;
  } else if (this->cmd_reply_.length < 2) {
    ESP_LOGW(TAG, "Command frame too short");
    return;
  }
  memcpy(&this->cmd_reply_.error, &buffer[CMD_ERROR_WORD], sizeof(this->cmd_reply_.error));
  const char *result = this->cmd_reply_.error ? "failure" : "success";
  this->cmd_reply_.ack = true;
  if (this->cmd_reply_.error > 0) {
    return;
  }
  const uint8_t cmd_base = this->cmd_reply_.command & 0xFF;
  switch (cmd_base) {
    case 0xFF:
      ESP_LOGV(TAG, "Set config enable: CMD = %2X %s", CMD_ENABLE_CONF, result);
      break;
    case 0xFE:
      ESP_LOGV(TAG, "Set config disable: CMD = %2X %s", CMD_DISABLE_CONF, result);
      break;
    case 0x07:
      ESP_LOGV(TAG, "Write gate parameter(s): %2X %s", CMD_WRITE_ABD_PARAM, result);
      break;
    case 0x08: {
      ESP_LOGV(TAG, "Read gate parameter(s): %2X %s", CMD_READ_ABD_PARAM, result);
      data_pos = CMD_ABD_DATA_REPLY_START;
      uint16_t abd_count = std::min<uint16_t>((buffer[CMD_FRAME_DATA_LENGTH] - 4) / CMD_ABD_DATA_REPLY_SIZE,
                                              sizeof(this->cmd_reply_.data) / sizeof(this->cmd_reply_.data[0]));
      for (uint16_t i = 0; i < abd_count; i++) {
        memcpy(&this->cmd_reply_.data[i], &buffer[data_pos + i * CMD_ABD_DATA_REPLY_SIZE],
               sizeof(this->cmd_reply_.data[i]));
      }
      break;
    }
    case 0x12:
      ESP_LOGV(TAG, "Set system parameter(s): %2X %s", CMD_WRITE_SYS_PARAM, result);
      break;
    case 0x00: {
      uint8_t ver_len = std::min<uint8_t>(buffer[10], sizeof(this->firmware_ver_) - 1);
      memcpy(this->firmware_ver_, &buffer[12], ver_len);
      this->firmware_ver_[ver_len] = '\0';
      ESP_LOGV(TAG, "Firmware version: %s %s", this->firmware_ver_, result);
      break;
    }
    case 0x09:
      ESP_LOGI(TAG, "Auto-threshold generation started");
      break;
    case 0x0A: {
      const int progress = this->parse_calibration_progress_(buffer, len);
      if (progress >= 0) {
        this->cmd_reply_.ack_value = static_cast<uint16_t>(progress);
        ESP_LOGI(TAG, "Auto-threshold progress: %d%%", progress);
      }
      break;
    }
    case 0xFD:
      ESP_LOGI(TAG, "Parameters saved to radar flash");
      break;
    default:
      break;
  }
}

int LD2402Component::write_cmd_frame_(CmdFrameT frame) {
  uint8_t cmd_buffer[MAX_LINE_LENGTH];
  frame.length = 0;
  const uint16_t frame_data_bytes = frame.data_length + 2;

  memcpy(&cmd_buffer[frame.length], &frame.header, sizeof(frame.header));
  frame.length += sizeof(frame.header);

  memcpy(&cmd_buffer[frame.length], &frame_data_bytes, sizeof(frame.data_length));
  frame.length += sizeof(frame.data_length);

  memcpy(&cmd_buffer[frame.length], &frame.command, sizeof(frame.command));
  frame.length += sizeof(frame.command);

  for (uint16_t index = 0; index < frame.data_length; index++) {
    memcpy(&cmd_buffer[frame.length], &frame.data[index], sizeof(frame.data[index]));
    frame.length += sizeof(frame.data[index]);
  }

  memcpy(cmd_buffer + frame.length, &frame.footer, sizeof(frame.footer));
  frame.length += sizeof(frame.footer);
  this->write_array(cmd_buffer, frame.length);
  return 0;
}

int LD2402Component::send_cmd_from_array(CmdFrameT frame) {
  uint32_t start_millis = millis();
  uint8_t error = 0;
  uint8_t ack_buffer[MAX_LINE_LENGTH];
  this->cmd_reply_.ack = false;
  this->cmd_reply_.ack_value = 0;
  this->cmd_buffer_pos_ = 0;
  this->cmd_active_ = true;
  uint8_t retry = 3;
  while (retry) {
    this->write_cmd_frame_(frame);

    error = 0;
    while (!this->cmd_reply_.ack) {
      while (this->available()) {
        this->readline_(this->read(), ack_buffer, sizeof(ack_buffer), this->cmd_buffer_pos_);
      }
      delay_microseconds_safe(1450);
      // Wait on an Rx from the LD2402 for up to 3 1 second loops, otherwise it could trigger a WDT.
      if ((millis() - start_millis) > 1000) {
        start_millis = millis();
        error = LD2402_ERROR_TIMEOUT;
        retry--;
        break;
      }
    }
    if (this->cmd_reply_.ack) {
      retry = 0;
      if (this->cmd_reply_.error > 0) {
        ESP_LOGE(TAG, "Command failed: device error 0x%04X", this->cmd_reply_.error);
        error = LD2402_ERROR_UNKNOWN;
      }
    }
  }
  if (error == LD2402_ERROR_TIMEOUT) {
    this->handle_cmd_error(error);
  }
  if (!this->cmd_reply_.ack) {
    this->cmd_active_ = false;
    this->cmd_buffer_pos_ = 0;
  }
  return error;
}

uint8_t LD2402Component::set_config_mode(bool enable) {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = enable ? CMD_ENABLE_CONF : CMD_DISABLE_CONF;
  if (enable) {
    memcpy(&cmd_frame.data[0], &CMD_PROTOCOL_VER, sizeof(CMD_PROTOCOL_VER));
    cmd_frame.data_length += sizeof(CMD_PROTOCOL_VER);
  }
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG, "Sending set config %s command: %2X", enable ? "enable" : "disable", cmd_frame.command);
  return this->send_cmd_from_array(cmd_frame);
}

void LD2402Component::handle_cmd_error(uint8_t error) {
  const size_t n = sizeof(ERR_MESSAGE) / sizeof(ERR_MESSAGE[0]);
  if (error < n) {
    ESP_LOGE(TAG, "Command transport failed: %s", ERR_MESSAGE[error]);
  } else {
    ESP_LOGE(TAG, "Command transport failed: code %u", error);
  }
}

int LD2402Component::get_gate_threshold_(uint8_t gate) {
  uint8_t error;
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_READ_ABD_PARAM;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_GATE_MOVE_THRESH[gate], sizeof(CMD_GATE_MOVE_THRESH[gate]));
  cmd_frame.data_length += 2;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_GATE_STILL_THRESH[gate], sizeof(CMD_GATE_STILL_THRESH[gate]));
  cmd_frame.data_length += 2;
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG, "Sending read gate %d high/low threshold command: %2X", gate, cmd_frame.command);
  error = this->send_cmd_from_array(cmd_frame);
  if (error == 0) {
    this->current_config.move_thresh[gate] = cmd_reply_.data[0];
    this->current_config.still_thresh[gate] = cmd_reply_.data[1];
  }
  return error;
}

int LD2402Component::get_max_distance_and_timeout_() {
  uint8_t error;
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_READ_ABD_PARAM;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_MAX_GATE_REG, sizeof(CMD_MAX_GATE_REG));
  cmd_frame.data_length += sizeof(CMD_MAX_GATE_REG);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_TIMEOUT_REG, sizeof(CMD_TIMEOUT_REG));
  cmd_frame.data_length += sizeof(CMD_TIMEOUT_REG);
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG, "Sending read max distance and timeout parameters: %2X", cmd_frame.command);
  error = this->send_cmd_from_array(cmd_frame);
  if (error == 0) {
    this->current_config.max_gate = (uint16_t) cmd_reply_.data[0];
    this->current_config.timeout = (uint16_t) cmd_reply_.data[1];
  }
  return error;
}

void LD2402Component::set_system_mode(uint16_t mode) {
  CmdFrameT cmd_frame;
  uint16_t unknown_parm = 0x0000;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_WRITE_SYS_PARAM;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_SYSTEM_MODE, sizeof(CMD_SYSTEM_MODE));
  cmd_frame.data_length += sizeof(CMD_SYSTEM_MODE);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &mode, sizeof(mode));
  cmd_frame.data_length += sizeof(mode);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &unknown_parm, sizeof(unknown_parm));
  cmd_frame.data_length += sizeof(unknown_parm);
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG, "Sending write system mode command: %2X", cmd_frame.command);
  if (this->send_cmd_from_array(cmd_frame) == 0) {
    this->set_mode_(mode);
  }
}

void LD2402Component::get_firmware_version_() {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_READ_VERSION;
  cmd_frame.footer = CMD_FRAME_FOOTER;

  ESP_LOGV(TAG, "Sending read firmware version command: %2X", cmd_frame.command);
  this->send_cmd_from_array(cmd_frame);
}

void LD2402Component::set_max_distance_and_timeout(uint32_t max_gate_distance, uint32_t timeout) {
  // CMD_WRITE_ABD_PARAM (0x0007): max distance (reg 0x0001), disappearance delay / timeout (reg 0x0004).
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_WRITE_ABD_PARAM;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_MAX_GATE_REG, sizeof(CMD_MAX_GATE_REG));
  cmd_frame.data_length += sizeof(CMD_MAX_GATE_REG);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &max_gate_distance, sizeof(max_gate_distance));
  cmd_frame.data_length += sizeof(max_gate_distance);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_TIMEOUT_REG, sizeof(CMD_TIMEOUT_REG));
  cmd_frame.data_length += sizeof(CMD_TIMEOUT_REG);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &timeout, sizeof(timeout));
  cmd_frame.data_length += sizeof(timeout);
  cmd_frame.footer = CMD_FRAME_FOOTER;

  ESP_LOGV(TAG, "Sending write max distance and timeout: %2X", cmd_frame.command);
  this->send_cmd_from_array(cmd_frame);
}

void LD2402Component::set_gate_threshold(uint8_t gate) {
  // Header H, Length L, Command C, Register R, Value V, Footer F
  // HH HH HH HH LL LL CC CC RR RR VV VV VV VV RR RR VV VV VV VV FF FF FF FF
  // FD FC FB FA 14 00 07 00 10 00 00 FF 00 00 00 01 00 0F 00 00 04 03 02 01

  uint16_t move_threshold_gate = CMD_GATE_MOVE_THRESH[gate];
  uint16_t still_threshold_gate = CMD_GATE_STILL_THRESH[gate];
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_WRITE_ABD_PARAM;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &move_threshold_gate, sizeof(move_threshold_gate));
  cmd_frame.data_length += sizeof(move_threshold_gate);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &this->new_config.move_thresh[gate],
         sizeof(this->new_config.move_thresh[gate]));
  cmd_frame.data_length += sizeof(this->new_config.move_thresh[gate]);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &still_threshold_gate, sizeof(still_threshold_gate));
  cmd_frame.data_length += sizeof(still_threshold_gate);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &this->new_config.still_thresh[gate],
         sizeof(this->new_config.still_thresh[gate]));
  cmd_frame.data_length += sizeof(this->new_config.still_thresh[gate]);
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG, "Sending set gate %4X sensitivity command: %2X", gate, cmd_frame.command);
  this->send_cmd_from_array(cmd_frame);
}

uint16_t LD2402Component::coeff_to_param_(float coefficient) {
  if (coefficient < 1.0f) {
    coefficient = 1.0f;
  } else if (coefficient > 20.0f) {
    coefficient = 20.0f;
  }
  uint16_t value = static_cast<uint16_t>(coefficient * 10.0f);
  if (value < 0x000A) {
    value = 0x000A;
  } else if (value > 0x00C8) {
    value = 0x00C8;
  }
  return value;
}

int LD2402Component::parse_calibration_progress_(const uint8_t *buffer, int len) {
  if (len < 12) {
    return -1;
  }
  uint16_t cmd_word;
  memcpy(&cmd_word, &buffer[CMD_FRAME_COMMAND], sizeof(cmd_word));
  if ((cmd_word & 0xFF) != 0x0A) {
    return -1;
  }
  uint16_t error_word;
  memcpy(&error_word, &buffer[CMD_ERROR_WORD], sizeof(error_word));
  if (error_word != 0) {
    return -1;
  }

  uint16_t pct;
  memcpy(&pct, &buffer[10], sizeof(pct));
  if (pct <= 100) {
    return pct;
  }

  const uint8_t raw = buffer[10];
  if (raw <= 0x64) {
    return (raw * 100) / 0x64;
  }
  return -1;
}

int LD2402Component::start_auto_threshold_() {
  const uint16_t trigger = this->coeff_to_param_(this->auto_trigger_coefficient_);
  const uint16_t hold = this->coeff_to_param_(this->auto_hold_coefficient_);
  const uint16_t micro = this->coeff_to_param_(this->auto_micro_coefficient_);

  CmdFrameT cmd_frame;
  cmd_frame.data_length = 6;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_AUTO_THRESHOLD_START;
  memcpy(&cmd_frame.data[0], &trigger, sizeof(trigger));
  memcpy(&cmd_frame.data[2], &hold, sizeof(hold));
  memcpy(&cmd_frame.data[4], &micro, sizeof(micro));
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGI(TAG, "Sending auto-threshold start (params 0x%04X 0x%04X 0x%04X)", trigger, hold, micro);
  return this->send_cmd_from_array(cmd_frame);
}

int LD2402Component::query_auto_threshold_progress_() {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_AUTO_THRESHOLD_PROGRESS;
  cmd_frame.footer = CMD_FRAME_FOOTER;
  if (this->send_cmd_from_array(cmd_frame) != 0) {
    return -1;
  }
  return static_cast<int>(this->cmd_reply_.ack_value);
}

int LD2402Component::save_params_to_flash_() {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_SAVE_PARAMS;
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGI(TAG, "Sending parameter save command");
  return this->send_cmd_from_array(cmd_frame);
}

int LD2402Component::reload_gate_thresholds_() {
  for (uint8_t gate = 0; gate < TOTAL_GATES; gate++) {
    delay_microseconds_safe(125);
    if (this->get_gate_threshold_(gate) != 0) {
      return LD2402_ERROR_TIMEOUT;
    }
  }
  return 0;
}

void LD2402Component::publish_calibration_progress_(uint8_t progress) {
  this->calibration_progress_ = progress;
#ifdef USE_SENSOR
  if (this->calibration_progress_sensor_ != nullptr) {
    this->calibration_progress_sensor_->publish_state(progress);
  }
#endif
}

void LD2402Component::poll_auto_calibration_() {
  if (!this->calibrating_) {
    return;
  }

  const uint32_t now = millis();
  if (this->last_calibration_poll_ms_ != 0 && (now - this->last_calibration_poll_ms_) < CALIBRATION_POLL_INTERVAL_MS) {
    return;
  }
  this->last_calibration_poll_ms_ = now;

  const int progress = this->query_auto_threshold_progress_();
  if (progress < 0) {
    ESP_LOGW(TAG, "Auto-threshold progress query failed");
    return;
  }

  this->publish_calibration_progress_(static_cast<uint8_t>(progress));
  if (progress >= 100) {
    this->finish_auto_calibration_();
  }
}

void LD2402Component::finish_auto_calibration_() {
  ESP_LOGI(TAG, "Auto-threshold generation complete");
  if (this->reload_gate_thresholds_() == 0) {
    memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
#ifdef USE_NUMBER
    this->init_gate_config_numbers();
#endif
  } else {
    ESP_LOGW(TAG, "Failed to reload auto-generated gate thresholds");
  }
  if (this->save_params_to_flash_() != 0) {
    ESP_LOGW(TAG, "Failed to persist auto-generated thresholds to flash");
  }
  this->set_system_mode(this->system_mode_);
  this->set_config_mode(false);
  this->calibrating_ = false;
  this->calibration_interference_ = false;
  this->publish_calibration_progress_(100);
}

void LD2402Component::handle_calibration_interference_report_(uint8_t *buffer, int len) {
  if (len < 12) {
    return;
  }
  this->calibration_interference_ = true;
  ESP_LOGW(TAG, "Auto-threshold interference detected — remain still until calibration finishes");

  const uint16_t status = 0x0001;
  const uint16_t gate_bitmap = 0x0000;
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 4;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_AUTO_THRESHOLD_INTERFERENCE | 0x0100;
  memcpy(&cmd_frame.data[0], &status, sizeof(status));
  memcpy(&cmd_frame.data[2], &gate_bitmap, sizeof(gate_bitmap));
  cmd_frame.footer = CMD_FRAME_FOOTER;
  this->write_cmd_frame_(cmd_frame);
}

#ifdef USE_NUMBER
void LD2402Component::init_gate_config_numbers() {
  if (this->gate_timeout_number_ != nullptr) {
    this->gate_timeout_number_->publish_state(static_cast<uint16_t>(this->current_config.timeout));
  }
  if (this->gate_select_number_ != nullptr) {
    this->gate_select_number_->publish_state(0);
  }
  if (this->max_gate_distance_number_ != nullptr) {
    this->max_gate_distance_number_->publish_state(static_cast<uint16_t>(this->current_config.max_gate));
  }
  if (this->gate_move_sensitivity_factor_number_ != nullptr) {
    this->gate_move_sensitivity_factor_number_->publish_state(this->gate_move_sensitivity_factor);
  }
  if (this->gate_still_sensitivity_factor_number_ != nullptr) {
    this->gate_still_sensitivity_factor_number_->publish_state(this->gate_still_sensitivity_factor);
  }
  for (uint8_t gate = 0; gate < TOTAL_GATES; gate++) {
    if (this->gate_still_threshold_numbers_[gate] != nullptr) {
      this->gate_still_threshold_numbers_[gate]->publish_state(
          threshold_serial_to_host(this->current_config.still_thresh[gate]));
    }
    if (this->gate_move_threshold_numbers_[gate] != nullptr) {
      this->gate_move_threshold_numbers_[gate]->publish_state(
          threshold_serial_to_host(this->current_config.move_thresh[gate]));
    }
  }
}

void LD2402Component::refresh_gate_config_numbers() {
  this->gate_timeout_number_->publish_state(this->new_config.timeout);
  this->max_gate_distance_number_->publish_state(this->new_config.max_gate);
}

#endif

}  // namespace esphome::ld2402
