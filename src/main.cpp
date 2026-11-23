#include <Arduino.h>
#include <Wire.h>
#include "ASCII_FONT_5X8.h"
#include <Adafruit_BNO080.h>
#include <Adafruit_Sensor.h>
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

Adafruit_BNO055 IMUSensor = Adafruit_BNO055(55, 0x28, &Wire);

struct PulseAccumulator {
  double accumulated_ms = 0.0;
  int last_dir = 0;

  int update(double control_signal, double active_flow_rate, int sample_ms, int min_pulse_ms) {
    if (fabs(control_signal) <= 1e-8 || active_flow_rate <= 1e-6) {
      accumulated_ms = 0.0;
      last_dir = 0;
      return 0;
    }

    int dir = (control_signal > 0.0) ? 1 : -1;
    if (dir != last_dir) {
      accumulated_ms = 0.0;
      last_dir = dir;
    }

    double demand_ratio = constrain(fabs(control_signal) / active_flow_rate, 0.0, 1.0);
    double raw_ms = demand_ratio * (double)sample_ms;
    double total_ms = raw_ms + accumulated_ms;

    if (total_ms < min_pulse_ms) {
      accumulated_ms = total_ms;
      return 0;
    }

    int pulse_ms = (int)constrain(total_ms, 0.0, (double)sample_ms);
    accumulated_ms = fmax(0.0, total_ms - (double)pulse_ms);
    return dir * pulse_ms;
  }
};

PulseAccumulator buoyancy_acc;
PulseAccumulator pitch_acc;


unsigned long last_ms_sampled = 0;
int sample_rate = 100; // Time between updates in ms

// Control Gains and Tolerances
const double P_gain = 0.00005, D_gain = 0.0015, DD_gain = 0.015;  // Heave
const double Pa_gain = 0.001, Da_gain = 0.01, DDa_gain = 0.015;   // Pitch

double pos_tank1 = 0.25; // Position of tank 1 relative to back
double pos_tank2 = 2.25; // Position of tank 2 relative to back

// Tank
double fill_pressure = 3500; // psi
double bottle_volume_liters = 2.0;
double current_bottle_pressure = fill_pressure;
double remaining_air_liters = bottle_volume_liters * (current_bottle_pressure / 14.7);
double max_pump_rate_full = 0.0002;       // 0.2 L/s = 0.0002 m^3/s
double max_vent_rate_surface = 0.00025;   // 0.25 L/s = 0.00025 m^3/s
double regulated_pressure_psi = 50.0; // Output setting of secondary pressure reducer


double position = 0.0; // From surface, in m
double velocity = 0.0;
double acceleration = 0.0;

double pitch = 0.0; // Pitch angle in radians, +theta -> nose up, -theta -> nose down
double angular_velocity = 0.0;
double angular_acceleration = 0.0;

double pos_tolerance = 0.03;
double vel_tolerance = 0.02;
double pitch_tolerance = 0.03;
double ang_vel_tolerance = 0.02;
double z_trig = 0.05;

double buoyancy_setpoint = 0.0; // From surface
const int min_duty_cycle = 40; // Minimum pulse for actuator (ms)


void matrix_led(int row, int col, bool state);
void scrollStep(const char *str, int scroll_speed);
void update_bottle_pressure(double air_used_at_depth, double depth);
bool is_system_active(unsigned long sequence_start_time, unsigned long timeout_ms);
int buoyancy_control();
int pitch_control();

char myString[] = "This is a test string to scroll across the LED matrix display. It should wrap around and continue scrolling indefinitely";

// TODO: Calculate average pilot's breathing cadence for a low-pass filter to smooth out filling/venting commands
// TODO: Get pilot breath volume
// TODO: Implement buoyancy control and pitch control in loop()
// TODO: Solenoid control code
// TODO: Sensor code (processing)
// TODO: Implement calibration/idle state (active stabilization), race state (less aggressive stabilization), and passive state (no stabilization)
// TODO: Pitch control needs two separate PulseAccumulator instances (one scaled by fill rate, one scaled by vent rate) to keep net displaced volume constant
// TODO: Track tank air volume to anticipate air expansion and compression at depth, and to prevent overfilling/over-venting (venting when there's no air in the tank)
// TODO: Feed-forward control during race state (to counterract hydrodynamic hull lift)
// TODO: Adjust P-D-DD gains or tolerance
// TODO: Implement z-trig to prevent noise while on trolley


void setup() {
  matrix.begin();
  // Configure communication pins, start I2C bus, initialize BNO055 and pressure sensors, setup intial vars
}
  
void loop() {
  // 10hz sampling rate (every 0.1s)
  if (millis() - last_ms_sampled >= 100) {
    last_ms_sampled = millis();

  }
  scrollStep(myString, 50);
}

int getCharWidth(char c) {
  if (c == ' ') return 3; // Standard space width

  uint8_t buf[8];
  getCharBitmap(c, buf);

  // Column 0 = Bit 4 (0b10000), Column 4 = Bit 0 (0b00001)
  for (int col = 4; col >= 0; col--) {
    for (int row = 0; row < 8; row++) {
      if ((buf[row] >> (4 - col)) & 1) {
        return col + 1; // Found true rightmost lit pixel!
      }
    }
  }
  return 3;
}

void scrollStep(const char *str, int scroll_speed) {
  static unsigned long lastUpdate = 0;
  static char prevStr[256] = "";
  static int charIndex = 0;
  static int colIndex = 0; // Negative values represent loop padding steps
  static uint8_t displayBuffer[8][12] = {0};

  if (strcmp(str, prevStr) != 0) {
    strncpy(prevStr, str, sizeof(prevStr) - 1);
    prevStr[sizeof(prevStr) - 1] = '\0';
    charIndex = 0;
    colIndex = 0;
    memset(displayBuffer, 0, sizeof(displayBuffer));
  }

  if (millis() - lastUpdate < scroll_speed) {
    return;
  }
  lastUpdate = millis();

  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 11; col++) {
      displayBuffer[row][col] = displayBuffer[row][col + 1];
    }
  }

  uint8_t newColByte = 0;

  if (colIndex < 0) {
    // Negative colIndex handles the 6 padding columns at end-of-string
    newColByte = 0;
    colIndex++;
    if (colIndex == 0) {
      charIndex = 0; // Restart at first character
    }
  } else {
    uint8_t charBuf[8];
    getCharBitmap(str[charIndex], charBuf);
    int realWidth = getCharWidth(str[charIndex]);

    if (colIndex < realWidth) {
      for (int r = 0; r < 8; r++) {
        if ((charBuf[r] >> (4 - colIndex)) & 1) {
          newColByte |= (1 << (7 - r));
        }
      }
      colIndex++;
    } else {
      // 1-pixel gap between letters
      newColByte = 0;
      charIndex++;

      if (str[charIndex] == '\0') {
        colIndex = -10;
      } else {
        colIndex = 0;
      }
    }
  }

  for (int row = 0; row < 8; row++) {
    displayBuffer[row][11] = (newColByte >> (7 - row)) & 1;
  }

  matrix.renderBitmap(displayBuffer, 8, 12);
}

bool is_system_active(unsigned long sequence_start_time, unsigned long timeout_ms) {
  bool position_out   = (position <= buoyancy_setpoint - pos_tolerance) || (position >= buoyancy_setpoint + pos_tolerance);
  bool velocity_out   = (velocity <= -vel_tolerance) || (velocity >= vel_tolerance);
  bool pitch_out      = (fabs(pitch) >= pitch_tolerance);
  bool ang_vel_out    = (fabs(angular_velocity) >= ang_vel_tolerance);
  bool within_timeout = (millis() - sequence_start_time < timeout_ms);

  return (position_out || velocity_out || pitch_out || ang_vel_out) && within_timeout;
}

int buoyancy_control() {
  double control_signal = P_gain * (buoyancy_setpoint - position)
                        - D_gain * velocity
                        - DD_gain * acceleration;

  double ambient_ata = 1.0 + fmax(0.0, position) / 10.0;
  double active_flow_rate = 0.0;

  if (control_signal < 0.0) {
    // Negative signal = shallower setpoint = Fill
    double net_pressure_psi = regulated_pressure_psi - (position * 1.45);
    if (net_pressure_psi <= 2.0) {
      buoyancy_acc.update(0.0, 0.0, sample_rate, min_duty_cycle); // Reset state
      return 0;
    }
    active_flow_rate = (max_pump_rate_full / ambient_ata) * sqrt(net_pressure_psi / regulated_pressure_psi);
  } else {
    // Positive signal = deeper setpoint = Vent
    active_flow_rate = max_vent_rate_surface / sqrt(ambient_ata);
  }

  // Negate so + -> Fill and - -> Vent
  return -buoyancy_acc.update(control_signal, active_flow_rate, sample_rate, min_duty_cycle);
}

int pitch_control() {
  double control_signal = -Pa_gain * pitch
                        - Da_gain * angular_velocity
                        - DDa_gain * angular_acceleration;

  double ambient_ata = 1.0 + fmax(0.0, position) / 10.0;
  double net_pressure_psi = regulated_pressure_psi - (position * 1.45);
  if (net_pressure_psi <= 2.0) {
    pitch_acc.update(0.0, 0.0, sample_rate, min_duty_cycle); // Reset state
    return 0;
  }

  double active_flow_rate = (max_pump_rate_full / ambient_ata) * sqrt(net_pressure_psi / regulated_pressure_psi);

  // + = Nose UP (Fill Fore / Vent Aft), - = Nose DOWN (Vent Fore / Fill Aft)
  return pitch_acc.update(control_signal, active_flow_rate, sample_rate, min_duty_cycle);
}