#include "Adafruit_VL53L1X.h"

// timing condition on occupancy events to replace percent difference condition
// AND distance threshold for accumulation

// Const Parameters
const int GPIO_PIN_1 = 25;                      // beware communication, etc. pins on the ESP32
const int XSHUT_PIN_1 = 26;
const int GPIO_PIN_2 = 32;
const int XSHUT_PIN_2 = 33;
const int deriv_arr_length = 1;                 // can be used to do moving average on derivatives
const int tof_1_timing_budget = 100;            // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500 (ms)
const int tof_2_timing_budget = 100;            // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500 (ms)
const int deriv_accumulation_threshold = 15;
const double acc_threshold_pos = 20;            // how high deriv must be to begin accumulation
const double acc_threshold_neg = 20;            // how high deriv must be to begin accumulation
const double deriv_debounce_threshold = 15;     // if deriv is below this, debounce is cancelled
const int debounce_time_millis = 5000;          // when occupancy changes, how long to block continued accumulation in the same direction
const double pair_timing_threshold_millis = 300;      // how close occupancy events must be for the most recent one to get undone (considered a double-count)
const int pos_dist_threshold = 1800;                // distance beyond which distance measurements are ignored
const int neg_dist_threshold = 1800;                // distance beyond which distance measurements are ignored

// Variables
Adafruit_VL53L1X tof_1 = Adafruit_VL53L1X(XSHUT_PIN_1, GPIO_PIN_1);
Adafruit_VL53L1X tof_2 = Adafruit_VL53L1X(XSHUT_PIN_2, GPIO_PIN_2);
int16_t temp;
int16_t dist_1;
int16_t dist_2;
float dist_diff_percent;
int16_t old_dist_1;
int16_t old_dist_2;
int16_t deriv_1;
int16_t deriv_2;
int16_t old_deriv_1;
int16_t old_deriv_2;
double second_deriv_1;
double second_deriv_2;
double old_deriv_arr_avg_1;
double old_deriv_arr_avg_2;
double deriv_arr_1[deriv_arr_length];
double deriv_arr_2[deriv_arr_length];
double deriv_arr_avg_1;
double deriv_arr_avg_2;
int accumulator_pos_1;
int accumulator_neg_1;
int accumulator_pos_2;
int accumulator_neg_2;
int occupancy;
int last_occupant_millis_1;
int last_occupant_millis_2;
int last_occupant_millis_1_actual;
int last_occupant_millis_2_actual;
int previous_last_occupant_millis_1_actual;
int previous_last_occupant_millis_2_actual;
int last_occupancy_sign_1;
int last_occupancy_sign_2;
int num_cycles = 0;
int last_micros = 0;
int this_micros = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  if (! tof_1.begin(0x31, &Wire)) {
    Serial.print("ToF 1 begin error");
    Serial.println(tof_1.vl_status);
    while (1)       delay(10);
  }
  
  if (! tof_2.begin(0x35, &Wire)) {
    Serial.print("ToF 2 begin error");
    Serial.println(tof_2.vl_status);
    while (1)       delay(10);
  }

  if (!tof_1.startRanging()) {
    Serial.print("ToF 1 startranging error");
    Serial.println(tof_1.vl_status);
    while (1)       delay(10);
  }
  
  if (!tof_2.startRanging()) {
    Serial.print("ToF 2 startranging error");
    Serial.println(tof_2.vl_status);
    while (1)       delay(10);
  }

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500 (ms)
  tof_1.setTimingBudget(tof_1_timing_budget);
  tof_2.setTimingBudget(tof_2_timing_budget);
}

void loop() {
  // Retrieve distance measurements from sensors
  if (tof_1.dataReady()) {
    old_dist_1 = dist_1;
    temp = tof_1.distance();
    if (temp != -1) {
      dist_1 = temp;
    }
    tof_1.clearInterrupt();
  }
  if (tof_2.dataReady()) {
    old_dist_2 = dist_2;
    temp = tof_2.distance();
    if (temp != -1) {
      dist_2 = temp;
    }
    tof_2.clearInterrupt();
  }

  // Calculate first derivatives
  old_deriv_1 = deriv_1;
  old_deriv_2 = deriv_2;
  deriv_1 = dist_1 - old_dist_1;
  deriv_2 = dist_2 - old_dist_2;
  dist_diff_percent = 100.0*abs(dist_1 - dist_2)/max(dist_1, dist_2);

  // Moving average filter applied to first derivatives
  deriv_arr_1[num_cycles % deriv_arr_length] = deriv_1;
  deriv_arr_2[num_cycles % deriv_arr_length] = deriv_2;
  old_deriv_arr_avg_1 = deriv_arr_avg_1;
  old_deriv_arr_avg_2 = deriv_arr_avg_2;
  deriv_arr_avg_1 = 0;
  deriv_arr_avg_2 = 0;
  for (int i=0; i<deriv_arr_length; ++i) {
    deriv_arr_avg_1 += deriv_arr_1[i] / (deriv_arr_length*1.0);
    deriv_arr_avg_2 += deriv_arr_2[i] / (deriv_arr_length*1.0);
  }

  // Accumulator logic for ToF 1
  if ((millis() - last_occupant_millis_1) > debounce_time_millis) {
    if (deriv_arr_avg_1 > deriv_accumulation_threshold && dist_1 < pos_dist_threshold) {
      ++accumulator_pos_1;
    }
    else {
      accumulator_pos_1 = 0;
    }
    if (deriv_arr_avg_1 < -deriv_accumulation_threshold && dist_1 < neg_dist_threshold) {
      ++accumulator_neg_1;
    }
    else {
      accumulator_neg_1 = 0;
    }
  }

  // Accumulator logic for ToF 2
  if ((millis() - last_occupant_millis_2) > debounce_time_millis) {
    if (deriv_arr_avg_2 > deriv_accumulation_threshold && dist_2 < pos_dist_threshold) {
      ++accumulator_pos_2;
    }
    else {
      accumulator_pos_2 = 0;
    }
    if (deriv_arr_avg_2 < -deriv_accumulation_threshold && dist_2 < neg_dist_threshold) {
      ++accumulator_neg_2;
    }
    else {
      accumulator_neg_2 = 0;
    }
  }

  previous_last_occupant_millis_1_actual = last_occupant_millis_1_actual;
  previous_last_occupant_millis_2_actual = last_occupant_millis_2_actual;

  // Occupancy event logic for ToF 1
  if (accumulator_pos_1 > acc_threshold_pos) {
    ++occupancy;
    last_occupancy_sign_1 = 1;
    accumulator_pos_1 = 0;
    last_occupant_millis_1 = millis();
    last_occupant_millis_1_actual = millis();
  }
  if (accumulator_neg_1 > acc_threshold_neg) {
    --occupancy;
    last_occupancy_sign_1 = -1;
    accumulator_neg_1 = 0;
    last_occupant_millis_1 = millis();
    last_occupant_millis_1_actual = millis();
  }
  if (abs(deriv_arr_avg_1) < deriv_debounce_threshold) {
    last_occupant_millis_1 = 0;
  }

  // Occupancy event logic for ToF 2
  if (accumulator_pos_2 > acc_threshold_pos) {
    ++occupancy;
    last_occupancy_sign_2 = 1;
    accumulator_pos_2 = 0;
    last_occupant_millis_2 = millis();
    last_occupant_millis_2_actual = millis();
  }
  if (accumulator_neg_2 > acc_threshold_neg) {
    --occupancy;
    last_occupancy_sign_2 = -1;
    accumulator_neg_2 = 0;
    last_occupant_millis_2 = millis();
    
    last_occupant_millis_2_actual = millis();
  }
  if (abs(deriv_arr_avg_2) < deriv_debounce_threshold) {
    last_occupant_millis_2 = 0;
  }

  // Check whether both sensors triggered occupancy in the same direction in rapid succession.
  // Also check whether occupancy event has triggered this loop.
  // If so, undo the action of the most recent
  if ((abs(last_occupant_millis_1_actual - last_occupant_millis_2_actual) < pair_timing_threshold_millis) && 
  ((previous_last_occupant_millis_1_actual != last_occupant_millis_1_actual) ||
  (previous_last_occupant_millis_2_actual != last_occupant_millis_2_actual)) && 
  (last_occupancy_sign_1 == last_occupancy_sign_2)) {
    if (last_occupant_millis_1_actual > last_occupant_millis_2_actual) {  // tof 1 triggered last
      occupancy -= last_occupancy_sign_1;
      Serial.println("Undoing ToF 1");
    }
    if (last_occupant_millis_2_actual > last_occupant_millis_1_actual) {  // tof 2 triggered last
      occupancy -= last_occupancy_sign_2;
      Serial.println("Undoing ToF 2");
    }
  }


  // Displaying debug information
  //Serial.print(1000);  // +/- constant so that serial plotter window doesn't shrink too much
  //Serial.print('\t');
  //Serial.print(-1000);
  //Serial.print('\t');
  Serial.print("ToF 1: ");
  Serial.print(dist_1);
  //Serial.print('\t');
  //Serial.print(deriv_arr_avg_1);
  Serial.print('\t');
  Serial.print(accumulator_pos_1);
  Serial.print('\t');
  Serial.print(accumulator_neg_1);
  Serial.print('\t');
  Serial.print("ToF 2: ");
  Serial.print(dist_2);
  Serial.print('\t');
  //Serial.print(deriv_arr_avg_2);
  //Serial.print('\t');
  Serial.print(accumulator_pos_2);
  Serial.print('\t');
  Serial.print(accumulator_neg_2);
  Serial.print('\t');
  Serial.print("Occupancy: ");
  Serial.print(occupancy);
  Serial.print('\t');
  Serial.print("last_occupant_millis_1: ");
  Serial.print(last_occupant_millis_1_actual);
  Serial.print('\t');
  Serial.print("last_occupant_millis_2: ");
  Serial.print(last_occupant_millis_2_actual);
  Serial.print('\t');
  Serial.print("last_occupany_sign_1: ");
  Serial.print(last_occupancy_sign_1);
  Serial.print('\t');
  Serial.print("last_occupany_sign_2: ");
  Serial.print(last_occupancy_sign_2);
  Serial.println(" ");

  ++num_cycles;
  delay(10);
}
