#include "Adafruit_VL53L1X.h"

// Const Parameters
const int GPIO_PIN_1 = 25;                      // beware communication, etc. pins on the ESP32
const int XSHUT_PIN_1 = 26;
const int GPIO_PIN_2 = 32;
const int XSHUT_PIN_2 = 33;
const int deriv_arr_length = 10;                // longer array rejects noise but creates lag
const int tof_1_timing_budget = 100;             // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500 (ms)
const int tof_2_timing_budget = 100;             // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500 (ms)
const double milestone_threshold = 20;          // used for milestone 1 test
const double deriv_accumulation_threshold = 30; // how high deriv must be to begin accumulation
const double deriv_debounce_threshold = 10;     // if deriv is below this, debounce is cancelled
const int accumulator_threshold = 60;           // how high accumulators must climb to trigger occupancy change
const int debounce_time_millis = 5000;          // when occupancy changes, how long to block continued accumulation in the same direction

// Variables
Adafruit_VL53L1X tof_1 = Adafruit_VL53L1X(XSHUT_PIN_1, GPIO_PIN_1);
Adafruit_VL53L1X tof_2 = Adafruit_VL53L1X(XSHUT_PIN_2, GPIO_PIN_2);
int16_t temp;
int16_t dist_1;
int16_t dist_2;
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
int occupancy;
int last_occupant_millis_1;
int num_cycles = 0;

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

  // Calculate first and second derivatives
  old_deriv_1 = deriv_1;
  old_deriv_2 = deriv_2;
  deriv_1 = dist_1 - old_dist_1;
  deriv_2 = dist_2 - old_dist_2;
  // second_deriv_1 = deriv_arr_avg_1 - old_deriv_arr_avg_1;
  // second_deriv_2 = deriv_arr_avg_1 - old_deriv_arr_avg_2;

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

  // Accumulator logic
  if ((millis() - last_occupant_millis_1) > debounce_time_millis) {
    if (deriv_arr_avg_1 > deriv_accumulation_threshold) {
      ++accumulator_pos_1;
    }
    else {
      accumulator_pos_1 = 0;
    }
    if (deriv_arr_avg_1 < -deriv_accumulation_threshold) {
      ++accumulator_neg_1;
    }
    else {
      accumulator_neg_1 = 0;
    }
  }

  // Occupancy event logic
  if (accumulator_pos_1 > accumulator_threshold) {
    ++occupancy;
    accumulator_pos_1 = 0;
    last_occupant_millis_1 = millis();
  }
  if (accumulator_neg_1 > accumulator_threshold) {
    --occupancy;
    accumulator_neg_1 = 0;
    last_occupant_millis_1 = millis();
  }

  // Debounce cancellation
  if (abs(deriv_arr_avg_1) < deriv_debounce_threshold) {
    last_occupant_millis_1 = 0;
  }

/*
  // Displaying debug information
  Serial.print(250);
  Serial.print('\t');
  Serial.print(-250);
  Serial.print('\t');
  //Serial.print(dist_1);
  //Serial.print('\t');
  Serial.print(deriv_arr_avg_1);
  Serial.print('\t');
  Serial.print(accumulator_pos_1);
  Serial.print('\t');
  Serial.print(accumulator_neg_1);
  Serial.print('\t');
  Serial.print(occupancy);
  Serial.println(" ");
  */

  // Terminal output for Milestone 1 Tests
  /*
  Serial.print("Sensor 1 Reading: ");
  Serial.print(dist_1);
  Serial.print(" cm");
  Serial.print("\t");
  Serial.print("Sensor 2 Reading: ");
  Serial.print(dist_2);
  Serial.println(" cm");
  */
  if (deriv_arr_avg_1 > milestone_threshold) {
    Serial.print("Sensor 1: Moving away");
  }
  else if (deriv_arr_avg_1 < -milestone_threshold) {
    Serial.print("Sensor 1: Moving towards");
  }
  else {
    Serial.print("Sensor 1: No motion");
  }
  Serial.print('\t');

  if (deriv_arr_avg_2 > milestone_threshold) {
    Serial.print("Sensor 2: Moving away");
  }
  else if (deriv_arr_avg_2 < -milestone_threshold) {
    Serial.print("Sensor 2: Moving towards");
  }
  else {
    Serial.print("Sensor 2: No motion");
  }
  Serial.println('\t');

  ++num_cycles;
  delay(10);
}
