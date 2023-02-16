#include "Adafruit_VL53L1X.h"

#define GPIO_PIN_1 25
#define XSHUT_PIN_1 26

#define GPIO_PIN_2 32
#define XSHUT_PIN_2 33

const int deriv_arr_length = 25;    // longer array rejects noise but creates lag
const int tof_1_timing_budget = 20; // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500 (ms)
const int tof_2_timing_budget = 20;
const double deriv_avg_threshold = 20;

Adafruit_VL53L1X tof_1 = Adafruit_VL53L1X(XSHUT_PIN_1, GPIO_PIN_1);
Adafruit_VL53L1X tof_2 = Adafruit_VL53L1X(XSHUT_PIN_2, GPIO_PIN_2);
int16_t temp;
int16_t distance_1;
int16_t distance_2;
int16_t old_distance_1;
int16_t old_distance_2;
double deriv_arr_1[deriv_arr_length];
double deriv_arr_2[deriv_arr_length];
double deriv_arr_1_avg;
double deriv_arr_2_avg;
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
  if (tof_1.dataReady()) {
    old_distance_1 = distance_1;
    temp = tof_1.distance();
    if (temp != -1) {
      distance_1 = temp;
    }
    tof_1.clearInterrupt();
  }

  if (tof_2.dataReady()) {
    old_distance_2 = distance_2;
    temp = tof_2.distance();
    if (temp != -1) {
      distance_2 = temp;
    }
    tof_2.clearInterrupt();
  }

  deriv_arr_1[num_cycles % deriv_arr_length] = distance_1 - old_distance_1;
  deriv_arr_2[num_cycles % deriv_arr_length] = distance_2 - old_distance_2;
  ++num_cycles;
  deriv_arr_1_avg = 0;
  deriv_arr_2_avg = 0;
  for (int i=0; i<deriv_arr_length; ++i) {
    deriv_arr_1_avg += deriv_arr_1[i] / (deriv_arr_length*1.0);
    //Serial.print(deriv_arr_1[i]);
    //Serial.print(" ");
    deriv_arr_2_avg += deriv_arr_2[i] / (deriv_arr_length*1.0);
    //Serial.print(deriv_arr_2[i]);
    //Serial.print(" ");
  }

/*
  Serial.print(1000);
  Serial.print('\t');
  Serial.print(-1000);
  Serial.print('\t');
  Serial.print(distance_1);
  Serial.print('\t');
  Serial.print(distance_2);
  Serial.print('\t');
  Serial.print(deriv_arr_1_avg);
  Serial.print('\t');
  Serial.println(deriv_arr_2_avg);
  Serial.println(" ");
  */

  if (deriv_arr_1_avg > deriv_avg_threshold) {
    Serial.print(1);
  }
  else if (deriv_arr_1_avg < -deriv_avg_threshold) {
    Serial.print(-1);
  }
  else {
    Serial.print(0);
  }
  Serial.print('\t');

  if (deriv_arr_2_avg > deriv_avg_threshold) {
    Serial.print(1);
  }
  else if (deriv_arr_2_avg < -deriv_avg_threshold) {
    Serial.print(-1);
  }
  else {
    Serial.print(0);
  }
  Serial.println('\t');

  delay(10);
}
