void setupWatchdog(void);
void deepSleep(uint8_t cycles);
float dewpoint(float temperature, float humidity);

struct SensorData {
  float humidity;
  float temperature;
};

void loop();
