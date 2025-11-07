// we have ESP32-POE-WROVER
#include <cstdint>
#define BOARD_HAS_PSRAM

#include "HardwareSerial.h"
#include <ETH.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HAMqttDevice.h>
#include <optional>
#include <string>
#include <sstream>
#include <iomanip>
#include "driver/adc.h"


#define USE_WIFI false

#if USE_WIFI
std::string wifi_ssid = ;
std::string wifi_password = ;
#endif

std::string mqtt_server = "192.168.0.254";
int mqtt_port = 1883;
std::string mqtt_user = "schakelaars_living";
std::string mqtt_password = "schakelaars_living:D";
std::string mqtt_client_id = "schakelaars_living";
std::string topic_availability = mqtt_client_id + "/availability";

std::string MQTT_HA_DISCOVERY_PREFIX = "homeassistant";

// Matrix scanning configuration
// 2 rows (driver pins) x 3 columns (input pins) = 6 switches
constexpr std::array<uint8_t, 2> MATRIX_DRIVER_PINS = {4, 14};  // Drive r pins - outputs
constexpr std::array<uint8_t, 4> MATRIX_INPUT_PINS = {13, 32, 15, 3};  // Input pins - inputs with pullup
constexpr uint8_t ANALOG_PIN = 35;  // Analog input pin
constexpr uint8_t ANALOG_BUTTON_PIN = 5; // pullup
constexpr adc1_channel_t ANALOG_CHANNEL = adc1_channel_t::ADC1_CHANNEL_0;

#if USE_WIFI
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
#else
WiFiClient ethClient;
PubSubClient mqtt(ethClient);
#endif


template<typename T>
struct Input {
	Input(std::string _topic, std::string name, uint8_t pin, HAMqttDevice::DeviceType type) :
		device(name.c_str(), type, MQTT_HA_DISCOVERY_PREFIX.c_str()), pin(pin) {
		topic = mqtt_client_id + "/" + _topic;

		device.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32\",\"manufacturer\":\"Custom\"}");
		device.addConfigVar("payload_avail", "online");
		device.addConfigVar("payload_not_avail", "offline");
		device.addConfigVar("avty_t", topic_availability.c_str());
		device.addConfigVar("stat_t", topic.c_str());
	}

	HAMqttDevice device;
	uint8_t pin;
	std::string topic;
	std::optional<T> state, lastState;
	static constexpr unsigned long UPDATE_MIN_INTERVAL = 50;
	unsigned long lastUpdateTime = 0;

	void publishDiscovery() {
		mqtt.publish(device.getConfigTopic().c_str(), device.getConfigPayload().c_str(), true);
	}

protected:
	virtual std::optional<T> read() = 0;
	virtual void changed() { };
	virtual std::string getPayload() = 0;

	virtual bool shouldUpdate() {
		if (!state.has_value())
			return false;
		if (lastState.has_value())
			if (state.value() == lastState.value())
				return false;
		return millis() - lastUpdateTime >= UPDATE_MIN_INTERVAL;
	}

public:
	void update() {
		state = read();
		if (!shouldUpdate())
			return;
		lastState = state;
		lastUpdateTime = millis();

		auto payload = getPayload();
		Serial.print("Topic ");
		Serial.print(topic.c_str());
		Serial.print("\tPayload ");
		Serial.println(payload.c_str());
		mqtt.publish(topic.c_str(), payload.c_str(), true);
	}
};

struct DigitalInput : public Input<bool> {
	DigitalInput(std::string topic, std::string name, uint8_t pin)
		: Input<bool>(topic, name, pin, HAMqttDevice::BINARY_SENSOR) {
		this->device.addConfigVar("payload_on", "1");
		this->device.addConfigVar("payload_off", "0");
		pinMode(pin, INPUT_PULLUP);
	}

	std::string getPayload() override {
		return state.value() ? "1" : "0";
	}

	std::optional<bool> read() override {
		return !digitalRead(pin);
	}
};


struct AnalogInput : Input<float> {
	AnalogInput(std::string topic, std::string name, uint8_t pin)
		: Input(topic, name, pin, HAMqttDevice::SENSOR) {
		device.addConfigVar("unit_of_measurement", "%");
		device.addConfigVar("state_class", "measurement");
		device.addConfigVar("icon", "mdi:brightness-percent");
		pinMode(pin, INPUT);
	}

	std::string getPayload() override {
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(1) << state.value();
		return oss.str();
	}

	static constexpr int ADC_MAX = 4095; // 12-bit ADC on ESP32
	static constexpr float HYSTERESIS_PERCENT = .75f;

	std::array<uint32_t, 4> samples{ 0 };
	static constexpr size_t SAMPLES_PER_READ = 16;
	size_t sampleIndex = 0;
	uint64_t sampleSum = 0;

	std::optional<float> read() override {
		sampleSum -= samples[sampleIndex];
		samples[sampleIndex] = 0;
		for (int i = 0; i < SAMPLES_PER_READ; ++i)
			samples[sampleIndex] += analogRead(pin);
		sampleSum += samples[sampleIndex];
		if (++sampleIndex >= samples.size())
			sampleIndex = 0;
		else if (!state.has_value())
			return std::nullopt; // wait until buffer is full
		auto result = (sampleSum / float(samples.size() * SAMPLES_PER_READ * ADC_MAX)) * 100.0f;
		if (result < HYSTERESIS_PERCENT)
			return 0.0f;
		else if (result > 100.0f - HYSTERESIS_PERCENT)
			return 100.0f;
		return result;
	}

	bool shouldUpdate() override {
		if (!Input<float>::shouldUpdate())
			return false;
		if (!lastState.has_value())
			return true;
		if (state.value() == 0 || state.value() == 100)
			return true;
		return std::abs(state.value() - lastState.value()) >= HYSTERESIS_PERCENT;
	}
};

DigitalInput switch0Input("switch0", "Living Switch 0", MATRIX_INPUT_PINS[0]);
DigitalInput switch1Input("switch1", "Living Switch 1", MATRIX_INPUT_PINS[1]);
DigitalInput switch2Input("switch2", "Living Switch 2", MATRIX_INPUT_PINS[2]);
DigitalInput switch3Input("switch3", "Living Switch 3", MATRIX_INPUT_PINS[3]);
DigitalInput switch4Input("switch4", "Living Switch 4", MATRIX_INPUT_PINS[0]);
DigitalInput switch5Input("switch5", "Living Switch 5", MATRIX_INPUT_PINS[1]);
DigitalInput switch6Input("switch6", "Living Switch 6", MATRIX_INPUT_PINS[2]);
DigitalInput switch7Input("switch7", "Living Switch 7", MATRIX_INPUT_PINS[3]);

// Analog input
AnalogInput dimmerInput("dimmer", "Living Dimmer", ANALOG_PIN);
DigitalInput dimmerSwitchInput("dimmer_switch", "Living Dimmer Switch", ANALOG_BUTTON_PIN);

// Create status sensor to show device availability in HA
HAMqttDevice statusSensor("Status", HAMqttDevice::BINARY_SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str());

#if !USE_WIFI
static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event) {
	switch (event) {
		case ARDUINO_EVENT_ETH_START:
		Serial.println("ETH Started");
		ETH.setHostname(mqtt_client_id.c_str());
		break;
		case ARDUINO_EVENT_ETH_CONNECTED:
		Serial.println("ETH Connected");
		break;
		case ARDUINO_EVENT_ETH_GOT_IP:
		Serial.print("ETH IP: ");
		Serial.println(ETH.localIP());
		Serial.print("ETH MAC: ");
		Serial.println(ETH.macAddress());
		eth_connected = true;
		break;
		case ARDUINO_EVENT_ETH_DISCONNECTED:
		Serial.println("ETH Disconnected");
		eth_connected = false;
		break;
		case ARDUINO_EVENT_ETH_STOP:
		Serial.println("ETH Stopped");
		eth_connected = false;
		break;
		default:
		break;
	}
}
#endif

void mqtt_print_error() {
	int rc = mqtt.state();
	Serial.print("failed, rc=");
	Serial.print(rc);
	Serial.print(" (");
	switch(rc) {
		case -4: Serial.print("TIMEOUT");         break;
		case -3: Serial.print("CONNECTION_LOST"); break;
		case -2: Serial.print("CONNECT_FAILED");  break;
		case -1: Serial.print("DISCONNECTED");    break;
		case 1:  Serial.print("BAD_PROTOCOL");    break;
		case 2:  Serial.print("BAD_CLIENT_ID");   break;
		case 3:  Serial.print("UNAVAILABLE");     break;
		case 4:  Serial.print("BAD_CREDENTIALS"); break;
		case 5:  Serial.print("UNAUTHORIZED");    break;
		default: Serial.print("UNKNOWN");         break;
	}
	Serial.println(")");
	delay(100);
}

void setup() {
	delay(500);  // Give hardware time to stabilize

	Serial.begin(921600);
	Serial.println("Hello! ESP32-POE ADC to MQTT");

#if USE_WIFI
	Serial.print("Connecting to WiFi: ");
	Serial.println(wifi_ssid.c_str());
	WiFi.mode(WIFI_STA);
	WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("\nWiFi connected!");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
#else
	WiFi.mode(WIFI_OFF);
	Serial.println("Initializing Ethernet...");
	WiFi.onEvent(WiFiEvent);

	ETH.begin();
#endif

	for (auto pin : MATRIX_DRIVER_PINS) {
		pinMode(pin, INPUT); // float
	}
	adc1_config_channel_atten(ANALOG_CHANNEL, ADC_ATTEN_DB_12);
	adc_set_clk_div(255);

	mqtt.setServer(mqtt_server.c_str(), mqtt_port);
	mqtt.setBufferSize(2048);
	mqtt.setCallback([](char* topic, byte* payload, unsigned int length) {});

	// Setup main status sensor (ESP32 connectivity)
	statusSensor.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32\",\"manufacturer\":\"Custom\"}");
	statusSensor.addConfigVar("payload_on", "online");
	statusSensor.addConfigVar("payload_off", "offline");
	statusSensor.addConfigVar("stat_t", topic_availability.c_str());
	statusSensor.addConfigVar("device_class", "connectivity");
	statusSensor.addConfigVar("entity_category", "diagnostic");
}

void loop() {
#if USE_WIFI
	// TODO
#else
	// Check if Ethernet connection was lost
	if (!eth_connected) {
		Serial.print("Ethernet connection lost, reconnecting");
		while (!eth_connected) {
			delay(100);
			Serial.print(".");
		}
		Serial.println("\nEthernet reconnected!");
		delay(2000);  // Give Ethernet time to fully initialize before using WiFiClient
	}
#endif

	if (!mqtt.connected()) {
		Serial.print("Attempting MQTT connection... ");
		// Connect with Last Will and Testament (LWT) for availability
		if (!mqtt.connect(mqtt_client_id.c_str(), mqtt_user.c_str(), mqtt_password.c_str(),
		                 topic_availability.c_str(), 0, true, "offline")) {
			mqtt_print_error();
			return;
		}

		Serial.println("connected");

		// Publish mqtt autodiscovery & availability as online
		switch0Input.publishDiscovery();
		switch1Input.publishDiscovery();
		switch2Input.publishDiscovery();
		switch3Input.publishDiscovery();
		switch4Input.publishDiscovery();
		switch5Input.publishDiscovery();
		switch6Input.publishDiscovery();
		switch7Input.publishDiscovery();
		dimmerInput.publishDiscovery();
		dimmerSwitchInput.publishDiscovery();

		mqtt.publish(statusSensor.getConfigTopic().c_str(), statusSensor.getConfigPayload().c_str(), true);
		mqtt.publish(topic_availability.c_str(), "online", true);
		Serial.println("Published MQTT discovery configs");
	}
  	if (!mqtt.loop()) {
		mqtt_print_error();
		return;
	}


	constexpr unsigned long SETTLING_TIME_MS = 10;

	pinMode(MATRIX_DRIVER_PINS[0], OUTPUT);
	digitalWrite(MATRIX_DRIVER_PINS[0], LOW);
	delay(SETTLING_TIME_MS);
	switch0Input.update();
	switch1Input.update();
	switch2Input.update();
	switch3Input.update();
	pinMode(MATRIX_DRIVER_PINS[0], INPUT);

	pinMode(MATRIX_DRIVER_PINS[1], OUTPUT);
	digitalWrite(MATRIX_DRIVER_PINS[1], LOW);
	delay(SETTLING_TIME_MS);
	switch4Input.update();
	switch5Input.update();
	switch6Input.update();
	switch7Input.update();
	pinMode(MATRIX_DRIVER_PINS[1], INPUT);

	dimmerInput.update();
	dimmerSwitchInput.update();
}