#define BLYNK_TEMPLATE_ID "TMPL3H6aR4481"
#define BLYNK_TEMPLATE_NAME "HYDROGUARD"

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>

// WiFi Credentials
const char* ssid = "AYANOKOJI";
const char* password = "KARUIZAWA";

// HiveMQ MQTT Broker
const char* mqtt_server = "797b39718192449bbfe32bbe03652c57.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "SIDDMQTT";
const char* mqtt_pass = "SIDDmqtt@123";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Blynk Auth Token
char auth[] = "TBq35TGzti1SuxhqWHZ3HbG2XeNKlSAv";

// Node ID
const char* nodeID = "node2";

// MQTT Topics
String mqtt_topic_pub = String("esp32/") + nodeID + "/sensor";
String mqtt_topic_sub = String("esp32/") + nodeID + "/command";

// Sensor Pins
#define FLOW_SENSOR_3 32
#define FLOW_SENSOR_4 33
#define PRESSURE_SENSOR 35
#define TAMPER_SENSOR 25  // Tamper Detection GPIO Pin

// Flow Sensor Variables
volatile int flow3_pulse_count = 0;
volatile int flow4_pulse_count = 0;

// Alarm Logic Variables
bool alarmActive = false;
unsigned long lastMismatchTime = 0;
unsigned long alarmStartTime = 0;
const unsigned long ALARM_DURATION = 3600000;  // 1 hour
const unsigned long ALARM_DELAY = 10000;  // 10 seconds

// Interrupt Service Routine (ISR)
void IRAM_ATTR pulseCounter3() { flow3_pulse_count++; }
void IRAM_ATTR pulseCounter4() { flow4_pulse_count++; }

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi Connected!");

    // MQTT Setup
    espClient.setInsecure();
    mqttClient.setServer(mqtt_server, mqtt_port);
    reconnectMQTT();

    // Start Blynk
    Blynk.begin(auth, ssid, password);

    // Setup Sensors
    pinMode(FLOW_SENSOR_3, INPUT);
    pinMode(FLOW_SENSOR_4, INPUT);
    pinMode(PRESSURE_SENSOR, INPUT);
    pinMode(TAMPER_SENSOR, INPUT_PULLUP);  // Tamper Detection Setup
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_3), pulseCounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_4), pulseCounter4, RISING);
}

// ✅ Reconnect MQTT
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("🔄 Connecting to HiveMQ...");
        if (mqttClient.connect(nodeID, mqtt_user, mqtt_pass)) {
            Serial.println("✅ MQTT Connected!");
            mqttClient.subscribe(mqtt_topic_sub.c_str());
        } else {
            Serial.println("❌ Failed, retrying in 5s...");
            delay(5000);
        }
    }
}

// 🚨 Tamper Detection
void checkTamper() {
    if (digitalRead(TAMPER_SENSOR) == LOW) {
        Serial.println("🚨 Tamper Alert: Enclosure Opened!");
        Blynk.virtualWrite(V31, 1);
        mqttClient.publish("esp32/security", "{\"node\":\"node2\",\"tamper\":true}");
    } else {
        Blynk.virtualWrite(V31, 0);
    }
}

// ✅ Blynk Button to Manually Reset Alarm (V22)
BLYNK_WRITE(V22) {
    int value = param.asInt();
    if (value == 1) {
        alarmActive = false;
        alarmStartTime = 0;  // Reset alarm timer
        Blynk.virtualWrite(V10, 0);
        Serial.println("🔄 Alarm manually reset via Blynk");
        mqttClient.publish("esp32/alarm", "✅ Alarm Manually Reset");
    }
}

void loop() {
    Blynk.run();
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
    checkTamper();

    static unsigned long lastPublish = 0;
    unsigned long now = millis();

    if (now - lastPublish >= 1000) {
        float flowRate3 = (flow3_pulse_count / 7.5);
        float flowRate4 = (flow4_pulse_count / 7.5);
        flow3_pulse_count = 0;
        flow4_pulse_count = 0;

        // Read Pressure Sensor
        int rawPressure = analogRead(PRESSURE_SENSOR);
        float voltage = rawPressure * (3.3 / 4095.0);
        float pressureMPa = ((voltage - 0.5) / (4.0)) * 1.2;
        float pressureBar = pressureMPa * 10;

        // ✅ New Alarm Logic
        if (flowRate3 > 0 && flowRate4 > 0) {
            float difference = abs(flowRate3 - flowRate4);
            
            // If difference > 3 L/min for 10 sec, activate alarm
            if (difference > 3) {
                if (!alarmActive && (now - lastMismatchTime >= ALARM_DELAY)) {
                    alarmActive = true;
                    alarmStartTime = now;  // Store alarm activation time
                    Blynk.virtualWrite(V10, 1);
                    mqttClient.publish("esp32/alarm", "🚨 Alarm Triggered: Flow Mismatch > 3L/min");
                    Serial.println("🚨 Alarm Triggered!");
                }
            } else {
                lastMismatchTime = now;  // Reset timer if flow is normal
            }
        }

        // ✅ Auto Reset Alarm After 1 Hour
        if (alarmActive && (now - alarmStartTime >= ALARM_DURATION)) {
            alarmActive = false;
            alarmStartTime = 0;
            Blynk.virtualWrite(V10, 0);
            mqttClient.publish("esp32/alarm", "✅ Alarm Auto-Reset After 1 Hour");
            Serial.println("✅ Alarm Reset Automatically After 1 Hour");
        }

        // ✅ MQTT Payload
        char payload[150];
        sprintf(payload, "Flow3: %.2f L/min, Flow4: %.2f L/min, Pressure: %.2f bar", flowRate3, flowRate4, pressureBar);
        mqttClient.publish(mqtt_topic_pub.c_str(), payload);
        Serial.println("📤 Published: " + String(payload));

        // ✅ Send Data to Blynk
        Blynk.virtualWrite(V3, flowRate3);
        Blynk.virtualWrite(V4, flowRate4);
        Blynk.virtualWrite(V5, pressureBar);

        lastPublish = now;
    }
}