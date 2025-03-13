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
const char* nodeID = "node1";

// MQTT Topics
String mqtt_topic_pub = String("esp32/") + nodeID + "/sensor";
String mqtt_topic_sub = String("esp32/") + nodeID + "/command";

// Sensor Pins
#define FLOW_SENSOR_1 14
#define FLOW_SENSOR_2 27
#define PRESSURE_SENSOR 34
#define TAMPER_SENSOR 26  // Tamper Detection GPIO Pin

// Flow Sensor Variables
volatile int flow1_pulse_count = 0;
volatile int flow2_pulse_count = 0;

// Alarm Logic Variables
bool alarmActive = false;
unsigned long lastMismatchTime = 0;
unsigned long alarmStartTime = 0;
const unsigned long ALARM_DURATION = 3600000;  // 1 hour
const unsigned long ALARM_DELAY = 10000;  // 10 seconds

// Interrupt Service Routine (ISR)
void IRAM_ATTR pulseCounter1() { flow1_pulse_count++; }
void IRAM_ATTR pulseCounter2() { flow2_pulse_count++; }

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
    pinMode(FLOW_SENSOR_1, INPUT);
    pinMode(FLOW_SENSOR_2, INPUT);
    pinMode(PRESSURE_SENSOR, INPUT);
    pinMode(TAMPER_SENSOR, INPUT_PULLUP);  // Tamper Detection Setup
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_1), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_2), pulseCounter2, RISING);
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
        Blynk.virtualWrite(V30, 1);
        mqttClient.publish("esp32/security", "{\"node\":\"node1\",\"tamper\":true}");
    } else {
        Blynk.virtualWrite(V30, 0);
    }
}

// ✅ Blynk Button to Manually Reset Alarm (V21)
BLYNK_WRITE(V21) {
    int value = param.asInt();
    if (value == 1) {
        alarmActive = false;
        alarmStartTime = 0;  // Reset alarm timer
        lastMismatchTime = 0; // Reset mismatch timer
        Blynk.virtualWrite(V8, 0);
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
        float flowRate1 = (flow1_pulse_count / 7.5);
        float flowRate2 = (flow2_pulse_count / 7.5);
        flow1_pulse_count = 0;
        flow2_pulse_count = 0;

        // Read Pressure Sensor
        int rawPressure = analogRead(PRESSURE_SENSOR);
        float voltage = rawPressure * (3.3 / 4095.0);
        float pressureMPa = ((voltage - 0.5) / (4.0)) * 1.2;
        float pressureBar = pressureMPa * 10;

        if (flowRate1 > 0 && flowRate2 > 0) {
            float difference = abs(flowRate1 - flowRate2);
            
            if (difference > 3) {
                if (!alarmActive && lastMismatchTime == 0) {
                    lastMismatchTime = now;
                }
                if (!alarmActive && (now - lastMismatchTime >= ALARM_DELAY)) {
                    alarmActive = true;
                    alarmStartTime = now;
                    Blynk.virtualWrite(V8, 1);
                    mqttClient.publish("esp32/alarm", "🚨 Alarm Triggered: Flow Mismatch > 3L/min");
                    Serial.println("🚨 Alarm Triggered!");
                }
            } else {
                lastMismatchTime = 0;
            }
        }

        if (alarmActive && (now - alarmStartTime >= ALARM_DURATION)) {
            alarmActive = false;
            alarmStartTime = 0;
            Blynk.virtualWrite(V8, 0);
            mqttClient.publish("esp32/alarm", "✅ Alarm Auto-Reset After 1 Hour");
            Serial.println("✅ Alarm Reset Automatically After 1 Hour");
        }

        char payload[150];
        sprintf(payload, "Flow1: %.2f L/min, Flow2: %.2f L/min, Pressure: %.2f bar", flowRate1, flowRate2, pressureBar);
        mqttClient.publish(mqtt_topic_pub.c_str(), payload);
        Serial.println("📤 Published: " + String(payload));

        Blynk.virtualWrite(V1, flowRate1);
        Blynk.virtualWrite(V2, flowRate2);
        Blynk.virtualWrite(V3, pressureBar);

        lastPublish = now;
    }
}
node-1