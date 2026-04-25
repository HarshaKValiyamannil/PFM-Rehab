#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Lume_PFM_Rehab_Monitor_inferencing.h>

// Sensor: Arduino Nano 33 BLE Sense Rev1 uses LSM9DS1 for 9-axis sensor fusion.
#define LED_RED     22
#define LED_GREEN   23
#define LED_BLUE    24

// Sampling configuration
#define SAMPLING_INTERVAL_MS EI_CLASSIFIER_INTERVAL_MS  // 10ms for this model
#define NUM_SAMPLES         EI_CLASSIFIER_RAW_SAMPLE_COUNT  // 200 samples
#define INFERENCE_INTERVAL_SAMPLES 10  // Run every 100ms with a 2s sliding window

// Smoothing configuration
#define SMOOTHING_COUNT     3
#define ANOMALY_THRESHOLD   10.0f
#define GYRO_CALIBRATION_SAMPLES 200
#define REST_STILL_SAMPLES 35
#define REST_GYRO_THRESHOLD_DPS 6.0f
#define REST_ACCEL_MAG_TOLERANCE 0.9f

// Global variables
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static int sample_count = 0;
static bool imu_ready = false;
static bool mag_ready = false;

// Smoothing counters
static int correct_count = 0;
static int push_count = 0;
static int anomaly_count = 0;

// Current LED states
static bool led_green_on = false;
static bool led_red_on = false;
static bool led_blue_on = false;

static bool debug_nn = false;
static unsigned long last_sensor_error_time = 0;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static int still_sample_count = 0;
static bool rest_override_active = false;

// Function declarations
void collect_samples();
void perform_inference();
void update_led_states(ei_impulse_result_t result);
void control_leds();
void calibrate_gyroscope();
void keep_sliding_window();
void update_rest_override(float acc_x_ms2, float acc_y_ms2, float acc_z_ms2,
                          float gyr_x, float gyr_y, float gyr_z);
void apply_rest_override(ei_impulse_result_t *result);
bool is_valid_mag_value(float x, float y, float z);
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(2000);  // Give serial monitor time to connect
    Serial.println("PFM Rehab Monitor Starting...");

    // Initialize LEDs
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // Turn off all LEDs initially
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);

    // Initialize LSM9DS1 IMU (includes accelerometer, gyroscope, and magnetometer)
    if (!IMU.begin()) {
        Serial.println("Failed to initialize LSM9DS1 IMU!");
        while (1);  // Halt if IMU fails
    }

    Serial.println("LSM9DS1 IMU initialized successfully");
    Serial.print("Accelerometer sample rate: ");
    Serial.println(IMU.accelerationSampleRate());
    Serial.print("Gyroscope sample rate: ");
    Serial.println(IMU.gyroscopeSampleRate());
    Serial.print("Magnetometer sample rate: ");
    Serial.println(IMU.magneticFieldSampleRate());

    calibrate_gyroscope();
    imu_ready = true;

    Serial.println("Setup complete. Starting inference loop...");
}

void loop() {
    if (!imu_ready) return;

    // Collect samples
    collect_samples();

    // Run inference when we have enough samples
    if (sample_count >= NUM_SAMPLES) {
        perform_inference();
        keep_sliding_window();
    }
}

void collect_samples() {
    static unsigned long last_sample_time = 0;
    static float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
    static float gyr_x = 0.0f, gyr_y = 0.0f, gyr_z = 0.0f;
    static float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
    unsigned long current_time = millis();

    // Sample at specified frequency
    if (current_time - last_sample_time >= SAMPLING_INTERVAL_MS) {
        last_sample_time = current_time;

        bool acc_ok = IMU.readAcceleration(acc_x, acc_y, acc_z);
        bool gyr_ok = IMU.readGyroscope(gyr_x, gyr_y, gyr_z);
        if (gyr_ok) {
            gyr_x -= gyro_bias_x;
            gyr_y -= gyro_bias_y;
            gyr_z -= gyro_bias_z;
        }

        if (IMU.magneticFieldAvailable()) {
            float new_mag_x, new_mag_y, new_mag_z;
            bool mag_ok = IMU.readMagneticField(new_mag_x, new_mag_y, new_mag_z);
            if (mag_ok && is_valid_mag_value(new_mag_x, new_mag_y, new_mag_z)) {
                mag_x = new_mag_x;
                mag_y = new_mag_y;
                mag_z = new_mag_z;
                mag_ready = true;
            }
        }

        if (!acc_ok || !gyr_ok || !mag_ready) {
            if (current_time - last_sensor_error_time >= 1000) {
                last_sensor_error_time = current_time;
                Serial.print("Waiting for valid IMU data: acc=");
                Serial.print(acc_ok ? "OK" : "FAIL");
                Serial.print(" gyr=");
                Serial.print(gyr_ok ? "OK" : "FAIL");
                Serial.print(" mag=");
                Serial.println(mag_ready ? "OK" : "WAIT");
            }
            return;
        }

        // Magnetometer data updates more slowly than the 100 Hz model interval,
        // so reuse its last value between reads.
        int buffer_index = sample_count * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        float acc_x_ms2 = acc_x * 9.80665f;
        float acc_y_ms2 = acc_y * 9.80665f;
        float acc_z_ms2 = acc_z * 9.80665f;

        update_rest_override(acc_x_ms2, acc_y_ms2, acc_z_ms2, gyr_x, gyr_y, gyr_z);

        buffer[buffer_index]     = acc_x_ms2;  // accX in m/s^2
        buffer[buffer_index + 1] = acc_y_ms2;  // accY in m/s^2
        buffer[buffer_index + 2] = acc_z_ms2;  // accZ in m/s^2
        buffer[buffer_index + 3] = gyr_x;             // gyrX
        buffer[buffer_index + 4] = gyr_y;             // gyrY
        buffer[buffer_index + 5] = gyr_z;             // gyrZ
        buffer[buffer_index + 6] = mag_x;             // magX
        buffer[buffer_index + 7] = mag_y;             // magY
        buffer[buffer_index + 8] = mag_z;             // magZ

        sample_count++;

        // Debug: print first few samples
        if (sample_count <= 3) {
            Serial.print("Sample ");
            Serial.print(sample_count);
            Serial.print(": acc=");
            Serial.print(buffer[buffer_index], 2);
            Serial.print(",");
            Serial.print(buffer[buffer_index + 1], 2);
            Serial.print(",");
            Serial.print(buffer[buffer_index + 2], 2);
            Serial.print(" gyr=");
            Serial.print(gyr_x, 2);
            Serial.print(",");
            Serial.print(gyr_y, 2);
            Serial.print(",");
            Serial.print(gyr_z, 2);
            Serial.print(" mag=");
            Serial.print(mag_x, 2);
            Serial.print(",");
            Serial.print(mag_y, 2);
            Serial.print(",");
            Serial.println(mag_z, 2);
        }
    }
}

bool is_valid_mag_value(float x, float y, float z) {
    return x > -32000.0f && y > -32000.0f && z > -32000.0f;
}

void update_rest_override(float acc_x_ms2, float acc_y_ms2, float acc_z_ms2,
                          float gyr_x, float gyr_y, float gyr_z) {
    float acc_mag = sqrt(
        (acc_x_ms2 * acc_x_ms2) +
        (acc_y_ms2 * acc_y_ms2) +
        (acc_z_ms2 * acc_z_ms2)
    );
    float gyro_mag = sqrt((gyr_x * gyr_x) + (gyr_y * gyr_y) + (gyr_z * gyr_z));
    bool still = gyro_mag < REST_GYRO_THRESHOLD_DPS &&
                 fabs(acc_mag - 9.80665f) < REST_ACCEL_MAG_TOLERANCE;

    if (still) {
        still_sample_count = min(still_sample_count + 1, REST_STILL_SAMPLES);
    } else {
        still_sample_count = 0;
    }

    rest_override_active = still_sample_count >= REST_STILL_SAMPLES;
}

void apply_rest_override(ei_impulse_result_t *result) {
    if (!rest_override_active) {
        return;
    }

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (strcmp(result->classification[i].label, "Rest") == 0) {
            result->classification[i].value = 1.0f;
        } else {
            result->classification[i].value = 0.0f;
        }
    }
}

void calibrate_gyroscope() {
    Serial.println("Keep the board still. Calibrating gyroscope...");

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    int samples = 0;

    while (samples < GYRO_CALIBRATION_SAMPLES) {
        float x, y, z;
        if (IMU.readGyroscope(x, y, z)) {
            sum_x += x;
            sum_y += y;
            sum_z += z;
            samples++;
        }
        delay(10);
    }

    gyro_bias_x = sum_x / GYRO_CALIBRATION_SAMPLES;
    gyro_bias_y = sum_y / GYRO_CALIBRATION_SAMPLES;
    gyro_bias_z = sum_z / GYRO_CALIBRATION_SAMPLES;

    Serial.print("Gyro bias: ");
    Serial.print(gyro_bias_x, 2);
    Serial.print(",");
    Serial.print(gyro_bias_y, 2);
    Serial.print(",");
    Serial.println(gyro_bias_z, 2);
}

void perform_inference() {
    Serial.println("\n--- Running Inference ---");
    Serial.print("Window end: ");
    Serial.print(millis());
    Serial.println(" ms");

    // Prepare signal for Edge Impulse
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    // Run classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, debug_nn);
    if (res != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier (");
        Serial.print(res);
        Serial.println(")");
        return;
    }

    apply_rest_override(&result);

    // Print results
    Serial.println("Predictions:");
    const char *top_label = "";
    float top_value = 0.0f;
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.print("  ");
        Serial.print(result.classification[i].label);
        Serial.print(": ");
        Serial.println(result.classification[i].value, 5);

        if (result.classification[i].value > top_value) {
            top_label = result.classification[i].label;
            top_value = result.classification[i].value;
        }
    }
    if (rest_override_active) {
        Serial.println("Rest override: ON");
    }
    Serial.print("Top: ");
    Serial.print(top_label);
    Serial.print(" (");
    Serial.print(top_value, 5);
    Serial.println(")");
    Serial.print("Anomaly score: ");
    Serial.println(result.anomaly, 5);

    // Apply smoothing logic
    update_led_states(result);

    // Control LEDs based on smoothed results
    control_leds();
}

void keep_sliding_window() {
    const int samples_to_keep = NUM_SAMPLES - INFERENCE_INTERVAL_SAMPLES;
    const int values_to_keep = samples_to_keep * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
    const int values_to_drop = INFERENCE_INTERVAL_SAMPLES * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;

    memmove(buffer, buffer + values_to_drop, values_to_keep * sizeof(float));
    sample_count = samples_to_keep;
}

void update_led_states(ei_impulse_result_t result) {
    // Get probabilities
    float correct_prob = 0.0f;
    float push_prob = 0.0f;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (strcmp(result.classification[i].label, "Correct") == 0) {
            correct_prob = result.classification[i].value;
        } else if (strcmp(result.classification[i].label, "Push") == 0) {
            push_prob = result.classification[i].value;
        }
    }

    // Update smoothing counters
    if (correct_prob > 0.8f) {
        correct_count = min(correct_count + 1, SMOOTHING_COUNT);
    } else {
        correct_count = max(correct_count - 1, 0);
    }

    if (push_prob > 0.8f) {
        push_count = min(push_count + 1, SMOOTHING_COUNT);
    } else {
        push_count = max(push_count - 1, 0);
    }

    if (result.anomaly > ANOMALY_THRESHOLD) {
        anomaly_count = min(anomaly_count + 1, SMOOTHING_COUNT);
    } else {
        anomaly_count = max(anomaly_count - 1, 0);
    }

    // Update LED states based on smoothed counts
    led_green_on = (correct_count >= SMOOTHING_COUNT);
    led_red_on = (push_count >= SMOOTHING_COUNT);
    led_blue_on = (anomaly_count >= SMOOTHING_COUNT);
}

void control_leds() {
    digitalWrite(LED_GREEN, led_green_on ? HIGH : LOW);
    digitalWrite(LED_RED, led_red_on ? HIGH : LOW);
    digitalWrite(LED_BLUE, led_blue_on ? HIGH : LOW);

    Serial.print("LEDs - Green:");
    Serial.print(led_green_on ? "ON" : "OFF");
    Serial.print(" Red:");
    Serial.print(led_red_on ? "ON" : "OFF");
    Serial.print(" Blue:");
    Serial.println(led_blue_on ? "ON" : "OFF");
}

// Callback function for Edge Impulse signal
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = buffer[offset + i];
    }
    return 0;
}

