/**
 *  AOSIS Motion Controller — Production Revision
 *  Teensy 4.1 via NativeEthernet + TMCStepper Safe Software SPI 
 */ 
#include <Arduino.h> 
#include <TMCStepper.h> 
#include <NativeEthernet.h> 
#include <NativeEthernetUdp.h> 
#include <micro_ros_platformio.h> 
#include <rmw_microros/rmw_microros.h> 
#include <rcl/rcl.h> 
#include <rclc/rclc.h> 
#include <rclc/executor.h> 
#include <std_msgs/msg/float32_multi_array.h>

// ── SPI Pins ─────────────────────────────── 
#define SPI_MOSI  11 
#define SPI_MISO  12 
#define SPI_SCK   14  // FIX: Moved away from Pin 13 (LED_BUILTIN collision)

// ── X Axis Pins ───────────────────────────────────────── 
#define X_STEP    36 
#define X_DIR     37 
#define X_EN      7 
#define X_CS      8

#define R_SENSE   0.075f // BTT TMC5160 Pro specific sense resistor

TMC5160Stepper driver_x(X_CS, R_SENSE, SPI_MOSI, SPI_MISO, SPI_SCK);

// ── Step Generation Config ────────────────────────────────────── 
#define STEP_TIMER_HZ         50000
#define STEP_TIMER_US         20
#define VEL_TO_STEPS_PER_SEC  5000.0f
#define MAX_STEPS_PER_SEC     25000.0f 
#define ACCEL_PER_TICK        0.5f

struct StepAxis { 
    volatile float current_rate;
    volatile float target_rate;
    volatile float accumulator;
    volatile int32_t position;
};

StepAxis axis_x = { 0.0f, 0.0f, 0.0f, 0 };
IntervalTimer stepTimer;
volatile uint32_t last_cmd_time = 0; // Staleness watchdog

// ── Step Timer ISR (DDS Step Generator) ───────────────────────── 
void stepTimerISR() { 
    // Always clear the step pulse from the previous ISR execution
    digitalWriteFast(X_STEP, LOW);

    if (axis_x.current_rate < axis_x.target_rate) { 
        axis_x.current_rate += ACCEL_PER_TICK; 
        if (axis_x.current_rate > axis_x.target_rate) axis_x.current_rate = axis_x.target_rate; 
    } else if (axis_x.current_rate > axis_x.target_rate) { 
        axis_x.current_rate -= ACCEL_PER_TICK; 
        if (axis_x.current_rate < axis_x.target_rate) axis_x.current_rate = axis_x.target_rate; 
    }

    if (axis_x.current_rate == 0.0f) return;

    bool forward = axis_x.current_rate > 0; 
    static bool last_dir = forward;

    // FIX: Enforce 20ns+ TMC5160 DIR-to-STEP setup time on reversal
    if (forward != last_dir) {
        digitalWriteFast(X_DIR, forward ? HIGH : LOW);
        delayNanoseconds(40);
        last_dir = forward;
    } else {
        digitalWriteFast(X_DIR, forward ? HIGH : LOW);
    }

    // FIX: Use float-specific absolute value (abs() on float is undefined)
    float abs_rate = fabsf(axis_x.current_rate); 
    axis_x.accumulator += (abs_rate / STEP_TIMER_HZ);

    if (axis_x.accumulator >= 1.0f) { 
        axis_x.accumulator -= 1.0f; 
        digitalWriteFast(X_STEP, HIGH); // Latched high, will be pulled low on next ISR tick
        axis_x.position += forward ? 1 : -1; 
    } 
}

// ── Network & Micro-ROS Transports ──────────────────────────────
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress teensy_ip(192, 168, 10, 2);
IPAddress agent_ip(192, 168, 10, 1);
uint16_t agent_port = 8888;
uint16_t teensy_port = 9999;
EthernetUDP udp;

bool ethernet_transport_open(struct uxrCustomTransport* t) {
    Ethernet.begin(mac, teensy_ip);
    delay(500);
    return udp.begin(teensy_port);
}

bool ethernet_transport_close(struct uxrCustomTransport* t) {
    udp.stop();
    return true;
}

size_t ethernet_transport_write(struct uxrCustomTransport* t, const uint8_t* buf, size_t len, uint8_t* err) {
    udp.beginPacket(agent_ip, agent_port);
    size_t s = udp.write(buf, len);
    udp.endPacket();
    return s;
}

size_t ethernet_transport_read(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    uint32_t start = millis();
    while (millis() - start < (uint32_t)timeout) {
        if (udp.parsePacket() > 0) return udp.read(buf, len);
    }
    return 0;

}

// ── ROS 2 Entities ────────────────────────────── 
rcl_subscription_t cmd_vel_sub; 
std_msgs__msg__Float32MultiArray cmd_vel_msg; 
rclc_executor_t executor; 
rclc_support_t support; 
rcl_allocator_t allocator; 
rcl_node_t node;

const int BUF_CAPACITY = 3; 
static float cmd_buf[BUF_CAPACITY] = {0.0f, 0.0f, 0.0f};

enum AgentState { WAITING, CONNECTED } agent_state = WAITING;

#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) return false; }

void cmd_vel_callback(const void* msgin) { 
    const auto* msg = (const std_msgs__msg__Float32MultiArray*)msgin; 
    if (msg->data.size < 1) return;

    int x_axis_idx = 0; 
    float target = msg->data.data[x_axis_idx] * VEL_TO_STEPS_PER_SEC;

    if (target > MAX_STEPS_PER_SEC) target = MAX_STEPS_PER_SEC; 
    if (target < -MAX_STEPS_PER_SEC) target = -MAX_STEPS_PER_SEC;

    // FIX: Atomic write and watchdog update
    noInterrupts();
    axis_x.target_rate = target; 
    last_cmd_time = millis();
    interrupts();
    
    Serial.printf("[Callback] Target vel: %.2f\n", msg->data.data[x_axis_idx]);
}

bool create_entities() { 
    allocator = rcl_get_default_allocator(); 
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); 
    RCCHECK(rclc_node_init_default(&node, "aosis_motion_controller", "", &support));

    RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/aosis/cmd_vel"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); 
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

    cmd_vel_msg.data.data = cmd_buf; 
    cmd_vel_msg.data.size = BUF_CAPACITY; 
    cmd_vel_msg.data.capacity = BUF_CAPACITY;

    rmw_uros_sync_session(1000); 
    return true; 
}

void destroy_entities() { 
    rmw_context_t* ctx = rcl_context_get_rmw_context(&support.context); 
    rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0); 
    
    // FIX: Catch the return variables to satisfy the GCC compiler warnings
    rcl_ret_t rc;
    rc = rcl_subscription_fini(&cmd_vel_sub, &node); 
    (void)rc; // Explicitly void the unused variable
    
    rclc_executor_fini(&executor); 
    
    rc = rcl_node_fini(&node); 
    (void)rc;
    
    rclc_support_fini(&support); 
}

// ── Main Setup & Loop ────────────────────────────── 
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- AOSIS Motion Controller (Production Edition) ---");

    rmw_uros_set_custom_transport(true, NULL, ethernet_transport_open, ethernet_transport_close, ethernet_transport_write, ethernet_transport_read);

    pinMode(X_EN, OUTPUT);
    digitalWriteFast(X_EN, HIGH); 

    pinMode(X_STEP, OUTPUT);
    digitalWriteFast(X_STEP, LOW);
    pinMode(X_DIR, OUTPUT);
    digitalWriteFast(X_DIR, LOW);
    pinMode(LED_BUILTIN, OUTPUT);

    // TMCStepper Safe Software SPI Init
    driver_x.begin(); 
    
    driver_x.bbmtime(15); 
    driver_x.bbmclks(4);
    
    driver_x.en_pwm_mode(false);   
    driver_x.pwm_autoscale(false); 
    
    driver_x.toff(5); 
    
    // FIX: Add hold current scaling (30% of run current) and enable interpolation
    driver_x.rms_current(1500, 0.3f); 
    driver_x.iholddelay(5);
    driver_x.intpol(true);

    driver_x.microsteps(16); 

    delay(100);
    digitalWriteFast(X_EN, LOW); 

    Serial.println("\n--- TMC5160 RAW REGISTER DUMP ---");
    Serial.print("IOIN   (0x04): 0x"); Serial.println(driver_x.IOIN(), HEX);
    Serial.print("GCONF  (0x00): 0x"); Serial.println(driver_x.GCONF(), HEX);
    Serial.print("GSTAT  (0x01): 0x"); Serial.println(driver_x.GSTAT(), HEX);
    Serial.print("DRV_STATUS   : 0x"); Serial.println(driver_x.DRV_STATUS(), HEX);
    Serial.println("---------------------------------\n");

    uint8_t connection_result = driver_x.test_connection();
    if (connection_result) {
        Serial.print("FATAL: Driver not communicating! Halting. Error code: ");
        Serial.println(connection_result);
        while(true) { // FIX: Strict halt on SPI failure
            digitalToggleFast(LED_BUILTIN);
            delay(100);
        }
    } else {
        Serial.println("SUCCESS: X-Axis driver fully configured!");
    }

    stepTimer.begin(stepTimerISR, STEP_TIMER_US);
}

void loop() { 
    static elapsedMillis logTimer; 
    static elapsedMillis pingTimer; 
    static elapsedMillis diagTimer; 

switch (agent_state) { 
    case WAITING: 
        digitalToggleFast(LED_BUILTIN); 
        delay(200); 

        if (logTimer > 5000) { 
            Serial.println("[loop] pinging Orin AGX agent..."); 
            logTimer = 0; 
        } 
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) { 
            Serial.println("[loop] Agent found! Connecting micro-ROS..."); 
            if (create_entities()) { 
                agent_state = CONNECTED; 
                digitalWriteFast(LED_BUILTIN, HIGH); 
                Serial.println("[loop] CONNECTED — Subscribed to /aosis/cmd_vel"); 
                pingTimer = 0;
                diagTimer = 0;
                last_cmd_time = millis();
            } 
          }

        break;    
 
        case CONNECTED:
            if (driver_x.GSTAT() & 0x01) { 
                Serial.println("\n[WARNING] Driver Reset Detected! Restoring config...");
                digitalWriteFast(X_EN, HIGH); 
                driver_x.bbmtime(15);
                driver_x.bbmclks(4);
                driver_x.push();       
                driver_x.GSTAT(0x01);  
                digitalWriteFast(X_EN, LOW);  
            }

            rclc_executor_spin_some(&executor, 10000000); 

            // FIX: Command Staleness Watchdog (150ms timeout)
            if (millis() - last_cmd_time > 150 && axis_x.target_rate != 0.0f) {
                noInterrupts();
                axis_x.target_rate = 0.0f;
                interrupts();
                Serial.println("[WARNING] Command Stale! Halting motor.");
            }

            // --- Restored Health Diagnostics (StallGuard Removed) ---
            if (diagTimer > 5000) {
                Serial.println("\n--- TMC5160 Health Diagnostics ---");
                Serial.print("Open Load Phase A (OLA): "); Serial.println(driver_x.ola() ? "WARNING: OPEN" : "OK");
                Serial.print("Open Load Phase B (OLB): "); Serial.println(driver_x.olb() ? "WARNING: OPEN" : "OK");
                Serial.print("Short to GND Phase A (S2GA): "); Serial.println(driver_x.s2ga() ? "WARNING: SHORT" : "OK");
                Serial.print("Short to GND Phase B (S2GB): "); Serial.println(driver_x.s2gb() ? "WARNING: SHORT" : "OK");
                Serial.print("Over Temp Pre-warn (OTPW): "); Serial.println(driver_x.otpw() ? "WARNING: HOT" : "OK");
                Serial.print("Over Temp Fault (OT): "); Serial.println(driver_x.ot() ? "FATAL: TOO HOT" : "OK");
                Serial.print("Actual Current Scaling (CS): "); Serial.println(driver_x.cs_actual());
                Serial.println("----------------------------------\n");
                diagTimer = 0;
            }

            if (pingTimer > 2000) {
                if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK) {
                    // FIX: Force motor stop on disconnect
                    noInterrupts();
                    axis_x.target_rate = 0.0f;
                    interrupts();
                    
                    Serial.println("[WARNING] Agent Disconnected. Halting.");
                    agent_state = WAITING;
                    destroy_entities();
                }
                pingTimer = 0;
            }
            break;
    }
}
