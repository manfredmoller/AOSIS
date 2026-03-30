#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

// Network config
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress teensy_ip(192, 168, 10, 2);
IPAddress agent_ip(192, 168, 10, 1);
uint16_t agent_port = 8888;
uint16_t teensy_port = 9999;

EthernetUDP udp;

// Micro-ROS transport callbacks
bool ethernet_transport_open(struct uxrCustomTransport * transport) {
    Ethernet.begin(mac, teensy_ip);
    delay(500);
    return udp.begin(teensy_port);
}

bool ethernet_transport_close(struct uxrCustomTransport * transport) {
    udp.stop();
    return true;
}

size_t ethernet_transport_write(struct uxrCustomTransport * transport,
    const uint8_t * buf, size_t len, uint8_t * err) {
    udp.beginPacket(agent_ip, agent_port);
    size_t sent = udp.write(buf, len);
    udp.endPacket();
    return sent;
}

size_t ethernet_transport_read(struct uxrCustomTransport * transport,
    uint8_t * buf, size_t len, int timeout, uint8_t * err) {
    uint32_t start = millis();
    while (millis() - start < (uint32_t)timeout) {
        int psize = udp.parsePacket();
        if (psize > 0) {
            return udp.read(buf, len);
        }
    }
    return 0;
}

// Micro-ROS objects
rcl_publisher_t joint_state_publisher;
rcl_subscription_t cmd_vel_subscriber;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray cmd_vel_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float axis_positions[3] = {0.0, 0.0, 0.0};
float axis_velocities[3] = {0.0, 0.0, 0.0};
static double positions[3] = {0.0, 0.0, 0.0};
static double velocities[3] = {0.0, 0.0, 0.0};
static float cmd_data[3] = {0.0, 0.0, 0.0};

#define LED_PIN LED_BUILTIN
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) return false; }

enum AgentState { WAITING, CONNECTED };
AgentState agent_state = WAITING;

void blink_slow() {
    digitalWrite(LED_PIN, HIGH); delay(500);
    digitalWrite(LED_PIN, LOW);  delay(500);
}

void cmd_vel_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 3) {
        axis_velocities[0] = msg->data.data[0];
        axis_velocities[1] = msg->data.data[1];
        axis_velocities[2] = msg->data.data[2];
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        joint_state_msg.position.data[0] = axis_positions[0];
        joint_state_msg.position.data[1] = axis_positions[1];
        joint_state_msg.position.data[2] = axis_positions[2];
        joint_state_msg.velocity.data[0] = axis_velocities[0];
        joint_state_msg.velocity.data[1] = axis_velocities[1];
        joint_state_msg.velocity.data[2] = axis_velocities[2];
        int64_t now = rmw_uros_epoch_millis();
        joint_state_msg.header.stamp.sec = now / 1000;
        joint_state_msg.header.stamp.nanosec = (now % 1000) * 1000000;
        rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

bool create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "aosis_motion_controller", "", &support));
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states"));
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/aosis/cmd_vel"));
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

    joint_state_msg.position.data = positions;
    joint_state_msg.position.size = 3;
    joint_state_msg.position.capacity = 3;
    joint_state_msg.velocity.data = velocities;
    joint_state_msg.velocity.size = 3;
    joint_state_msg.velocity.capacity = 3;
    cmd_vel_msg.data.data = cmd_data;
    cmd_vel_msg.data.size = 3;
    cmd_vel_msg.data.capacity = 3;

    rmw_uros_sync_session(1000);
    return true;
}

void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    rmw_uros_set_custom_transport(
        false,
        NULL,
        ethernet_transport_open,
        ethernet_transport_close,
        ethernet_transport_write,
        ethernet_transport_read
    );
    delay(2000);
}

void loop() {
    switch (agent_state) {
        case WAITING:
            blink_slow();
            if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
                if (create_entities()) {
                    agent_state = CONNECTED;
                    digitalWrite(LED_PIN, HIGH);
                }
            }
            break;

        case CONNECTED:
            if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                destroy_entities();
                agent_state = WAITING;
            } else {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            }
            break;
    }
}
