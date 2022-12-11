#include <Arduino.h>
#include <micro_ros_platformio.h>
// #include "arduino/wifi/micro_ros_transport.h"
#include <esp_camera.h>
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static const char *TAG = "example:take_picture";

rcl_publisher_t publisher;
rcl_publisher_t publisher_img;
std_msgs__msg__Int32 msg;
sensor_msgs__msg__CompressedImage msg_img;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer2;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Error handle loop
void error_loop() {
  while(1) {
    delay(1);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void timer_callback_img(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    camera_fb_t *img =  esp_camera_fb_get();

    if (img != NULL)
    {
      msg.data = img->len;
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
      msg_img.data.size = img->len;
      memcpy(msg_img.data.data, img->buf, img->len);
      msg_img.header.frame_id = micro_ros_string_utilities_set(msg_img.header.frame_id, "cam_frame");
      msg_img.format = micro_ros_string_utilities_set(msg_img.format, "jpeg");
      RCSOFTCHECK(rcl_publish(&publisher_img, &msg_img, NULL));
      // ESP_LOGE(TAG, "Picture taken! Its size was: %zu bytes %d %d", 
      // img->len,img->width,img->height);

      esp_camera_fb_return(img);
    }
    else
    {
      // ESP_LOGE(TAG, "Picture not taken!");
    }

  }
}

void setup() {
  // Configure serial transport
  // Serial.begin(115200);
  // set_microros_serial_transports(Serial);
  // Configure Wi Fi transport
  IPAddress agent_ip(192, 168, 43, 6);
  size_t agent_port = 8888;

  char ssid[] = "hotspot";
  char psk[]= "12345678";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    //.pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    //.frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 5, //0-63 lower number means higher quality
    .fb_count = 2,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Camera Init Failed");
    return;
  }


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  msg.data = 1;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));


  RCCHECK(rclc_publisher_init_default(
    &publisher_img,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "micro_ros_image_publisher"));

  msg.data = 2;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  
  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  msg.data = 4;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  
  // create timer,
  const unsigned int timer_timeout2 = 100;
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer_timeout2),
    timer_callback_img));

  msg.data = 5;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));


  msg.data = 6;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));


  msg.data = 0;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  
	static micro_ros_utilities_memory_conf_t conf = {};

	// OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences

	conf.max_string_capacity = 50;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;

	// OPTIONALLY this struct can store rules for specific members
	// !! Using the API with rules will use dynamic memory allocations for handling strings !!

	micro_ros_utilities_memory_rule_t rules[] = {
		{"header.frame_id", 30},
		{"format",4},
		{"data", 40000}
	};
	conf.rules = rules;
	conf.n_rules = sizeof(rules) / sizeof(rules[0]);
	
	micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
		&msg_img,
		conf
	);
}

void loop() {
  // delay(RCL_MS_TO_NS(1));
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000000)));
  msg.data++;
  // Serial.print("Loop");
}
