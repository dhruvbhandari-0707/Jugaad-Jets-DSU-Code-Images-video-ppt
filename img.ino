#include "esp_camera.h"
#include "fd_forward.h"  // The Face Detection Library
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===================
// Select Camera Model
// ===================
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h" 

// ===================
// Configuration
// ===================
mtmn_config_t mtmn_config = {0}; // Detection settings

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  Serial.println("Starting Face Detector...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  // CRITICAL FOR FACE DETECT:
  // Use RGB565 (Raw color) or JPEG. 
  // We will convert whatever we get to RGB888 for the AI.
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_QVGA; // 320x240 (Do not go higher)
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Camera Init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x", err);
    return;
  }

  // Load default face detection parameters
  mtmn_config = mtmn_init_config();
  
  Serial.println("Ready! Point camera at a face.");
}

void loop() {
  // 1. Capture Frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // 2. Prepare Image for AI (Convert to RGB888 Matrix)
  // The Face Detector needs a specific matrix structure (dl_matrix3du_t)
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  
  if (!image_matrix) {
    Serial.println("Memory Allocation Failed!");
    esp_camera_fb_return(fb);
    return;
  }

  // Convert the raw frame buffer to the matrix format
  // fmt2rgb888 decodes JPEG or RGB565 into standard RGB
  if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
    Serial.println("Image conversion failed");
    dl_matrix3du_free(image_matrix);
    esp_camera_fb_return(fb);
    return;
  }

  // 3. Run Face Detection
  // This function returns a list of boxes (faces) found
  box_array_t *boxes = face_detect(image_matrix, &mtmn_config);

  // 4. Process Results
  if (boxes) {
    Serial.printf("FACES DETECTED: %d \n", boxes->len);
    
    // Example: If a face is found, you could trigger your buzzer here
    // digitalWrite(BUZZER_PIN, HIGH); 

    // Free the detection memory (Very Important!)
    // If you don't do this, the ESP32 will crash after 5 seconds.
    dl_lib_free(boxes->score);
    dl_lib_free(boxes->box);
    dl_lib_free(boxes->landmark);
    dl_lib_free(boxes);
  } else {
    // No face
    // digitalWrite(BUZZER_PIN, LOW); 
  }

  // 5. Clean up
  dl_matrix3du_free(image_matrix); // Free the image matrix
  esp_camera_fb_return(fb);        // Return the camera frame
  
  delay(100); // Small delay to let the CPU breathe
}