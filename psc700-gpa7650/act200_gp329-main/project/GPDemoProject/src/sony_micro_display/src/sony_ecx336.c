#include "sony_ecx336.h"
#include "DISP_YUYV_VGA.h"
#include "cmd_tx_task.h"
#include "drv_l1_rotator.h"
#include "drv_l1_scaler.h"
#include "drv_l2_display.h"
#include "drv_l2_rotator.h"
#include "drv_l2_scaler.h"
#include "fixation1_as.h"
#include "fixation1_ps.h"
#include "fixation2_as.h"
#include "fixation2_ps.h"
#include "gp_stdlib.h"
#include "gplib_mm_gplus.h"
#include "gplib_print_string.h"
#include "tft_sony_ecx336.h"

/**************************************************************************
 *                              D E F I N E                               *
 **************************************************************************/
#define DEFAULT_X       0
#define DEFAULT_Y       0

#define DEFAULT_COLOR   SONY_COLOR_GREEN
#define DEFAULT_PATTERN SONY_PATTERN_RECTANGLE

/* The 14x9 rectangle will overlap with a height of 2 pixels below
due to the optical system, resulting in a 14x14 square.*/
#define DEFAULT_RECT_WIDTH      14
#define DEFAULT_RECT_HEIGHT     9

#define WIDTH                   10 // The thickness of the outer rectangle.
#define DISTANCE_STEP           5  // Step size for animation movement.
#define TRANSITION_STEP         15 // Transition speed during animation.

#define ARRAY_WIDTH(arr)        (sizeof(arr[0]) / sizeof(arr[0][0]))
#define ARRAY_HEIGHT(arr)       (sizeof(arr) / sizeof(arr[0]))
#define ARRAY_WIDTH_BYTES(arr)  (sizeof(arr[0]))
#define ARRAY_HEIGHT_BYTES(arr) (sizeof(arr))

extern osThreadId ud_blink_id;
/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static INT16U       h_size, v_size;
xQueueHandle        queue_cmd_to_sony_task = NULL;
sony_task_message_t m_tft_data;         // Store pattern info
INT8U               m_brightness = 255; // Global brightness

// Animation parameters
INT8U volatile animation_state = ANIMATE_STATE_STOP;
INT16S volatile animation_distance = 0;
INT32U  transition_step = TRANSITION_STEP;
INT16U  animation_color;
INT32U  buf_size;
INT32U  base_disp_buf;
INT32U  transition_buf; // Full screen for transition animation
INT32S  x_point;
INT32S  y_point;
INT32S  x_start, x_end;
INT32S  y_start, y_end;
BOOLEAN b_show_animation = TRUE;

/**
 * @brief   Convert RGB888 to RGB565 with dimming factor
 * @param   color: RGB888
 * @param   dim_factor: Dim factor (0.0 - 1.0)
 * @return  RGB565 color
 */
INT16U rgb888_to_rgb565(INT32U color, float dim_factor) {
    if (dim_factor > 1.0f)
        dim_factor = 1.0f;
    if (dim_factor < 0.0f)
        dim_factor = 0.0f;

    INT8U r, g, b;
    r = (color >> 16) & 0xFF;
    g = (color >> 8) & 0xFF;
    b = color & 0xFF;

    r = (INT8U)(r * dim_factor);
    g = (INT8U)(g * dim_factor);
    b = (INT8U)(b * dim_factor);

    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

/**
 * @brief   Set the global brightness level.
 * @param   brightness: Brightness value (range: 0 to 255).
 * @return  TRUE if the brightness is valid; FALSE otherwise.
 */
BOOLEAN sony_set_brightness(INT16U brightness) {
    if (brightness > 255) {
        DBG_PRINT("Invalid brightness\r\n");
        return FALSE;
    }
    m_brightness = brightness;
    return TRUE;
}
/**
 * @brief   Mapping color index to color
 * @param   color_idx: color index
 * @return  ret (RGB565)
 */
INT16U sony_check_color(INT8U color_idx) {
    INT16U ret;
    float  dim = m_brightness / DEFAULT_BRIGHTNESS;
    switch (color_idx) {
    case SONY_COLOR_WHITE:
        ret = rgb888_to_rgb565(COLOR_WHITE, dim);
        break;
    case SONY_COLOR_RED:
        ret = rgb888_to_rgb565(COLOR_RED, dim);
        break;
    case SONY_COLOR_BLUE:
        ret = rgb888_to_rgb565(COLOR_BLUE, dim);
        break;
    case SONY_COLOR_GREEN:
    default:
        ret = rgb888_to_rgb565(COLOR_GREEN, dim);
        break;
    }
    return ret;
}

// Save the current pattern to the global m_tft_data to redraw it when the brightness changes
void update_current_pattern(sony_task_message_t msg) {
    m_tft_data = msg;
}

static void sony_free_transition_buffer(void) {
    gp_free((void*)transition_buf);
}

static void sony_stop_transition() {
    sony_free_transition_buffer();
}

static void sony_free_animation_buffer(void) {
    gp_free((void*)base_disp_buf);
}

void sony_stop_animation() {
    animation_state = ANIMATE_STATE_STOP;
    b_show_animation = TRUE;
    sony_free_animation_buffer();
    sony_free_transition_buffer();
}

/**
 * @brief   Draw a single pixel on the screen at a specified position.
 * @param   src: Pointer to the frame buffer.
 * @param   x: X-coordinate of the pixel.
 * @param   y: Y-coordinate of the pixel.
 * @param   color: RGB565 color value for the pixel.
 * @note    Ensures the pixel is within the valid display boundaries.
 */
void draw_pixel(INT16U* src, INT32S x, INT32S y, INT16U color) {
    if (x >= 0 && x < h_size && y >= 0 && y < v_size)
        src[y * (h_size + 1) + x] = color;
}

/**
 * @brief Draw a binary image on the frame buffer with a colored center region.
 *
 * @param src         Pointer to the frame buffer (target image).
 * @param image_array Input binary image data (1 for active pixel, 0 for inactive).
 * @param img_w       Width of the input image.
 * @param img_h       Height of the input image.
 * @param offset_x    Horizontal offset to shift the image position.
 * @param offset_y    Vertical offset to shift the image position.
 * @param color       RGB565 color value used for active pixels outside the center region.
 */
void draw_image(
    INT16U*      src,
    const INT8U* image_array,
    INT32S       img_w,
    INT32S       img_h,
    INT32S       offset_x,
    INT32S       offset_y,
    INT16U       color) {
    // Calculate top-left reference point (centered + offset)
    INT32S start_x = (h_size - img_w) / 2 + offset_x;
    INT32S start_y = (v_size - img_h) / 2 + offset_y;

    // Calculate image center position
    INT32S center_x = img_w / 2;
    INT32S center_y = img_h / 2;
    // Color for the border region (blue)
    INT16U boader_color = sony_check_color(SONY_COLOR_BLUE);

    // Automatically determine center radius based on image size
    INT32S min_len = (img_w < img_h) ? img_w : img_h;
    INT32S center_len = (INT32S)(min_len * 0.15); // Center region side length is 15% of smaller dimension
    if (center_len % 2 == 0)
        center_len += 1; // Ensure the length is odd
    if (center_len < 5)
        center_len = 5; // Minimum size is 5x5

    INT8U center_radius = (center_len - 1) / 2; // Convert to radius

    INT32U x, y;

    for (y = 0; y < img_h; y++) {
        for (x = 0; x < img_w; x++) {
            if (image_array[y * img_w + x] == 1) {
                // Check if pixel is inside center region use the given color
                if (abs(x - center_x) <= center_radius && abs(y - center_y) <= center_radius) {
                    draw_pixel(src, start_x + x, start_y + y, color);
                } else {
                    // All other active pixels use the blue color
                    draw_pixel(src, start_x + x, start_y + y, boader_color);
                }
            }
        }
    }
}

/**
 * @brief   Draw a circle on the screen at a specified position.
 * @param   src         Pointer to the frame buffer.
 * @param   cx          X-coordinate of the pixel.
 * @param   cy          Y-coordinate of the pixel.
 * @param   radius      Radius of the pixel.
 * @param   thickness   Thickness of the border
 * @param   color       RGB565 color value for the pixel.
 * @note    Ensures the pixel is within the valid display boundaries.
 */
void draw_circle(INT16U* src, INT32S cx, INT32S cy, INT32S radius, INT32S thickness, INT16U color) {
    INT32S x, y;
    INT32S inner = (radius - thickness) * (radius - thickness);
    INT32S outer = (radius + thickness) * (radius + thickness);
    for (y = -radius - thickness; y <= radius + thickness; y++) {
        for (x = -radius - thickness; x <= radius + thickness; x++) {
            INT32S dist2 = x * x + y * y;
            if (dist2 >= inner && dist2 <= outer) {
                draw_pixel(src, cx + x, cy + y, color);
            }
        }
    }
}

/**
 * @brief   Draw a rectangle centered at (cx, cy) with the given width and height.
 *          The rectangle is filled with the specified RGB565 color.
 * @param   src     Pointer to the frame buffer.
 * @param   cx      X-coordinate of the rectangle center.
 * @param   cy      Y-coordinate of the rectangle center.
 * @param   width   Width of the rectangle.
 * @param   height  Height of the rectangle.
 * @param   color   RGB565 color value for each pixel in the rectangle.
 */
void draw_rectagngle(INT16U* src, INT32S cx, INT32S cy, INT32S width, INT32S height, INT16U color) {
    INT32S x = 0, y = 0;
    for (y = cy - height / 2; y < cy + height / 2; y++) {
        for (x = cx - width / 2; x < cx + width / 2; x++) {
            draw_pixel(src, x, y, color);
        }
    }
}

// Draw pattern on TFT panel
// Note: When drawing pattern the h_size need add 1
static BOOLEAN sony_show(sony_task_message_t msg) {
    // DBG_PRINT("[Show] (%d,%d) %d, %d, %d\r\n", msg.x, msg.y, msg.size, msg.pattern, msg.color);
    BOOLEAN ret = TRUE;
    INT16U  pattern_w, pattern_h;
    INT16U* src;
    INT16U  color;
    INT32U  disp_buf;
    INT32S  x_point;
    INT32S  y_point;
    INT32S  x, y; // for loop
    INT32S  pattern_index;

    color = sony_check_color(msg.color);
    if (msg.pattern >= SONY_PATTERN_MAX) { // check pattern is valid or not
        DBG_PRINT("Invalid pattern\r\n");
        ret = FALSE;
        return ret;
    }
    // Stop animation before drawing new pattern
    if (animation_state != ANIMATE_STATE_STOP) {
        sony_stop_animation();
    }
    pattern_w = msg.w_size;
    pattern_h = msg.h_size;
    x_point = msg.x + (h_size / 2);
    y_point = msg.y + (v_size / 2);
    // DBG_PRINT("abs x,y (%d,%d), size %dx%d\r\n", x_point, y_point, pattern_w, pattern_h);

    disp_buf = (INT32U)gp_malloc_align(((buf_size * DISP_NUMBER_MAX) + 64), 64);
    // Initial buffer
    gp_memset((INT8S*)disp_buf, 0, buf_size);

    src = (INT16U*)disp_buf;
    switch (msg.pattern) {
    case SONY_PATTERN_RECTANGLE:
        // Draw a rectangle
        draw_rectagngle(src, x_point, y_point, pattern_w, pattern_h, color);
        break;
    case SONY_PATTERN_BLINK:
        sony_init_base_buff(msg);
        break;
    case SONY_PATTERN_OPEN:
    case SONY_PATTERN_CLOSE:
    case SONY_PATTERN_OK:
        pattern_index = 0;
        for (y = y_point - pattern_h / 2; y < y_point + pattern_h / 2; y++) {
            for (x = x_point - pattern_w / 2; x < x_point + pattern_w / 2; x++) {
                if (msg.pattern == SONY_PATTERN_OPEN)
                    draw_pixel(src, x, y, display_open_eye[pattern_index]);
                else if (msg.pattern == SONY_PATTERN_CLOSE)
                    draw_pixel(src, x, y, display_close_eye[pattern_index]);
                else if (msg.pattern == SONY_PATTERN_OK)
                    draw_pixel(src, x, y, display_finish[pattern_index]);

                pattern_index = pattern_index + 1;
            }
        }
        break;
    case SONY_PATTERN_1_PS:
        draw_image(src, &fixation1_ps[0][0], ARRAY_WIDTH(fixation1_ps), ARRAY_HEIGHT(fixation1_ps), msg.x, msg.y, color);
        break;
    case SONY_PATTERN_1_AS:
        draw_image(src, &fixation1_as[0][0], ARRAY_WIDTH(fixation1_as), ARRAY_HEIGHT(fixation1_as), msg.x, msg.y, color);
        break;
    case SONY_PATTERN_2_PS:
        draw_image(src, &fixation2_ps[0][0], ARRAY_WIDTH(fixation2_ps), ARRAY_HEIGHT(fixation2_ps), msg.x, msg.y, color);
        break;
    case SONY_PATTERN_2_AS:
        draw_image(src, &fixation2_as[0][0], ARRAY_WIDTH(fixation2_as), ARRAY_HEIGHT(fixation2_as), msg.x, msg.y, color);
        break;
    default:
        // Should never reach here
        DBG_PRINT("Invalid pattern\r\n");
        ret = FALSE;
        break;
    }

    update_current_pattern(msg);
    if (ret && msg.pattern != SONY_PATTERN_BLINK) {
        drv_l2_display_update(DISPLAY_DEVICE, (INT32U)disp_buf);
    }
    gp_free((void*)disp_buf);
    return ret;
}

// Display W/R/G/B on full screen
static void sony_show_full_screen(sony_task_message_t msg) {
    INT16U* src;
    INT32U  i;
    INT32U  disp_buf;
    INT16U  color;
    if (animation_state != ANIMATE_STATE_STOP) {
        sony_stop_animation();
    }
    color = sony_check_color(msg.color);

    disp_buf = (INT32U)gp_malloc_align(((buf_size * DISP_NUMBER_MAX) + 64), 64);
    // Initial buffer
    gp_memset((INT8S*)disp_buf, 0, buf_size);

    src = (INT16U*)disp_buf;
    for (i = 0; i < buf_size / 2; i++)
        *src++ = color;
    drv_l2_display_update(DISPLAY_DEVICE, (INT32U)disp_buf);
    gp_free((void*)disp_buf);
    update_current_pattern(msg);
}

// initial the base display buffer
/*
A 14x9 rectangle in the middle.
+---------+---------+---------+
|                             |
+                             +
|              .              |
+                             +
|                             |
+---------+---------+---------+
*/
/**
 * @brief   Initialize the base display buffer and animation settings.
 * @param   msg: Pattern information including size, position, and color.
 * @note    Allocates memory for base and transition buffers and clears them.
 */
void sony_init_base_buff(sony_task_message_t msg) {
    INT32S  x, y;
    INT16U* src;
    INT16U  pattern_w, pattern_h;

    pattern_w = msg.w_size;
    pattern_h = msg.h_size;
    x_point = msg.x + (h_size / 2);
    y_point = msg.y + (v_size / 2);
    animation_color = sony_check_color(msg.color);

    // if (animation_state != ANIMATE_STATE_STOP) {
    //     sony_stop_animation();
    // }
    animation_distance = v_size / 2;
    base_disp_buf = (INT32U)gp_malloc_align(((buf_size * DISP_NUMBER_MAX) + 64), 64);
    transition_buf = (INT32U)gp_malloc_align(((buf_size * DISP_NUMBER_MAX) + 64), 64);
    // Initial buffer
    gp_memset((INT8S*)base_disp_buf, 0, buf_size);
    gp_memset((INT8S*)transition_buf, 0, buf_size);

    /* transition_buf is a full screen buffer*/
    src = (INT16U*)transition_buf;
    for (y = 0; y < v_size; y++) {
        for (x = 0; x < h_size; x++) {
            draw_pixel(src, x, y, animation_color);
        }
    }
    src = (INT16U*)base_disp_buf;
    x_start = x_point - pattern_w / 2;
    x_end = x_point + pattern_w / 2;
    y_start = y_point - pattern_h / 2;
    y_end = y_point + pattern_h / 2;
    // DBG_PRINT("x %d~%d, y %d~%d\r\n", x_start, x_end, y_start, y_end);
    // Draw a rectangle on base_disp_buff
    for (y = y_start; y < y_end; y++) {
        for (x = x_start; x < x_end; x++) {
            draw_pixel(src, x, y, animation_color);
        }
    }

    animation_state = ANIMATE_STATE_START;
}

static void sony_show_transition(INT8U d) {
    INT32S  x, y;
    INT32U  disp_buf;
    INT16U* src;
    // allocate buffer
    disp_buf = (INT32U)gp_malloc_align(((buf_size * DISP_NUMBER_MAX) + 64), 64);

    // Copy content of transition_buf to display buffer
    gp_memcpy((INT8S*)disp_buf, (INT8S*)transition_buf, buf_size);

    src = (INT16U*)disp_buf;

    // Calculate the range of pixels to be processed.
    INT32S x_s = x_point - d;
    INT32S y_s = y_point - d;
    INT32S x_e = x_point + d;
    INT32S y_e = y_point + d;

    for (y = 0; y < v_size; y++) {
        for (x = 0; x < h_size; x++) {
            // Check if the pixel is within the specified range
            if ((x < x_s || x > x_e) || (y < y_s || y > y_e)) {
                // Set the pixel to black
                draw_pixel(src, x, y, 0);
            }
        }
    }

    drv_l2_display_update(DISPLAY_DEVICE, (INT32U)disp_buf);
    gp_memcpy((INT8S*)transition_buf, (INT8S*)disp_buf, buf_size); // Copy current content to transition buffer
    gp_free((void*)disp_buf);
}

// Function to draw the animation in buffer
/**
 * @brief   Draw the animation in buffer
 * @param   d: the distance between outer rectangle and the middle cordinate
 * @return  none
 */
/*
+---------+---------+---------+
|       +-------------+       |
+       |   +-----+   |       +
|       |   |  .  |   |       |
+       |   +-----+   |       +
|       +-------------+       |
+---------+---------+---------+
*/

static void sony_draw_frame(INT8U d) {
    INT8U   i;
    INT32S  x, y;
    INT32U  disp_buf;
    INT16U* src;
    // allocate buffer
    disp_buf = (INT32U)gp_malloc_align(((buf_size * DISP_NUMBER_MAX) + 64), 64);

    // Copy content of base_disp_buf to display buffer
    gp_memcpy((INT8S*)disp_buf, (INT8S*)base_disp_buf, buf_size);

    src = (INT16U*)disp_buf;

    // Calculate the range of pixels to be processed.
    INT32S x_s = x_point - d;
    INT32S y_s = y_point - d;
    INT32S x_e = x_point + d;
    INT32S y_e = y_point + d; // be careful v_size

    // Draw line.
    for (i = 0; i < WIDTH; i++) {
        for (x = x_s; x < x_e; x++) {
            // horizontal line
            draw_pixel(src, x, (y_s + i), animation_color);
            draw_pixel(src, x, (y_e - i), animation_color);
        }
        for (y = y_s; y < y_e; y++) {
            // vertical line
            draw_pixel(src, (x_s + i), y, animation_color);
            draw_pixel(src, (x_e - i), y, animation_color);
        }
    }

    // Draw the rectangle again to bring it to the top layer.
    for (y = y_start; y < y_end; y++) {
        for (x = x_start; x < x_end; x++) {
            if (x >= 0 && x < h_size && y >= 0 && y < v_size) {
                draw_pixel(src, x, y, animation_color);
            }
        }
    }

    drv_l2_display_update(DISPLAY_DEVICE, (INT32U)disp_buf);
    gp_free((void*)disp_buf);
}

INT8U sony_change_transition_speed(float speed) {
    if (speed <= 0 || speed > 1)
        return 1;

    transition_step = TRANSITION_STEP * speed;

    return 0;
}

void sony_task_runner(void) {
    portTickType xLastWakeTimevBlink;
    portTickType last_tick;

    sony_task_message_t msg;

    queue_cmd_to_sony_task = xQueueCreate(10, sizeof(sony_task_message_t));
    DBG_PRINT("udisplay_task start \r\n");

    // Initialize display device
    drv_l2_display_init();

    drv_l2_display_start(DISPLAY_DEVICE, DISP_FMT_RGB565); // DISP_FMT_GP420, DISP_FMT_RGB565
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U*)&h_size, (INT16U*)&v_size);
    buf_size = (h_size * v_size * 2);
    m_tft_data.x = DEFAULT_X;
    m_tft_data.y = DEFAULT_Y;
    m_tft_data.pattern = DEFAULT_PATTERN;
    m_tft_data.w_size = DEFAULT_RECT_WIDTH;
    m_tft_data.h_size = DEFAULT_RECT_HEIGHT;
    m_tft_data.color = DEFAULT_COLOR;
    sony_show(m_tft_data);
    animation_distance = v_size / 2;
    xLastWakeTimevBlink = xTaskGetTickCount();
    last_tick = xLastWakeTimevBlink;
    while (1) {
        /* Wait for up to 10 milliseconds to receive a new command from the queue */
        if (queue_cmd_to_sony_task && (xQueueReceive(queue_cmd_to_sony_task, &msg, pdMS_TO_TICKS(10)) == pdTRUE)) {
            switch (msg.sony_cmd) {
            case SONY_TASK_CMD_ENABLE:
                tft_sony_ecx336_power_on();
                ack();
                break;
            case SONY_TASK_CMD_DISABLE:
                tft_sony_ecx336_power_off();
                ack();
                break;
            case SONY_TASK_CMD_FIXATION_MODE:
                break;
            case SONY_TASK_CMD_SHOW:
                if (sony_show(msg))
                    ack();
                else
                    nak();
                break;
            case SONY_TASK_CMD_DISC:
                break;
            case SONY_TASK_CMD_FULL:
                sony_show_full_screen(msg);
                ack();
                break;
            case SONY_TASK_CMD_CHANGE_SIZE: {
                sony_task_message_t tmp_msg = m_tft_data;
                tmp_msg.w_size = msg.w_size;
                tmp_msg.h_size = msg.h_size;
                if (sony_show(tmp_msg))
                    ack();
                else
                    nak();
                break;
            }
            case SONY_TASK_CMD_CHANGE_POSITION: {
                sony_task_message_t tmp_msg = m_tft_data;
                tmp_msg.x = msg.x;
                tmp_msg.y = msg.y;
                if (sony_show(tmp_msg))
                    ack();
                else
                    nak();
                break;
            }
            case SONY_TASK_CMD_GET_INFO:
                DBG_PRINT("%dx%d at (%d,%d).\r\n", m_tft_data.w_size, m_tft_data.h_size, m_tft_data.x, m_tft_data.y);
                uart2_printf("%dx%d at (%d,%d).\r\n", m_tft_data.w_size, m_tft_data.h_size, m_tft_data.x, m_tft_data.y);
                break;
            case SONY_TASK_CMD_CHANGE_SPEED:
                if (sony_change_transition_speed(msg.speed)) {
                    DBG_PRINT("Invalid speed\r\n");
                    uart2_printf("Invalid speed\r\n");
                } else {
                    DBG_PRINT("%.1fx\r\n", msg.speed);
                    ack();
                }
                break;
            case SONY_TASK_CMD_CHANGE_BRIGHTNESS:
                if (sony_set_brightness(msg.brightness)) {
                    if (m_tft_data.pattern < SONY_PATTERN_FULL) {
                        if (sony_show(m_tft_data))
                            ack();
                        else
                            nak();
                    } else {
                        sony_show_full_screen(m_tft_data);
                        ack();
                    }
                } else {
                    nak();
                }
                break;
            default:
                break;
            }
        }
        // Animation logic
        if (animation_state != ANIMATE_STATE_STOP) {
            portTickType current_tick = xTaskGetTickCount();
            // Somehow the vTaskDelayUntil() doesn't work as expected
            if ((current_tick - last_tick) >= ACTUAL_TICK_DELAY_COUNT) {
                last_tick = current_tick;
                switch (animation_state) {
                case ANIMATE_STATE_START:
                    sony_show_transition(animation_distance);
                    if (animation_distance > 0)
                        animation_distance -= transition_step;
                    else {
                        animation_distance = v_size / 2;
                        animation_state = ANIMATE_STATE_PLAY;
                        sony_stop_transition();
                    }
                    break;
                case ANIMATE_STATE_PLAY:
                    if (b_show_animation) // Play the animation every other time
                        sony_draw_frame(animation_distance);
                    if (animation_distance > 0)
                        animation_distance -= DISTANCE_STEP;
                    else {
                        animation_distance = v_size / 2;
                        b_show_animation = !b_show_animation;
                    }
                    break;
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTimevBlink, pdMS_TO_TICKS(MICRO_DISPLAY_BLINK_TASK_CYCLE));
    }
}
