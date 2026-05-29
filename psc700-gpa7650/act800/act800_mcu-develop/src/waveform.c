#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mems.h"

// Defines a -90 degree (or -PI/2) angle offset to start the radial scan from the negative Y-axis.
#define RADIAL_START_OFFSET    (-0.5 * M_PI)

const float SAMPLING_INTERVAL_US = 12.5f; // Sampling interval is fixed at 12.5 us due to the timer callback interval set by TCC1.

/* triangle function
 *   ^         /\              /\              /
 *   |        /  \            /  \            / 
 *   |       /    \          /    \          /  
 *   |      /      \        /      \        /   
 *interval /        \      /        \      /    ...(x edges)
 *   |    /          \    /          \    /     
 *   |   /            \  /            \  /      
 *   v  /              \/              \/       
 *      <-ramp-> <-ramp-><-ramp-> <-ramp-><-ramp->
 *      <----------------------- 5 edges ----------------------->
 */
void triangle(int edges, int ramp, int interval[2], uint16_t* wave, int repeat) {
    //    printf("edge %d, ramp %d, [%d,%d] \r\n", edges, ramp, interval[0], interval[1]);

    if (repeat == 0) {
        repeat = 1;
    }
    int index = 0;
    double range = (double) (interval[1] - interval[0]);
    for (int r = 0; r < repeat; r++) {
        for (int wave_index = 0; wave_index < edges; wave_index++) {
            if (wave_index % 2 == 0) {
                // Rising Edge
                for (int i = 0; i < ramp; i++) {
                    wave[index++] = (uint16_t) (interval[0] + (range / (ramp - 1)) * i);
                }
            } else {
                // Falling Edge
                for (int i = 0; i < ramp; i++) {
                    wave[index++] = (uint16_t) (interval[1] - (range / (ramp - 1)) * i);
                }
            }
        }
    }
}

/*
 * staircase function
 *                                     __________
 *                                   _|<-  tread  ->    ^
 *                         (x levels)                |
 *                       _    ...                interval
 *            __________|                            |
 * __________|<-  tread  ->                             v
 * <-  tread  ->
 */
void staircase(int levels, int tread, int interval[2], uint16_t* wave) {
    //    printf("levels %d, tread %d, [%d,%d] \r\n", levels, tread, interval[0], interval[1]);
    double level_height;
    int index = 0;

    // Calculate the step size for each level
    double step = (interval[1] - interval[0]) / (levels - 1);

    for (int i = 0; i < levels; i++) {
        // Calculate the height of the current level
        level_height = interval[0] + i * step;

        // Repeat the current level height 'tread' times in the wave array
        for (int j = 0; j < tread; j++) {
            wave[index++] = level_height;
        }
    }
}

/**
 * @brief Generates a cosine waveform between two voltage values.
 * 
 * This function calculates a cosine waveform between the start and end voltages,
 * mapping the result into a specified interval. The generated waveform is stored
 * in the provided array.
 * 
 * @param startv: The starting voltage value.
 * @param endv: The ending voltage value.
 * @param interval: Array containing the minimum and maximum values for the wave output.
 * @param wave: Pointer to the array where the generated waveform will be stored.
 * @param length: Pointer to an integer where the number of generated points will be stored.
 * @return None
 */
void cosine_wave(double startv, double endv, int interval[2], uint16_t* wave, int* length) {

    double range = (double) (interval[1] - interval[0]);
    // Calculate the absolute voltage difference between start and end values.
    double diff_v = fabs(endv - startv);
    // If the voltage difference is zero, return early as there's no wave to generate.
    if (diff_v == 0)
        return;

    // Duration is set to the voltage difference in milliseconds. For example, a 100V difference maps to a 100ms duration.
    float duration_ms = diff_v;
    float sampling_interval_s = SAMPLING_INTERVAL_US * 1e-6; // Convert sampling interval to seconds.
    float duration_s = duration_ms * 1e-3; // Convert duration to seconds.

    // Calculate the number of points based on the duration and sampling interval.
    int num_points = (int) (duration_s / sampling_interval_s);
    *length = num_points;

    // Generate the cosine wave over the computed number of points.
    for (int i = 0; i < num_points; i++) {
        // Calculate normalized time value between 0 and 1.
        float t = (float) i / (num_points - 1);
        // Compute the cosine value, scaled between -1 and 1, and shift it to [0, 1].
        float cos_value = cos(M_PI * (1 - t));
        // Map the cosine value to the specified output range and store in the wave array.
        wave[i] = (uint16_t) ((cos_value + 1.0) * 0.5 * range + interval[0]);
    }
}

/**
 * @brief Generates an array of trigger times for scan operations.
 * 
 * The generated trigger times are stored in the provided result array.
 * The function supports both non-averaging and averaging modes. In averaging mode, 
 * odd averaging skips every (avgcnt + 1)-th trigger, while even averaging triggers 
 * at regular intervals.
 * 
 * @param avgcnt: Averaging count. If zero, non-averaging mode is used.
 * @param linecnt: Number of lines to scan.
 * @param trigger_start: First trigger.
 * @param period: Time interval between triggers.
 * @param result_array: Pointer to the array where the generated trigger times will be stored.
 * @param result_size: Pointer to an integer where the number of generated trigger points will be stored.
 * @return None
 */
void trigger_array(int avgcnt, int linecnt, int trigger_start, int period, uint32_t* result_array, int* result_size) {
    int i_range;

    if (avgcnt == 0) {
        i_range = linecnt; // Non-averaging mode
    } else if (avgcnt & 1) {
        i_range = (avgcnt + 1) * linecnt; // Odd averaging
    } else {
        i_range = avgcnt * linecnt; // Even averaging
    }

    int index = 0;
    for (int i = 0; i < i_range; i++) {
        // Skip every (avgcnt + 1)-th iteration for odd averaging
        if (avgcnt != 0 && (avgcnt & 1) && (i + 1) % (avgcnt + 1) == 0) {
            continue;
        }

        result_array[index] = trigger_start + period * i;
        index++;

        // Avoid overflow
        if (index >= MAX_LINESCAN_SIZE) {
            printf("Error: Exceeded maximum result array size\n");
            break;
        }
    }
    // update trigger_length
    *result_size = index;
}

/**
 * @brief Generates a waveform for X-axis movement, consisting of a horizontal line
 *        (triangle waves) followed by vertical lines (staircase wave).
 *
 * This function is designed to generate a waveform pattern for X-axis scanning. The first
 * section is a horizontal line generated as triangle waves, and the second section
 * consists of vertical lines generated as a staircase wave.
 *
 * @param edges      Number of edges to generate in the triangle wave.
 * @param ramp       Number of points for each edge transition.
 * @param linecnt    Number of vertical lines in the staircase wave.
 * @param interval   Interval range for the wave [min, max].
 * @param wave       Pointer to the output array where the generated waveform will be stored.
 */
void gcc_x_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave) {
    int horizontal_length = edges * ramp;
    int vertical_length = edges * linecnt * ramp;

    uint16_t horizontal_section[horizontal_length];
    uint16_t vertical_section[vertical_length];

    // Generate a triangle wave for the horizontal section with the given interval
    triangle(edges, ramp, interval, horizontal_section, 0);
    // Generate a staircase waveform for the vertical section based on the interval
    staircase(linecnt, edges*ramp, interval, vertical_section);

    // Copy the horizontal section data into the wave array
    for (int i = 0; i < horizontal_length; i++) {
        wave[i] = horizontal_section[i];
    }
    // Copy the vertical section data into the wave array
    for (int i = 0; i < vertical_length; i++) {
        wave[horizontal_length + i] = vertical_section[i];
    }
}

/**
 * @brief Generates a waveform for Y-axis movement, consisting of a horizontal line
 *        (triangle wave) followed by vertical lines (triangle wave).
 *
 * This function is designed to generate a waveform pattern for Y-axis scanning. The first
 * section is a horizontal line generated as a triangle wave at half the interval range, and the second section
 * consists of vertical lines also generated as a triangle wave. The number
 * of edges represents the average number of repetitions for each line in the pattern.
 *
 * @param edges      Number of edges to generate in the triangle wave.
 * @param ramp       Number of points for each edge transition.
 * @param linecnt    Number of vertical lines in the triangle wave.
 * @param interval   Interval range for the wave [min, max].
 * @param wave       Pointer to the output array where the generated waveform will be stored.
 */
void gcc_y_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave) {
    int horizontal_length = edges * ramp; //4620
    int vertical_length = edges * linecnt * ramp;

    uint16_t horizontal_section[horizontal_length];
    uint16_t vertical_section[vertical_length];

    // Calculate the midpoint value of the interval, this will be the constant Y value for the horizontal section
    float mid_value = 0.5 * (interval[1] - interval[0]) + interval[0];
    int horizontal_section_interval[2] = {mid_value, mid_value};
    // Generate a triangle wave for the horizontal section with a fixed Y value
    triangle(edges, ramp, horizontal_section_interval, horizontal_section, 0);
    // Generate a triangle wave for the vertical section with a variable Y value based on the interval
    triangle(edges* linecnt, ramp, interval, vertical_section, 0);
    // Copy the horizontal section data into the wave array
    for (int i = 0; i < horizontal_length; i++) {
        wave[i] = horizontal_section[i];
    }
    // Copy the vertical section data into the wave array after the horizontal section
    for (int i = 0; i < vertical_length; i++) {
        wave[horizontal_length + i] = vertical_section[i]; // Copy section2 to wave after section1
    }

    // Replace the last 'ramp' points of the horizontal section with values linearly decreasing from mid_value to 0
    for (int i = horizontal_length - ramp; i < horizontal_length; i++) {
        // Linearly interpolate values from mid_value to 0
        float ratio = (float) (i - (horizontal_length - ramp)) / (ramp - 1); // Calculate the interpolation ratio
        wave[i] = (uint16_t) (mid_value * (1.0 - ratio)); // Apply linear interpolation
    }
}

/**
 * @brief Generates a radial wave along the X-axis based on cosine values.
 * 
 * This function calculates X-axis radial waveforms using cosine values. The number 
 * of edges and the ramp specify how each waveform section is constructed. For each 
 * line, it computes a minimum and maximum value based on the cosine of the angle 
 * and uses these values to create a triangle wave. The resulting waveform is stored 
 * in the provided wave array.
 * 
 * @param edges      Number of edges to generate in the triangle wave.
 * @param ramp       Number of points for each edge transition.
 * @param linecnt    Number of lines in the radial wave, which determines the angular step between each line.
 * @param interval   Interval range for the wave [min, max].
 * @param wave       Pointer to the output array where the generated waveform will be stored.
 */
void radial_x_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave) {
    int wave_index = 0;
    int radial_length = edges * ramp;
    uint16_t radial_section[radial_length]; // Create an array to store each radial line waveform

    float cosine[64]; // Cosine values for each radial line x coordinate

    // 'angle' is the angular step size based on the total number of lines (linecnt)
    float angle = M_PI / linecnt;
    for (int i = 0; i < linecnt; i++) {
        cosine[i] = cos(angle * i + RADIAL_START_OFFSET);
    }
    // Iterate through each line
    for (int i = 0; i < linecnt; i++) {
        float min_val = (cosine[i] + 1) / 2 * (interval[1] - interval[0]) + interval[0];
        float max_val = (-cosine[i] + 1) / 2 * (interval[1] - interval[0]) + interval[0];
        // Define the new interval for the current radial section
        int radial_interval[2] = {min_val, max_val};
        // Copy the generated radial section into the main wave array
        triangle(edges, ramp, radial_interval, radial_section, 0);
        // Copy the generated radial section into the main wave array
        for (int i = 0; i < radial_length; i++) {
            wave[wave_index++] = (uint16_t) radial_section[i];
        }
    }
}

/**
 * @brief Generates a radial wave along the Y-axis based on sine values.
 * 
 * This function generates Y-axis radial waveforms using sine values. Similar to the 
 * X-axis function, it calculates the minimum and maximum values for each radial line 
 * based on the sine of the angle. These values are used to create triangle waves, 
 * which are then stored in the provided wave array. The function handles the entire 
 * process for the given number of lines (linecnt).
 * 
 * @param edges      Number of edges to generate in the triangle wave.
 * @param ramp       Number of points for each edge transition.
 * @param linecnt    Number of lines in the radial wave, which determines the angular step between each line.
 * @param interval   Interval range for the wave [min, max].
 * @param wave       Pointer to the output array where the generated waveform will be stored.
 */
void radial_y_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave) {
    int wave_index = 0;
    int radial_length = edges * ramp;
    uint16_t radial_section[radial_length]; // Create an array to store each radial line waveform

    float sine[64]; // Sine values for each radial line y coordinate

    // 'angle' is the angular step size based on the total number of lines (linecnt)
    float angle = M_PI / linecnt;
    for (int i = 0; i < linecnt; i++) {
        sine[i] = sin(angle * i + RADIAL_START_OFFSET);
    }
    // Iterate through each line
    for (int i = 0; i < linecnt; i++) {
        float min_val = (sine[i] + 1) / 2 * (interval[1] - interval[0]) + interval[0];
        float max_val = (-sine[i] + 1) / 2 * (interval[1] - interval[0]) + interval[0];
        // Define the new interval for the current radial section
        int radial_interval[2] = {min_val, max_val};
        // Copy the generated radial section into the main wave array
        triangle(edges, ramp, radial_interval, radial_section, 0);
        // Copy the generated radial section into the main wave array
        for (int i = 0; i < radial_length; i++) {
            wave[wave_index++] = (uint16_t) radial_section[i];
        }
    }
}