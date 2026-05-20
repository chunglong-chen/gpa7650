 /** @file af_alg.c
 *  @brief Autofocus algorithm source file
 *
 *  This file is responsible for calculating the focus value of a single frame.
 */
 #include "af_alg.h"
 #define NOISE_THRESHOLD 5
/**
 * @brief  Compute a normalized sharpness metric over two separated semi-circular ROIs
 *         (left and right halves of the fundus field) on a YUY2 frame, using a
 *         lightweight Sobel-like gradient (1×3 horizontal and 3×1 vertical).
 *
 * ROI geometry:
 *   - Centered at (width/2, height/2).
 *   - Base radius r_base = (height/2 − 10), then reduced by roi_shrink (=30) to avoid the dark rim.
 *   - Vertically truncated by roi_vratio (=0.6): only rows within cy ± (R * roi_vratio) are processed.
 *   - The middle strip is excluded by roi_offset_x (=120), ensuring the left and right ROIs do not touch:
 *       Left segment : [cx − dx_max, cx − roi_offset_x]
 *       Right segment: [cx + roi_offset_x, cx + dx_max]
 *
 * Sharpness metric:
 *   Sum of |Gx| + |Gy| over the ROI, normalized by the sum of Y-luma values (sum_bg):
 *       sharpness = sum / sum_bg
 *   This normalization compensates for overall brightness differences.
 *
 * Notes:
 *   - YUY2 layout is assumed (Y at even byte indices per pixel column).
 *   - Horizontal gradient Gx: [-1, 0, 1] using (x−1, x+1) columns in the same row.
 *   - Vertical   gradient Gy: [-1, 0, 1] using (y−1, y+1) rows at the same column.
 *   - Integer-only circular boundary: dx_max is approximated without sqrt() for efficiency.
 */
float af_task_calc_sharpness(INT8U * frm, INT32U width, INT32U height)
{
    // Bytes per row in YUY2 format
    INT32U stride = width * 2;

    const INT32S roi_shrink   = ROI_SHRINK;       // inward shrink to avoid dark rim
    const float  roi_vratio   = 0.6f;     // vertical height ratio (0..1)
    const INT32S roi_offset_x = ROI_OFFSET_X;      // half-gap from center; defines middle separation


    // ROI geometry configuration center and radius
    const INT32S cx = (INT32S)width  / 2;
    const INT32S cy = (INT32S)height / 2;

    // Estimate base radius from image height, keeping a safety margin
    const INT32S margin = 10;
    const INT32S r_base = (INT32S)((height / 2) - margin);
    const INT32S R = (r_base - roi_shrink > 1) ? (r_base - roi_shrink) : 1;
    const INT64S R2 = (INT64S)R * (INT64S)R;

    // Vertical truncation: only process rows in [cy - dy_max, cy + dy_max]
    const INT32S dy_max = (INT32S)((float)R * roi_vratio);
    const INT32S y_top    = (cy - dy_max);
    const INT32S y_bottom = (cy + dy_max);

    INT64S sum = 0;
    INT64S sum_bg = 0;

    // Debug: Check raw pixel values at frame center to verify image data integrity
    // console_task_tx_send("%d,%d,%d\r\n", *(frm + 240 * stride + 319*2)
    //                                    , *(frm + 240 * stride + 320*2)
    //                                    , *(frm + 240 * stride + 321*2));
    for (INT32S y = y_top + 1; y <= y_bottom - 1; y++) {
        if (y < 1 || y > (INT32S)height - 2) continue;

        INT8U *row_above = frm + (y - 1) * stride;
        INT8U *row_curr  = frm +  y      * stride;
        INT8U *row_below = frm + (y + 1) * stride;

        // Compute horizontal extent within the circle: R^2 >= dx^2 + dy^2 ⇒ dx_max
        const INT32S dy = y - cy;
        const INT64S dy2 = (INT64S)dy * (INT64S)dy;
        if (dy2 > R2) continue;

        // Integer-only floor-sqrt approximation for dx_max
        INT32S dx_max;
        {
            INT64S remain = R2 - dy2;
            if (remain <= 0) continue;
            INT64S x_est = 0;
            while ((x_est * x_est) < remain) x_est += AF_ROI_STEP_SIZE; // step 8 for speed
            dx_max = (INT32S)(x_est - AF_ROI_STEP_SIZE);
            if (dx_max < 0) continue;
        }

        // Two disjoint segments (skip the middle gap of roi_offset_x around cx)
        // Left segment ：[cx - dx_max,      cx - roi_offset_x]
        // Right segment：[cx + roi_offset_x,    cx + dx_max]
        INT32S xL0 = cx - dx_max;
        INT32S xL1 = cx - roi_offset_x;
        INT32S xR0 = cx + roi_offset_x;
        INT32S xR1 = cx + dx_max;

        // Keep one-pixel margin horizontally for Gx (x-1 and x+1 must be valid)
        if (xL0 < 1) xL0 = 1;
        if (xL1 > (INT32S)width - 2) xL1 = (INT32S)width - 2;
        if (xR0 < 1) xR0 = 1;
        if (xR1 > (INT32S)width - 2) xR1 = (INT32S)width - 2;

        // Left segment
        if (xL0 <= xL1) {
            for (INT32S x = xL0; x <= xL1; x++) {
                const INT32U offset = (INT32U)x * 2; // YUY2: Y at even bytes
                INT32S gx = (INT32S)row_curr[offset + 2] - (INT32S)row_curr[offset - 2];
                INT32S gy = (INT32S)row_below[offset]    - (INT32S)row_above[offset];
                INT32S d = (gx < 0) ? -gx : gx;
                if (d >= NOISE_THRESHOLD) sum += d;
                d = (gy < 0) ? -gy : gy;
                if (d >= NOISE_THRESHOLD) sum += d;
                sum_bg += row_curr[offset];
            }
        }

        // Right segment
        if (xR0 <= xR1) {
            for (INT32S x = xR0; x <= xR1; x++) {
                const INT32U offset = (INT32U)x * 2;
                INT32S gx = (INT32S)row_curr[offset + 2] - (INT32S)row_curr[offset - 2];
                INT32S gy = (INT32S)row_below[offset]    - (INT32S)row_above[offset];
                INT32S d = (gx < 0) ? -gx : gx;
                if (d >= NOISE_THRESHOLD) sum += d;
                d = (gy < 0) ? -gy : gy;
                if (d >= NOISE_THRESHOLD) sum += d;
                sum_bg += row_curr[offset];
            }
        }
    }

    if (sum_bg == 0) return 0.0f;
    return (float)((double)sum / (double)sum_bg);
}

/**
 * @brief Finds the optimal sub-index using high-density correlation scanning.
 * * This function identifies the best focus position by comparing measured sharpness
 * values against a synthetic triangular model. By using a 0.5 step increment, 
 * it achieves "sub-index" resolution, allowing the peak to be located between 
 * two physical frames (effectively doubling the motor positioning precision).
 *
 * @param sharpness_values [in] Pointer to the array of sharpness scores (30 samples).
 * @param num              [in] Total number of samples (fixed at 30).
 * @return float           The optimal sub-index (e.g., 14.5) representing the focus peak.
 */
float af_task_find_best_sub_index_by_correlation(float *sharpness_values, INT32U num) {
    if (num < 2) return 0.0f;

    float y_start = sharpness_values[0];
    float y_end   = sharpness_values[num - 1];
    float y_max   = sharpness_values[0];

    // 1. Identify the global maximum sharpness to define the reference model's height
    for (INT32U i = 1; i < num; i++) {
        if (sharpness_values[i] > y_max) {
            y_max = sharpness_values[i];
        }
    }

    float max_corr = -1.1f;    // Initial value lower than any possible correlation
    float best_sub_idx = 0.0f;
    float ref_shape[30];

    /** * 2. High-density scan: 
     * A step of 0.5 represents a 1-step motor movement resolution (since sampling was every 2 steps).
     * k iterates through 0.0, 0.5, 1.0, 1.5 ... up to 29.0
     */
    for (float k = 0.0f; k <= (float)(num - 1); k += 0.5f) {
        
        /* --- Generate Ideal Reference Shape (Peak position 'k' can be a float) --- */
        for (INT32U i = 0; i < num; i++) {
            float fi = (float)i;
            if (fi <= k) {
                // Upslope: Handle k near 0 to prevent division by zero
                ref_shape[i] = (k < 0.001f) ? y_max : y_start + (y_max - y_start) * fi / k;
            } else {
                // Downslope: Handle k near the last index to prevent division by zero
                ref_shape[i] = (k > (float)(num - 1) - 0.001f) ? y_max : 
                               y_max + (y_end - y_max) * (fi - k) / ((float)(num - 1) - k);
            }
        }

        /* --- Calculate Pearson Correlation Coefficient --- */
        float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0, sum_y2 = 0;
        for (INT32U i = 0; i < num; i++) {
            float x = sharpness_values[i];
            float y = ref_shape[i];
            sum_x  += x;
            sum_y  += y;
            sum_xy += x * y;
            sum_x2 += x * x;
            sum_y2 += y * y;
        }

        float n = (float)num;
        float numerator = (n * sum_xy) - (sum_x * sum_y);
        float denominator = sqrtf(((n * sum_x2) - (sum_x * sum_x)) * ((n * sum_y2) - (sum_y * sum_y)));
        
        /**
         * Assign -1.0f if denominator is zero (occurs with flat signals, 
         * common in low-contrast IR scenarios).
         */
        float corr = (denominator != 0) ? (numerator / denominator) : -1.0f;
        // printf("[AF_ALG][LOG] [%.1f] %f \r\n", k, corr);

        // Update the best sub-index if a higher correlation is found
        if (corr > max_corr) {
            max_corr = corr;
            best_sub_idx = k;
        }
    }
    // printf("[AF_ALG][LOG] Max corr %f \r\n", max_corr);

    return best_sub_idx;
}