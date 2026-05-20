#ifndef __AF_ALG_H__
#define __AF_ALG_H__

#include "typedef.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define ROI_SHRINK        (30)
#define ROI_OFFSET_X      (120)
#define AF_ROI_STEP_SIZE  (8)

/**
  * @brief Auto focus sharpness algorithm
  * @param frm    Pointer to the collected frame buffer (YUY2 format). 
  * @param  width Width (columns) of the frame.
  * @param  height Height (rows) of the frame.
  * @retval Normalized sharpness ratio (sum of absolute gradients / background intensity)
  */
float af_task_calc_sharpness(INT8U * frm, INT32U width, INT32U height);

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
float af_task_find_best_sub_index_by_correlation(float *sharpness_values, INT32U num);

#endif //__AF_ALG_H__
