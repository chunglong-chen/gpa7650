/**
 * @file probe_board_groups.h
 * @brief Doxygen group definitions for Probe Board firmware modules
 */


/**
 * @defgroup uart_transceiver UART Transceiver
 * @ingroup probe_board_firmware
 * @brief UART communication module.
 *
 * This module transmits and receives data via UART
 * with the Driver APP.
 *
 * @{
 */
/** @} */

/**
 * @defgroup command_parser Command Parser
 * @ingroup probe_board_firmware
 * @brief Command parsing and dispatch module.
 *
 * This module parses incoming commands and invokes
 * the corresponding control or processing functions.
 *
 * @{
 */
/** @} */

/**
 * @defgroup liquid_lens_controller Liquid Lens Controller
 * @ingroup probe_board_firmware
 * @brief Liquid lens control module.
 *
 * This module controls the liquid lens, including
 * mode switching, optical power control, voltage control,
 * and status query.
 *
 * @{
 */
/** @} */

/**
 * @defgroup pupil_led_controller Pupil LED Controller
 * @ingroup probe_board_firmware
 * @brief Pupil LED control module.
 *
 * This module controls the pupil illumination LED
 * through the I2C interface, including brightness
 * setting and status query.
 *
 * @{
 */
/** @} */

/**
 * @defgroup asps_switcher AS/PS Switcher
 * @ingroup probe_board_firmware
 * @brief AS/PS mode switching control module.
 *
 * This module switches between AS and PS modes
 * through motor and GPIO control, and monitors
 * the switching status.
 *
 * @{
 */
/** @} */

/**
 * @defgroup fixation_display_controller Fixation Display Controller
 * @ingroup probe_board_firmware
 * @brief Fixation display control module.
 *
 * This module controls the fixation display,
 * including display power control, pattern update,
 * animation control, and full-screen color display.
 *
 * @{
 */
/** @} */

/**
 * @defgroup three_axis_controller 3-Axis Controller
 * @ingroup probe_board_firmware
 * @brief 3-axis motor control module.
 *
 * This module controls the 3-axis mechanism,
 * including homing, absolute and relative movement,
 * position query, and zero-point setting.
 *
 * @{
 */
/** @} */