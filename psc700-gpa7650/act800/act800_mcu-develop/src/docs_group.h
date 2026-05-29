/**
 * @file main_board_groups.h
 * @brief Doxygen group definitions for Main Board firmware modules
 */


/**
 * @defgroup uart_transceiver UART Transceiver
 * @ingroup main_board_firmware
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
 * @ingroup main_board_firmware
 * @brief Command parsing and dispatch module.
 *
 * This module parses incoming commands and invokes
 * the corresponding control or processing functions.
 *
 * @{
 */
/** @} */

/**
 * @defgroup indicator_led_controller Indicator LED Controller
 * @ingroup main_board_firmware
 * @brief Indicator LED control module.
 *
 * This module controls the Indicator LED through
 * the I2C interface.
 *
 * @{
 */
/** @} */

/**
 * @defgroup motor_controller Motor Controller
 * @ingroup main_board_firmware
 * @brief Motor control module.
 *
 * This module controls the motors of the polarization
 * controller and reference arm via GPIO, including
 * motor actuation and status handling.
 *
 * @{
 */
/** @} */

/**
 * @defgroup sld_controller SLD Controller
 * @ingroup main_board_firmware
 * @brief SLD power and monitoring control module.
 *
 * This module controls the power of the SLD module
 * through GPIO and monitors its operating status
 * via ADC.
 *
 * @{
 */
/** @} */

/**
 * @defgroup spectrometer_mems_controller Spectrometer and MEMS Controller
 * @ingroup main_board_firmware
 * @brief Spectrometer trigger and MEMS drive control module.
 *
 * This module controls the spectrometer trigger
 * through GPIO and drives the MEMS through DAC.
 *
 * @{
 */
/** @} */

/**
 * @defgroup data_storage Data Storage
 * @ingroup main_board_firmware
 * @brief Flash data storage module.
 *
 * This module stores MEMS calibration data
 * in Flash memory.
 *
 * @{
 */
/** @} */