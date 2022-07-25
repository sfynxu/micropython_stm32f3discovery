#define MICROPY_HW_BOARD_NAME       "F3DISC"
#define MICROPY_HW_MCU_NAME         "STM32F303"

#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_ENABLE_SERVO     (0)

// HSE is 8MHz
//#define MICROPY_HW_CLK_PLLM (8)
//#define MICROPY_HW_CLK_PLLN (336)
//#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
//#define MICROPY_HW_CLK_PLLQ (7)

// For system clock, enable one source:
#define MICROPY_HW_CLK_USE_HSI      (1) // internal 8MHz -> PLL = 48MHz.
#define MICROPY_HW_CLK_USE_HSI48    (1) // internal 48MHz.
// #define MICROPY_HW_CLK_USE_HSE      (1) // external crystal -> PLL = 48MHz.
// For HSE set the crystal / clock input frequency HSE_VALUE in stm32f0xx_hal_conf.h
//#if MICROPY_HW_CLK_USE_HSE
//#define MICROPY_HW_CLK_USE_BYPASS   (1) // HSE comes from STLINK 8MHz, not crystal.
//#endif

// The board has an external 32kHz crystal
#define MICROPY_HW_RTC_USE_LSE      (1)


// UART config
#if 0
// A9 is used for USB VBUS detect, and A10 is used for USB_FS_ID.
// UART1 is also on PB6/7 but PB6 is tied to the Audio SCL line.
// Without board modifications, this makes UART1 unusable on this board.
#define MICROPY_HW_UART1_TX     (pin_A9)
#define MICROPY_HW_UART1_RX     (pin_A10)
#endif
#define MICROPY_HW_UART2_TX     (pin_A2)
#define MICROPY_HW_UART2_RX     (pin_A3)
#define MICROPY_HW_UART2_RTS    (pin_A1)
#define MICROPY_HW_UART2_CTS    (pin_A0)
#define MICROPY_HW_UART3_TX     (pin_D8)
#define MICROPY_HW_UART3_RX     (pin_D9)
#define MICROPY_HW_UART3_RTS    (pin_D12)
#define MICROPY_HW_UART3_CTS    (pin_D11)
#if MICROPY_HW_HAS_SWITCH == 0
// NOTE: A0 also connects to the user switch. To use UART4 you should
//       set MICROPY_HW_HAS_SWITCH to 0, and also remove SB20 (on the back
//       of the board near the USER switch).
#define MICROPY_HW_UART4_TX     (pin_A0)
#define MICROPY_HW_UART4_RX     (pin_A1)
#endif
// NOTE: PC7 is connected to MCLK on the Audio chip. This is an input signal
//       so I think as long as you're not using the audio chip then it should
//       be fine to use as a UART pin.
//#define MICROPY_HW_UART6_TX     (pin_C6)
//#define MICROPY_HW_UART6_RX     (pin_C7)

// I2C buses
#define MICROPY_HW_I2C1_SCL (pin_B6)
#define MICROPY_HW_I2C1_SDA (pin_B7)
//#define MICROPY_HW_I2C2_SCL (pin_B10)
//#define MICROPY_HW_I2C2_SDA (pin_B11)
#define MICROPY_HW_I2C2_SCL (pin_A9)
#define MICROPY_HW_I2C2_SDA (pin_A10)

// SPI buses
#define MICROPY_HW_SPI1_NSS  (pin_A4)
#define MICROPY_HW_SPI1_SCK  (pin_A5)
#define MICROPY_HW_SPI1_MISO (pin_A6)
#define MICROPY_HW_SPI1_MOSI (pin_A7)
#define MICROPY_HW_SPI2_NSS  (pin_B12)
#define MICROPY_HW_SPI2_SCK  (pin_B13)
#define MICROPY_HW_SPI2_MISO (pin_B14)
#define MICROPY_HW_SPI2_MOSI (pin_B15)

// CAN buses
#define MICROPY_HW_CAN1_TX (pin_B9)
#define MICROPY_HW_CAN1_RX (pin_B8)
//#define MICROPY_HW_CAN2_TX (pin_B13)
//#define MICROPY_HW_CAN2_RX (pin_B12)

// USRSW is pulled low. Pressing the button makes the input go high.
#define MICROPY_HW_USRSW_PIN        (pin_A0)
#define MICROPY_HW_USRSW_PULL       (GPIO_NOPULL)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_RISING)
#define MICROPY_HW_USRSW_PRESSED    (1)

// LEDs
//#define MICROPY_HW_LED1             (pin_D14) // red
#define MICROPY_HW_LED2             (pin_D12) // green
//#define MICROPY_HW_LED3             (pin_D13) // orange
//#define MICROPY_HW_LED4             (pin_D15) // blue
#define MICROPY_HW_LED3             (pin_E9)    // red1
#define MICROPY_HW_LED4             (pin_E8)    // blue1
#define MICROPY_HW_LED5             (pin_E10)   // orange1
#define MICROPY_HW_LED6             (pin_E15)   // green1
#define MICROPY_HW_LED7             (pin_E11)   // green2
#define MICROPY_HW_LED8             (pin_E14)   // orange2
#define MICROPY_HW_LED9             (pin_E12)   // blue2
#define MICROPY_HW_LED10            (pin_E13)   // red2
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_low(pin))

// USB config
#define MICROPY_HW_USB_FS              (0)
//#define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_A9)
//#define MICROPY_HW_USB_OTG_ID_PIN      (pin_A10)
