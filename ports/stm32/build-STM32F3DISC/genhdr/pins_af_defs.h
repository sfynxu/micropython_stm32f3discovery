#define STATIC_AF_TIM2_CH1_ETR(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_A5_obj)")  & strcmp( #pin_obj , "((&pin_A5_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D3_obj)")  & strcmp( #pin_obj , "((&pin_D3_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G1_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART2_CTS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D3_obj)")  & strcmp( #pin_obj , "((&pin_D3_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP1_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_F3_obj)")  & strcmp( #pin_obj , "((&pin_F3_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_BKIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (11) : \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_D2_obj)")  & strcmp( #pin_obj , "((&pin_D2_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_ETR(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_EVENTOUT_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A0_obj)")  & strcmp( #pin_obj , "((&pin_A0_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A1_obj)")  & strcmp( #pin_obj , "((&pin_A1_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A2_obj)")  & strcmp( #pin_obj , "((&pin_A2_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A3_obj)")  & strcmp( #pin_obj , "((&pin_A3_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A5_obj)")  & strcmp( #pin_obj , "((&pin_A5_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B0_obj)")  & strcmp( #pin_obj , "((&pin_B0_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B10_obj)")  & strcmp( #pin_obj , "((&pin_B10_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B11_obj)")  & strcmp( #pin_obj , "((&pin_B11_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B13_obj)")  & strcmp( #pin_obj , "((&pin_B13_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B1_obj)")  & strcmp( #pin_obj , "((&pin_B1_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B2_obj)")  & strcmp( #pin_obj , "((&pin_B2_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (15) : \
    ((strcmp( #pin_obj , "(&pin_C0_obj)")  & strcmp( #pin_obj , "((&pin_C0_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C10_obj)")  & strcmp( #pin_obj , "((&pin_C10_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C11_obj)")  & strcmp( #pin_obj , "((&pin_C11_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C12_obj)")  & strcmp( #pin_obj , "((&pin_C12_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C1_obj)")  & strcmp( #pin_obj , "((&pin_C1_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C2_obj)")  & strcmp( #pin_obj , "((&pin_C2_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C3_obj)")  & strcmp( #pin_obj , "((&pin_C3_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C4_obj)")  & strcmp( #pin_obj , "((&pin_C4_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C5_obj)")  & strcmp( #pin_obj , "((&pin_C5_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C6_obj)")  & strcmp( #pin_obj , "((&pin_C6_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C7_obj)")  & strcmp( #pin_obj , "((&pin_C7_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C8_obj)")  & strcmp( #pin_obj , "((&pin_C8_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_C9_obj)")  & strcmp( #pin_obj , "((&pin_C9_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D0_obj)")  & strcmp( #pin_obj , "((&pin_D0_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D10_obj)")  & strcmp( #pin_obj , "((&pin_D10_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D11_obj)")  & strcmp( #pin_obj , "((&pin_D11_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D12_obj)")  & strcmp( #pin_obj , "((&pin_D12_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D13_obj)")  & strcmp( #pin_obj , "((&pin_D13_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D14_obj)")  & strcmp( #pin_obj , "((&pin_D14_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D15_obj)")  & strcmp( #pin_obj , "((&pin_D15_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D1_obj)")  & strcmp( #pin_obj , "((&pin_D1_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D2_obj)")  & strcmp( #pin_obj , "((&pin_D2_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D3_obj)")  & strcmp( #pin_obj , "((&pin_D3_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D4_obj)")  & strcmp( #pin_obj , "((&pin_D4_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D5_obj)")  & strcmp( #pin_obj , "((&pin_D5_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D6_obj)")  & strcmp( #pin_obj , "((&pin_D6_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D7_obj)")  & strcmp( #pin_obj , "((&pin_D7_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D8_obj)")  & strcmp( #pin_obj , "((&pin_D8_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D9_obj)")  & strcmp( #pin_obj , "((&pin_D9_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E0_obj)")  & strcmp( #pin_obj , "((&pin_E0_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E10_obj)")  & strcmp( #pin_obj , "((&pin_E10_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E11_obj)")  & strcmp( #pin_obj , "((&pin_E11_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E12_obj)")  & strcmp( #pin_obj , "((&pin_E12_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E13_obj)")  & strcmp( #pin_obj , "((&pin_E13_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E14_obj)")  & strcmp( #pin_obj , "((&pin_E14_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E15_obj)")  & strcmp( #pin_obj , "((&pin_E15_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E1_obj)")  & strcmp( #pin_obj , "((&pin_E1_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E2_obj)")  & strcmp( #pin_obj , "((&pin_E2_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E3_obj)")  & strcmp( #pin_obj , "((&pin_E3_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E4_obj)")  & strcmp( #pin_obj , "((&pin_E4_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E5_obj)")  & strcmp( #pin_obj , "((&pin_E5_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E6_obj)")  & strcmp( #pin_obj , "((&pin_E6_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E7_obj)")  & strcmp( #pin_obj , "((&pin_E7_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E8_obj)")  & strcmp( #pin_obj , "((&pin_E8_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E9_obj)")  & strcmp( #pin_obj , "((&pin_E9_obj))")) == 0) ? (1) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_RTC_REFIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A1_obj)")  & strcmp( #pin_obj , "((&pin_A1_obj))")) == 0) ? (0) : \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM2_CH2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A1_obj)")  & strcmp( #pin_obj , "((&pin_A1_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D4_obj)")  & strcmp( #pin_obj , "((&pin_D4_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G1_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A1_obj)")  & strcmp( #pin_obj , "((&pin_A1_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART2_RTS_DE(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A1_obj)")  & strcmp( #pin_obj , "((&pin_A1_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D4_obj)")  & strcmp( #pin_obj , "((&pin_D4_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM15_CH1N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A1_obj)")  & strcmp( #pin_obj , "((&pin_A1_obj))")) == 0) ? (9) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM2_CH3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A2_obj)")  & strcmp( #pin_obj , "((&pin_A2_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B10_obj)")  & strcmp( #pin_obj , "((&pin_B10_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D7_obj)")  & strcmp( #pin_obj , "((&pin_D7_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G1_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A2_obj)")  & strcmp( #pin_obj , "((&pin_A2_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART2_TX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_A2_obj)")  & strcmp( #pin_obj , "((&pin_A2_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D5_obj)")  & strcmp( #pin_obj , "((&pin_D5_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP2_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_A2_obj)")  & strcmp( #pin_obj , "((&pin_A2_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (8) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM15_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A2_obj)")  & strcmp( #pin_obj , "((&pin_A2_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_F9_obj)")  & strcmp( #pin_obj , "((&pin_F9_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM2_CH4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_A3_obj)")  & strcmp( #pin_obj , "((&pin_A3_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B11_obj)")  & strcmp( #pin_obj , "((&pin_B11_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_D6_obj)")  & strcmp( #pin_obj , "((&pin_D6_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G1_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A3_obj)")  & strcmp( #pin_obj , "((&pin_A3_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART2_RX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_A3_obj)")  & strcmp( #pin_obj , "((&pin_A3_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D6_obj)")  & strcmp( #pin_obj , "((&pin_D6_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM15_CH2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A3_obj)")  & strcmp( #pin_obj , "((&pin_A3_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_F10_obj)")  & strcmp( #pin_obj , "((&pin_F10_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM3_CH2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_C7_obj)")  & strcmp( #pin_obj , "((&pin_C7_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_E3_obj)")  & strcmp( #pin_obj , "((&pin_E3_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G2_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI1_NSS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI3_NSS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S3WS_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART2_CK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A4_obj)")  & strcmp( #pin_obj , "((&pin_A4_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D7_obj)")  & strcmp( #pin_obj , "((&pin_D7_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G2_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A5_obj)")  & strcmp( #pin_obj , "((&pin_A5_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI1_SCK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A5_obj)")  & strcmp( #pin_obj , "((&pin_A5_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM16_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E0_obj)")  & strcmp( #pin_obj , "((&pin_E0_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM3_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_C6_obj)")  & strcmp( #pin_obj , "((&pin_C6_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_E2_obj)")  & strcmp( #pin_obj , "((&pin_E2_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G2_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI1_MISO(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_BKIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_A6_obj)")  & strcmp( #pin_obj , "((&pin_A6_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (12) : \
    ((strcmp( #pin_obj , "(&pin_E15_obj)")  & strcmp( #pin_obj , "((&pin_E15_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM17_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_E1_obj)")  & strcmp( #pin_obj , "((&pin_E1_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G2_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH1N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_C10_obj)")  & strcmp( #pin_obj , "((&pin_C10_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI1_MOSI(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH1N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_A7_obj)")  & strcmp( #pin_obj , "((&pin_A7_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C13_obj)")  & strcmp( #pin_obj , "((&pin_C13_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_E8_obj)")  & strcmp( #pin_obj , "((&pin_E8_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_MCO_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2C2_SMBA(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S2_MCK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_C6_obj)")  & strcmp( #pin_obj , "((&pin_C6_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_E9_obj)")  & strcmp( #pin_obj , "((&pin_E9_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART1_CK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP3_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_C8_obj)")  & strcmp( #pin_obj , "((&pin_C8_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM4_ETR(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A8_obj)")  & strcmp( #pin_obj , "((&pin_A8_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_E0_obj)")  & strcmp( #pin_obj , "((&pin_E0_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G4_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2C2_SCL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_F1_obj)")  & strcmp( #pin_obj , "((&pin_F1_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_F6_obj)")  & strcmp( #pin_obj , "((&pin_F6_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S3_MCK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_C7_obj)")  & strcmp( #pin_obj , "((&pin_C7_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_E11_obj)")  & strcmp( #pin_obj , "((&pin_E11_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART1_TX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_C4_obj)")  & strcmp( #pin_obj , "((&pin_C4_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_E0_obj)")  & strcmp( #pin_obj , "((&pin_E0_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP5_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_C7_obj)")  & strcmp( #pin_obj , "((&pin_C7_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM15_BKIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A9_obj)")  & strcmp( #pin_obj , "((&pin_A9_obj))")) == 0) ? (9) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM17_BKIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (10) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G4_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2C2_SDA(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_F0_obj)")  & strcmp( #pin_obj , "((&pin_F0_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_E13_obj)")  & strcmp( #pin_obj , "((&pin_E13_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART1_RX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_C5_obj)")  & strcmp( #pin_obj , "((&pin_C5_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_E1_obj)")  & strcmp( #pin_obj , "((&pin_E1_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP6_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A10_obj)")  & strcmp( #pin_obj , "((&pin_A10_obj))")) == 0) ? (8) : \
    ((strcmp( #pin_obj , "(&pin_C6_obj)")  & strcmp( #pin_obj , "((&pin_C6_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART1_CTS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_CAN1_RX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_D0_obj)")  & strcmp( #pin_obj , "((&pin_D0_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM4_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_D12_obj)")  & strcmp( #pin_obj , "((&pin_D12_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (11) : \
    ((strcmp( #pin_obj , "(&pin_E14_obj)")  & strcmp( #pin_obj , "((&pin_E14_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_BKIN2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (12) : \
    ((strcmp( #pin_obj , "(&pin_C3_obj)")  & strcmp( #pin_obj , "((&pin_C3_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_E14_obj)")  & strcmp( #pin_obj , "((&pin_E14_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USB_DM(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A11_obj)")  & strcmp( #pin_obj , "((&pin_A11_obj))")) == 0) ? (14) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH2N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_B0_obj)")  & strcmp( #pin_obj , "((&pin_B0_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_E10_obj)")  & strcmp( #pin_obj , "((&pin_E10_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART1_RTS_DE(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_CAN1_TX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (9) : \
    ((strcmp( #pin_obj , "(&pin_D1_obj)")  & strcmp( #pin_obj , "((&pin_D1_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM4_CH2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_D13_obj)")  & strcmp( #pin_obj , "((&pin_D13_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_ETR(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (11) : \
    ((strcmp( #pin_obj , "(&pin_E7_obj)")  & strcmp( #pin_obj , "((&pin_E7_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USB_DP(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A12_obj)")  & strcmp( #pin_obj , "((&pin_A12_obj))")) == 0) ? (14) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SWDIO_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_JTMS_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM16_CH1N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (1) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G4_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_IR_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART3_CTS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_B13_obj)")  & strcmp( #pin_obj , "((&pin_B13_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D11_obj)")  & strcmp( #pin_obj , "((&pin_D11_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM4_CH3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A13_obj)")  & strcmp( #pin_obj , "((&pin_A13_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_D14_obj)")  & strcmp( #pin_obj , "((&pin_D14_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SWCLK_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_JTCK_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G4_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2C1_SDA(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A14_obj)")  & strcmp( #pin_obj , "((&pin_A14_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_C7_obj)")  & strcmp( #pin_obj , "((&pin_C7_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_JTDI_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_C6_obj)")  & strcmp( #pin_obj , "((&pin_C6_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2C1_SCL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S3_WS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_A15_obj)")  & strcmp( #pin_obj , "((&pin_A15_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM3_CH3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B0_obj)")  & strcmp( #pin_obj , "((&pin_B0_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_C8_obj)")  & strcmp( #pin_obj , "((&pin_C8_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_E4_obj)")  & strcmp( #pin_obj , "((&pin_E4_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G3_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B0_obj)")  & strcmp( #pin_obj , "((&pin_B0_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH2N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B0_obj)")  & strcmp( #pin_obj , "((&pin_B0_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_C11_obj)")  & strcmp( #pin_obj , "((&pin_C11_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM3_CH4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B1_obj)")  & strcmp( #pin_obj , "((&pin_B1_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_C9_obj)")  & strcmp( #pin_obj , "((&pin_C9_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_E5_obj)")  & strcmp( #pin_obj , "((&pin_E5_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G3_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B1_obj)")  & strcmp( #pin_obj , "((&pin_B1_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH3N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B1_obj)")  & strcmp( #pin_obj , "((&pin_B1_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (3) : \
    ((strcmp( #pin_obj , "(&pin_C12_obj)")  & strcmp( #pin_obj , "((&pin_C12_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM1_CH3N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_B1_obj)")  & strcmp( #pin_obj , "((&pin_B1_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_E12_obj)")  & strcmp( #pin_obj , "((&pin_E12_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_F0_obj)")  & strcmp( #pin_obj , "((&pin_F0_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP4_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B1_obj)")  & strcmp( #pin_obj , "((&pin_B1_obj))")) == 0) ? (8) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G3_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B2_obj)")  & strcmp( #pin_obj , "((&pin_B2_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_JTDO_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TRACESWO_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G5_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI3_SCK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C10_obj)")  & strcmp( #pin_obj , "((&pin_C10_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S3_CK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C10_obj)")  & strcmp( #pin_obj , "((&pin_C10_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM3_ETR(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B3_obj)")  & strcmp( #pin_obj , "((&pin_B3_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_D2_obj)")  & strcmp( #pin_obj , "((&pin_D2_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_NJTRST_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G5_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI3_MISO(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C11_obj)")  & strcmp( #pin_obj , "((&pin_C11_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S3_EXTSD(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B4_obj)")  & strcmp( #pin_obj , "((&pin_B4_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C11_obj)")  & strcmp( #pin_obj , "((&pin_C11_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM16_BKIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (1) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2C1_SMBA(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI3_MOSI(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C12_obj)")  & strcmp( #pin_obj , "((&pin_C12_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S3_SD(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B5_obj)")  & strcmp( #pin_obj , "((&pin_B5_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_C12_obj)")  & strcmp( #pin_obj , "((&pin_C12_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G5_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_BKIN2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B6_obj)")  & strcmp( #pin_obj , "((&pin_B6_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_C9_obj)")  & strcmp( #pin_obj , "((&pin_C9_obj))")) == 0) ? (6) : \
    ((strcmp( #pin_obj , "(&pin_D1_obj)")  & strcmp( #pin_obj , "((&pin_D1_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM17_CH1N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (1) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G5_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B7_obj)")  & strcmp( #pin_obj , "((&pin_B7_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_SYNC(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B10_obj)")  & strcmp( #pin_obj , "((&pin_B10_obj))")) == 0) ? (3) : \
    ((strcmp( #pin_obj , "(&pin_B8_obj)")  & strcmp( #pin_obj , "((&pin_B8_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM4_CH4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_D15_obj)")  & strcmp( #pin_obj , "((&pin_D15_obj))")) == 0) ? (2) : \
    ((strcmp( #pin_obj , "(&pin_F6_obj)")  & strcmp( #pin_obj , "((&pin_F6_obj))")) == 0) ? (2) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B9_obj)")  & strcmp( #pin_obj , "((&pin_B9_obj))")) == 0) ? (10) : \
    ((strcmp( #pin_obj , "(&pin_C8_obj)")  & strcmp( #pin_obj , "((&pin_C8_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART3_TX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B10_obj)")  & strcmp( #pin_obj , "((&pin_B10_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_C10_obj)")  & strcmp( #pin_obj , "((&pin_C10_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D8_obj)")  & strcmp( #pin_obj , "((&pin_D8_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G6_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B11_obj)")  & strcmp( #pin_obj , "((&pin_B11_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART3_RX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B11_obj)")  & strcmp( #pin_obj , "((&pin_B11_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_C11_obj)")  & strcmp( #pin_obj , "((&pin_C11_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D9_obj)")  & strcmp( #pin_obj , "((&pin_D9_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_E15_obj)")  & strcmp( #pin_obj , "((&pin_E15_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G6_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI2_NSS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_D15_obj)")  & strcmp( #pin_obj , "((&pin_D15_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S2_WS(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART3_CK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B12_obj)")  & strcmp( #pin_obj , "((&pin_B12_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_C12_obj)")  & strcmp( #pin_obj , "((&pin_C12_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D10_obj)")  & strcmp( #pin_obj , "((&pin_D10_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G6_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B13_obj)")  & strcmp( #pin_obj , "((&pin_B13_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI2_SCK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B13_obj)")  & strcmp( #pin_obj , "((&pin_B13_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_F10_obj)")  & strcmp( #pin_obj , "((&pin_F10_obj))")) == 0) ? (5) : \
    ((strcmp( #pin_obj , "(&pin_F9_obj)")  & strcmp( #pin_obj , "((&pin_F9_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S2_CK(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B13_obj)")  & strcmp( #pin_obj , "((&pin_B13_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM_CH1N(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B13_obj)")  & strcmp( #pin_obj , "((&pin_B13_obj))")) == 0) ? (6) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G6_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI2_MISO(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S2_EXTSD(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_USART3_RTS_DE(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B14_obj)")  & strcmp( #pin_obj , "((&pin_B14_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_D12_obj)")  & strcmp( #pin_obj , "((&pin_D12_obj))")) == 0) ? (7) : \
    ((strcmp( #pin_obj , "(&pin_F6_obj)")  & strcmp( #pin_obj , "((&pin_F6_obj))")) == 0) ? (7) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_SPI2_MOSI(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S2_SD(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_B15_obj)")  & strcmp( #pin_obj , "((&pin_B15_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_COMP7_OUT(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C2_obj)")  & strcmp( #pin_obj , "((&pin_C2_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G3_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C5_obj)")  & strcmp( #pin_obj , "((&pin_C5_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TIM8_CH4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C9_obj)")  & strcmp( #pin_obj , "((&pin_C9_obj))")) == 0) ? (4) : \
    ((strcmp( #pin_obj , "(&pin_D1_obj)")  & strcmp( #pin_obj , "((&pin_D1_obj))")) == 0) ? (4) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_I2S_CKIN(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C9_obj)")  & strcmp( #pin_obj , "((&pin_C9_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_UART4_TX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C10_obj)")  & strcmp( #pin_obj , "((&pin_C10_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_UART4_RX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C11_obj)")  & strcmp( #pin_obj , "((&pin_C11_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_UART5_TX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_C12_obj)")  & strcmp( #pin_obj , "((&pin_C12_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_UART5_RX(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_D2_obj)")  & strcmp( #pin_obj , "((&pin_D2_obj))")) == 0) ? (5) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G8_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_D12_obj)")  & strcmp( #pin_obj , "((&pin_D12_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G8_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_D13_obj)")  & strcmp( #pin_obj , "((&pin_D13_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G8_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_D14_obj)")  & strcmp( #pin_obj , "((&pin_D14_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G8_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_D15_obj)")  & strcmp( #pin_obj , "((&pin_D15_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TRACECK_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E2_obj)")  & strcmp( #pin_obj , "((&pin_E2_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G7_IO1(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E2_obj)")  & strcmp( #pin_obj , "((&pin_E2_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TRACED_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E3_obj)")  & strcmp( #pin_obj , "((&pin_E3_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G7_IO2(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E3_obj)")  & strcmp( #pin_obj , "((&pin_E3_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TRACED1_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E4_obj)")  & strcmp( #pin_obj , "((&pin_E4_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G7_IO3(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E4_obj)")  & strcmp( #pin_obj , "((&pin_E4_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TRACED2_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E5_obj)")  & strcmp( #pin_obj , "((&pin_E5_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TSC_G7_IO4(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E5_obj)")  & strcmp( #pin_obj , "((&pin_E5_obj))")) == 0) ? (3) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_TRACED3_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_E6_obj)")  & strcmp( #pin_obj , "((&pin_E6_obj))")) == 0) ? (0) : \
    (0xffffffffffffffffULL))

#define STATIC_AF_EVENOUT_NULL(pin_obj) ( \
    ((strcmp( #pin_obj , "(&pin_F10_obj)")  & strcmp( #pin_obj , "((&pin_F10_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_F2_obj)")  & strcmp( #pin_obj , "((&pin_F2_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_F3_obj)")  & strcmp( #pin_obj , "((&pin_F3_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_F6_obj)")  & strcmp( #pin_obj , "((&pin_F6_obj))")) == 0) ? (1) : \
    ((strcmp( #pin_obj , "(&pin_F9_obj)")  & strcmp( #pin_obj , "((&pin_F9_obj))")) == 0) ? (1) : \
    (0xffffffffffffffffULL))

