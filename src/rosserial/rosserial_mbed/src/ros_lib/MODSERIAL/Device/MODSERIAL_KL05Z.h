#if defined(TARGET_KL05Z)

#define MODSERIAL_IRQ_REG ((UARTLP_Type*)_base)->C2
#define DISABLE_TX_IRQ MODSERIAL_IRQ_REG &= ~(1UL << UARTLP_C2_TIE_SHIFT)
#define DISABLE_RX_IRQ MODSERIAL_IRQ_REG &= ~(1UL << UARTLP_C2_RIE_SHIFT)
#define ENABLE_TX_IRQ MODSERIAL_IRQ_REG |= (1UL << UARTLP_C2_TIE_SHIFT)
#define ENABLE_RX_IRQ MODSERIAL_IRQ_REG |= (1UL << UARTLP_C2_RIE_SHIFT)

#define MODSERIAL_READ_REG ((UARTLP_Type*)_base)->D
#define MODSERIAL_WRITE_REG ((UARTLP_Type*)_base)->D
#define MODSERIAL_READABLE ((((UARTLP_Type*)_base)->S1 & (1UL<<5)) != 0)
#define MODSERIAL_WRITABLE ((((UARTLP_Type*)_base)->S1 & (1UL<<7)) != 0)

#define RESET_TX_FIFO while(0 == 1)
#define RESET_RX_FIFO while(MODSERIAL_READABLE) char dummy = MODSERIAL_READ_REG

#define RX_IRQ_ENABLED ((MODSERIAL_IRQ_REG & (1UL << UARTLP_C2_RIE_SHIFT)) != 0 )
#define TX_IRQ_ENABLED ((MODSERIAL_IRQ_REG & (1UL << UARTLP_C2_TIE_SHIFT)) != 0 )

#endif