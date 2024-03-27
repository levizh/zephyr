
#include <kernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <soc.h>
#include <init.h>
#include <uart.h>
#include <clock_control.h>
#include <interrupt_controller/intc_hc32.h>

#include <linker/sections.h>

#if CONFIG_UART_INTERRUPT_DRIVEN
enum UART_TYPE{
    UART_INT_IDX_EI,
    UART_INT_IDX_RI,
    UART_INT_IDX_TI,
    UART_INT_IDX_TCI,
    UART_INT_IDX_RTO,
    UART_INT_NUM
};
#endif /* UART_INTERRUPT_DRIVEN */

struct hc32_usart_cb_data {
    /** Callback function */
    uart_irq_callback_user_data_t user_cb;
    /** User data. */
    void *user_data;
};

/* device config */
struct uart_hc32_config {
	struct uart_device_config uart_cfg;
	/* clock subsystem driving this peripheral */
	struct device *dev_clock;
};

/* driver data */
struct uart_hc32_data {
	/* Baud rate */
	uint32_t baud_rate;
	/* clock device */
	struct device *clock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	struct hc32_usart_cb_data cb[UART_INT_NUM];
#endif
};

/* convenience defines */
#define DEV_USART_CFG(dev)												\
	((const struct uart_device_config * const)(dev)->config->config_info)

#define DEV_USART_DATA(dev)												\
	((struct uart_hc32_data * const)(dev)->driver_data)

#define DEV_USART_BASE(dev)												\
	((CM_USART_TypeDef *)(DEV_USART_CFG(dev)->base))

static inline void uart_hc32_set_baudrate(struct device *dev,
					 uint32_t baud_rate)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	(void)USART_SetBaudrate(USARTx, baud_rate, NULL);
}

static inline void uart_hc32_set_parity(struct device *dev, uint32_t parity)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetParity(USARTx, parity);
}

static inline uint32_t uart_hc32_get_parity(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	return USART_GetParity(USARTx);
}

static inline void uart_hc32_set_stopbits(struct device *dev, uint32_t stopbits)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetStopBit(USARTx, stopbits);
}

static inline uint32_t uart_hc32_get_stopbits(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStopBit(USARTx);
}

static inline void uart_hc32_set_databits(struct device *dev, uint32_t databits)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetDataWidth(USARTx, databits);
}

static inline uint32_t uart_hc32_get_databits(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetDataWidth(USARTx);
}

static inline void uart_hc32_set_hwctrl(const struct device *dev,
					 uint32_t hwctrl)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetHWFlowControl(USARTx, hwctrl);
}

static inline uint32_t uart_hc32_get_hwctrl(const struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetHWFlowControl(USARTx);
}

static inline uint32_t uart_hc32_cfg2ll_parity(enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return USART_PARITY_ODD;
	case UART_CFG_PARITY_EVEN:
		return USART_PARITY_EVEN;
	case UART_CFG_PARITY_NONE:
	default:
		return USART_PARITY_NONE;
	}
}

static inline enum uart_config_parity uart_hc32_ll2cfg_parity(uint32_t parity)
{
	switch (parity) {
	case USART_PARITY_ODD:
		return UART_CFG_PARITY_ODD;
	case USART_PARITY_EVEN:
		return UART_CFG_PARITY_EVEN;
	case USART_PARITY_NONE:
	default:
		return UART_CFG_PARITY_NONE;
	}
}

static inline uint32_t uart_hc32_cfg2ll_stopbits(enum uart_config_stop_bits sb)
{
	switch (sb) {
	/* Some MCU's don't support 0.5 stop bits */
#ifdef USART_STOPBIT_2BIT
	case UART_CFG_STOP_BITS_2:
		return USART_STOPBIT_2BIT;
#endif	/* USART_STOPBIT_2BIT */

#ifdef USART_STOPBIT_1BIT
	case UART_CFG_STOP_BITS_1:
#endif	/* USART_STOPBIT_1BIT */
	default:
		return USART_STOPBIT_1BIT;
	}
}

static inline enum uart_config_stop_bits uart_hc32_ll2cfg_stopbits(uint32_t sb)
{
	switch (sb) {
#ifdef USART_STOPBIT_2BIT
	case USART_STOPBIT_2BIT:
		return UART_CFG_STOP_BITS_2;
#endif	/* USART_STOPBIT_1BIT */

#ifdef USART_STOPBIT_1BIT
		case USART_STOPBIT_1BIT:
#endif	/* USART_STOPBIT_1BIT */
		default:
			return UART_CFG_STOP_BITS_1;
	}
}

static inline uint32_t uart_hc32_cfg2ll_databits(enum uart_config_data_bits db)
{
	switch (db) {
	case UART_CFG_DATA_BITS_8:
	default:
		return USART_DATA_WIDTH_8BIT;
	}
}

static inline enum uart_config_data_bits uart_hc32_ll2cfg_databits(uint32_t db)
{
	switch (db) {
	case USART_DATA_WIDTH_8BIT:
	default:
		return UART_CFG_DATA_BITS_8;
	}
}

static inline uint32_t uart_hc32_cfg2ll_hwctrl(enum uart_config_flow_control fc)
{
	/* default config */
	return USART_HW_FLOWCTRL_RTS;
}

static inline enum uart_config_flow_control uart_hc32_ll2cfg_hwctrl(uint32_t fc)
{
	/* DDL driver compatible with cfg, just return none */
	return UART_CFG_FLOW_CTRL_NONE;
}

static int uart_device_configure(struct device *dev,
				const struct uart_config *cfg)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	uint32_t parity;
	uint32_t stopbits;
	uint32_t databits;

	/* Hardware doesn't support mark or space parity */
	if ((UART_CFG_PARITY_NONE != cfg->parity) &&
			(UART_CFG_PARITY_ODD != cfg->parity) &&
			(UART_CFG_PARITY_EVEN != cfg->parity)) {
		return -ENOTSUP;
	}
	/* Driver does not supports parity + 9 databits */
	if ((cfg->parity != UART_CFG_PARITY_NONE) &&
		(cfg->data_bits == USART_DATA_WIDTH_9BIT)) {
		return -ENOTSUP;
	}

	if (UART_CFG_STOP_BITS_1 != cfg->stop_bits
#if defined(USART_STOPBIT_2BIT)
		&& UART_CFG_STOP_BITS_2 != cfg->stop_bits
#endif
	) {
		return -ENOTSUP;
	}

	parity = uart_hc32_cfg2ll_parity(cfg->parity);
	stopbits = uart_hc32_cfg2ll_stopbits(cfg->stop_bits);
	databits = uart_hc32_cfg2ll_databits(cfg->data_bits);

	USART_FuncCmd(USARTx, USART_TX | USART_RX, DISABLE);

	if (parity != uart_hc32_get_parity(dev)) {
		uart_hc32_set_parity(dev, parity);
	}

	if (stopbits != uart_hc32_get_stopbits(dev)) {
		uart_hc32_set_stopbits(dev, stopbits);
	}

	if (databits != uart_hc32_get_databits(dev)) {
		uart_hc32_set_databits(dev, databits);
	}

	if (cfg->baudrate != data->baud_rate) {
		uart_hc32_set_baudrate(dev, cfg->baudrate);
		data->baud_rate = cfg->baudrate;
	}

	USART_FuncCmd(USARTx, USART_TX | USART_RX, DISABLE);

	return 0;
};

static int uart_device_config_get(struct device *dev, struct uart_config *cfg)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);

	cfg->baudrate = data->baud_rate;
	cfg->parity = uart_hc32_ll2cfg_parity(uart_hc32_get_parity(dev));
	cfg->stop_bits = uart_hc32_ll2cfg_stopbits(uart_hc32_get_stopbits(dev));
	cfg->data_bits = uart_hc32_ll2cfg_databits(uart_hc32_get_databits(dev));
	cfg->flow_ctrl = uart_hc32_ll2cfg_hwctrl(uart_hc32_get_hwctrl(dev));

	return 0;
}

static int uart_hc32_poll_in(struct device *dev, unsigned char *c)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	if (USART_GetStatus(USARTx, USART_FLAG_OVERRUN | USART_FLAG_RX_TIMEOUT)) {
		USART_ClearStatus(USARTx, USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR);
	}

	if (SET != USART_GetStatus(USARTx, USART_FLAG_RX_FULL))
	{
		return EIO;
	}

	*c = (unsigned char)USART_ReadData(USARTx);

	return 0;
}

static void uart_hc32_poll_out(struct device *dev,
					unsigned char c)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	/* Wait for TXE flag to be raised */
	while (RESET == USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
		;
	}

	USART_WriteData(USARTx, (uint16_t)c);
}

// static inline void __uart_hc32_get_clock(struct device *dev)
// {
// 	struct uart_hc32_data *data = DEV_USART_DATA(dev);
// 	struct device *clk =
// 		device_get_binding(HC32_CLOCK_CONTROL_NAME);

// 	__ASSERT_NO_MSG(clk);

// 	data->clock = clk;
// }

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_hc32_fifo_fill(struct device *dev, const u8_t *tx_data,
				  int size)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	u8_t num_tx = 0U;

	if (!USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
		return num_tx;
	}

	while ((size - num_tx > 0) &&
		USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
		/* Send a character (8bit , parity none) */
		uart_hc32_poll_out(dev, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_hc32_fifo_read(struct device *dev, u8_t *rx_data,
				  const int size)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	u8_t num_rx = 0U;
	int ret;

	while ((size - num_rx > 0) && USART_GetStatus(USARTx, USART_FLAG_RX_FULL)) {
		/* Receive a character (8bit , parity none) */
		ret = uart_hc32_poll_in(dev, rx_data + num_rx++);
		if (0 != ret) {
			return ret;
		}

		/* Clear overrun error flag */
		if (USART_GetStatus(USARTx, USART_FLAG_OVERRUN)) {
			USART_ClearStatus(USARTx, USART_FLAG_OVERRUN);
		}
	}
	return num_rx;
}

static void uart_hc32_irq_tx_enable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_TX_EMPTY | USART_INT_TX_CPLT, ENABLE);
}

static void uart_hc32_irq_tx_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_TX_EMPTY | USART_INT_TX_CPLT, DISABLE);
}

static int uart_hc32_irq_tx_ready(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY);
}

static int uart_hc32_irq_tx_complete(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_TX_CPLT);
}

static void uart_hc32_irq_rx_enable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_RX, ENABLE);
}

static void uart_hc32_irq_rx_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_RX, DISABLE);
}

static int uart_hc32_irq_rx_ready(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_RX_FULL);
}

static void uart_hc32_irq_err_enable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	/* Enable FE, ORE parity error interruption */
	USART_FuncCmd(USARTx, USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT, ENABLE);
}

static void uart_hc32_irq_err_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT, DISABLE);
}

static int uart_hc32_irq_is_pending(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_ALL);
}

static int uart_hc32_irq_update(struct device *dev)
{
	return 1;
}

/*
  user may use void *user_data = x to specify irq type, the x may be:
  0：usart_hc32f4_rx_error_isr
  1：usart_hc32f4_rx_full_isr
  2：usart_hc32f4_tx_empty_isr
  3：usart_hc32f4_tx_complete_isr
  4：usart_hc32f4_rx_timeout_isr
  others or user_data = NULL：set cb for all interrupt handlers
*/
static void uart_hc32_irq_callback_set(struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *user_data)
{
	uint32_t i;
	struct uart_hc32_data *data = DEV_USART_DATA(dev);

	if ((user_data == NULL) || (*(uint32_t *)user_data >= UART_INT_NUM)) {
		for (i = 0; i< UART_INT_NUM; i++) {
			data->cb[i].user_cb = cb;
			data->cb[i].user_data = user_data;
		}
	} else {
		i = *(uint32_t *)user_data;
		data->cb[i].user_cb = cb;
		data->cb[i].user_data = user_data;
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_hc32_driver_api = {
	.poll_in = uart_hc32_poll_in,
	.poll_out = uart_hc32_poll_out,
	.configure = uart_device_configure,
	.config_get = uart_device_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_hc32_fifo_fill,
	.fifo_read = uart_hc32_fifo_read,
	.irq_tx_enable = uart_hc32_irq_tx_enable,
	.irq_tx_disable = uart_hc32_irq_tx_disable,
	.irq_tx_ready = uart_hc32_irq_tx_ready,
	.irq_tx_complete = uart_hc32_irq_tx_complete,
	.irq_rx_enable = uart_hc32_irq_rx_enable,
	.irq_rx_disable = uart_hc32_irq_rx_disable,
	.irq_rx_ready = uart_hc32_irq_rx_ready,
	.irq_err_enable = uart_hc32_irq_err_enable,
	.irq_err_disable = uart_hc32_irq_err_disable,
	.irq_is_pending = uart_hc32_irq_is_pending,
	.irq_update = uart_hc32_irq_update,
	.irq_callback_set = uart_hc32_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_hc32_clocks_enable(const struct device *dev)
{
	// CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	// struct uart_hc32_data *data = DEV_USART_DATA(dev);
	// const struct device *const dev_clk = data->clock;
	// int err;

	// if (!device_is_ready(dev_clk)) {
	// 	LOG_ERR("clock control device not ready");
	// 	return -ENODEV;
	// }

	// /* enable clock */
	// err = clock_control_on(dev_clock, (clock_control_subsys_t)config->clk_cfg);
	// if (err != 0) {
	// 	return err;
	// }
	return 0;
}

static int uart_hc32_registers_configure(const struct device *dev)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	stc_usart_uart_init_t stcUartInit;

	USART_FuncCmd(USARTx, USART_FUNC_ALL, DISABLE);

	(void)USART_UART_StructInit(&stcUartInit);
	stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
	stcUartInit.u32Baudrate = data->baud_rate;

	(void)USART_UART_Init(USARTx, &stcUartInit, NULL);
	USART_FuncCmd(USARTx, USART_TX | USART_RX, ENABLE);

	return 0;
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_hc32_init(struct device *dev)
{
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	const struct uart_device_config *config = DEV_USART_CFG(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
	// struct uart_hc32_data *data = DEV_USART_DATA(dev);
	// CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	int err;

	err = uart_hc32_clocks_enable(dev);
	if (err < 0) {
		return err;
	}
	FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART4, ENABLE);

	// /* Configure dt provided device signals when available */
	// err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	// if (err < 0) {
	// 	return err;
	// }
	GPIO_SetFunc(GPIO_PORT_E, GPIO_PIN_06, GPIO_FUNC_36);
	GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_37);

	err = uart_hc32_registers_configure(dev);
	if (err < 0) {
		return err;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
	return 0;
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define USART_IRQ_ISR_CONFIG(isr_name_prefix, isr_idx, index)				\
	IRQ_CONNECT(DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx,					\
		DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx##_PRIORITY,				\
		isr_name_prefix##_##index,											\
		DEVICE_GET(uart_hc32_##index),										\
		0);																	\
	hc32_intc_irq_signin(													\
		DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx,							\
		DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx##_INT_SRC);				\
	irq_enable(DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx);

#define HC32_UART_IRQ_HANDLER_DECL(index)									\
	static void usart_hc32_isr_func_cfg_##index(struct device *dev);		\
	static void	usart_hc32_rx_error_isr_##index(const struct device *dev);	\
	static void usart_hc32_rx_full_isr_##index(const struct device *dev);	\
	static void usart_hc32_tx_empty_isr_##index(const struct device *dev);	\
	static void usart_hc32_tx_complete_isr_##index(const struct device *dev);\
	static void	usart_hc32_rx_timeout_isr_##index(const struct device *dev)

#define HC32_UART_ISR_DEF(index)											\
	static void usart_hc32_rx_error_isr_##index(const struct device *dev)	\
	{																		\
		struct uart_hc32_data *const data = DEV_USART_DATA(dev);			\
																			\
		if (data->cb[UART_INT_IDX_EI].user_cb) {							\
			data->cb[UART_INT_IDX_EI].user_cb( 								\
				data->cb[UART_INT_IDX_EI].user_data);						\
		}																	\
	}																		\
																			\
	static void usart_hc32_rx_full_isr_##index(const struct device *dev)	\
	{																		\
		struct uart_hc32_data *const data = DEV_USART_DATA(dev);			\
																			\
		if (data->cb[UART_INT_IDX_RI].user_cb) {							\
			data->cb[UART_INT_IDX_RI].user_cb( 								\
			data->cb[UART_INT_IDX_RI].user_data);							\
		}																	\
	}																		\
																			\
	static void 															\
		usart_hc32_tx_empty_isr_##index(const struct device *dev)			\
	{																		\
		struct uart_hc32_data *const data = DEV_USART_DATA(dev);			\
																			\
		if (data->cb[UART_INT_IDX_TI].user_cb) {							\
			data->cb[UART_INT_IDX_TI].user_cb( 								\
				data->cb[UART_INT_IDX_TI].user_data);						\
		}																	\
	}																		\
																			\
	static void usart_hc32_tx_complete_isr_##index(const struct device *dev)\
	{																		\
		struct uart_hc32_data *const data = DEV_USART_DATA(dev);			\
																			\
		if (data->cb[UART_INT_IDX_TCI].user_cb) {							\
			data->cb[UART_INT_IDX_TCI].user_cb( 							\
				data->cb[UART_INT_IDX_TCI].user_data);						\
		}																	\
	}																		\
																			\
	static void usart_hc32_rx_timeout_isr_##index(const struct device *dev)	\
		{																	\
			struct uart_hc32_data *const data = DEV_USART_DATA(dev);		\
																			\
			if (data->cb[UART_INT_IDX_RTO].user_cb) {						\
				data->cb[UART_INT_IDX_RTO].user_cb( 						\
					data->cb[UART_INT_IDX_RTO].user_data);					\
			}																\
		}																	\
																			\
	static void usart_hc32_isr_func_cfg_##index(struct device *dev)			\
	{																		\
		USART_IRQ_ISR_CONFIG(usart_hc32_rx_error_isr, 0, index)				\
		USART_IRQ_ISR_CONFIG(usart_hc32_rx_full_isr, 1, index)				\
		USART_IRQ_ISR_CONFIG(usart_hc32_tx_empty_isr, 2, index)				\
		USART_IRQ_ISR_CONFIG(usart_hc32_tx_complete_isr, 3, index)			\
		USART_IRQ_ISR_CONFIG(usart_hc32_rx_timeout_isr, 4, index)			\
	}

#define HC32_UART_ISR_FUN_CONFIG(index)										\
	.irq_config_func = usart_hc32_isr_func_cfg_##index,

#else  /* CONFIG_UART_INTERRUPT_DRIVEN */
#define HC32_UART_IRQ_HANDLER_DECL(index)
#define HC32_UART_ISR_DEF(index)
#define HC32_UART_ISR_FUN_CONFIG(index)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define HC32_UART_INIT(index)												\
HC32_UART_IRQ_HANDLER_DECL(index);											\
static const struct uart_hc32_config uart_hc32_cfg_##index = {				\
	.uart_cfg = {															\
		.base = (u8_t *)DT_XHSC_HC32_USART_##index##_BASE_ADDRESS,			\
		HC32_UART_ISR_FUN_CONFIG(index)										\
	},																		\
};																			\
static struct uart_hc32_data uart_hc32_data_##index = {						\
	.baud_rate = DT_XHSC_HC32_USART_##index##_CURRENT_SPEED					\
};																			\
DEVICE_AND_API_INIT(uart_hc32_##index, 										\
		DT_XHSC_HC32_USART_##index##_LABEL,									\
		&uart_hc32_init,													\
		&uart_hc32_data_##index, &uart_hc32_cfg_##index,					\
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,					\
		&uart_hc32_driver_api);												\
HC32_UART_ISR_DEF(index)

#ifdef CONFIG_UART_1
HC32_UART_INIT(USART_1)
#endif	/* CONFIG_UART_1 */

#ifdef CONFIG_UART_2
HC32_UART_INIT(USART_2)
#endif	/* CONFIG_UART_2 */

#ifdef CONFIG_UART_3
HC32_UART_INIT(USART_3)
#endif	/* CONFIG_UART_3 */

#ifdef CONFIG_UART_4
HC32_UART_INIT(0)
#endif /* CONFIG_UART_4 */
