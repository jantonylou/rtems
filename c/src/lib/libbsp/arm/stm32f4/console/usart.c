/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Obere Lagerstr. 30
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <libchip/sersupp.h>

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/rcc.h>
#include <bsp/irq.h>
#include <bsp/usart.h>
#include <bsp/stm32f4.h>

static volatile stm32f4_usart *usart_get_regs(const console_tbl *ct)
{
  return (stm32f4_usart *) ct->ulCtrlPort1;
}

#if STM32_CONSOLE_USE_INTERRUPTS
static rtems_vector_number usart_get_irq_number(const console_tbl *ct)
{
  return ct->ulIntVector;
}
#endif

static const stm32f4_rcc_index usart_rcc_index [] = {
  STM32F4_RCC_USART1,
  STM32F4_RCC_USART2,
  STM32F4_RCC_USART3,
  STM32F4_RCC_UART4,
  STM32F4_RCC_UART5,
#ifdef STM32F4_FAMILY_F4XXXX
  STM32F4_RCC_USART6
#endif /* STM32F4_FAMILY_F4XXXX */
};

static stm32f4_rcc_index usart_get_rcc_index(const console_tbl *ct)
{
  return usart_rcc_index [ct->ulCtrlPort2];
}

static const uint8_t usart_pclk_index [] = { 1, 0, 0, 0, 0, 1 };

static const uint32_t usart_pclk_by_index [] = {
  STM32F4_PCLK1,
  STM32F4_PCLK2
};

static uint32_t usart_get_pclk(const console_tbl *ct)
{
  return usart_pclk_by_index [usart_pclk_index [ct->ulCtrlPort2]];
}

static uint32_t usart_get_baud(const console_tbl *ct)
{
  return ct->ulClock;
}

/*
 * a = 8 * (2 - CR1[OVER8])
 *
 * usartdiv = div_mantissa + div_fraction / a
 *
 * baud = pclk / (a * usartdiv)
 *
 * usartdiv = pclk / (a * baud)
 *
 * Calculation in integer arithmetic:
 *
 * 1. div_mantissa = pclk / (a * baud)
 *
 * 2. div_fraction = pclk / (baud - a * div_mantissa)
 */
static uint32_t usart_get_bbr(
  volatile stm32f4_usart *usart,
  uint32_t pclk,
  uint32_t baud
)
{
  uint32_t a = 8 * (2 - ((usart->cr1 & STM32F4_USART_CR1_OVER8) != 0));
  uint32_t div_mantissa_low = pclk / (a * baud);
  uint32_t div_fraction_low = pclk / (baud - a * div_mantissa_low);
  uint32_t div_mantissa_high;
  uint32_t div_fraction_high;
  uint32_t high_err;
  uint32_t low_err;
  uint32_t div_mantissa;
  uint32_t div_fraction;

  if (div_fraction_low < a - 1) {
    div_mantissa_high = div_fraction_low;
    div_fraction_high = div_fraction_low + 1;
  } else {
    div_mantissa_high = div_fraction_low + 1;
    div_fraction_high = 0;
  }

  high_err = pclk - baud * (a * div_mantissa_high + div_fraction_high);
  low_err = baud * (a * div_mantissa_low + div_fraction_low) - pclk;

  if (low_err < high_err) {
    div_mantissa = div_mantissa_low;
    div_fraction = div_fraction_low;
  } else {
    div_mantissa = div_mantissa_high;
    div_fraction = div_fraction_high;
  }

  return STM32F4_USART_BBR_DIV_MANTISSA(div_mantissa)
    | STM32F4_USART_BBR_DIV_FRACTION(div_fraction);
}

static void usart_initialize(int minor)
{
  const console_tbl *ct = Console_Port_Tbl [minor];
  volatile stm32f4_usart *usart = usart_get_regs(ct);
  stm32f4_rcc_index rcc_index = usart_get_rcc_index(ct);

  stm32f4_rcc_set_clock(rcc_index, true);

  usart->cr1 = 0;
  usart->cr2 = 0;
  usart->cr3 = 0;
}

#if STM32_CONSOLE_USE_INTERRUPTS
static void usart_interrupt_handler(void *arg)
{
  struct rtems_termios_tty *tty = arg;
  const console_tbl *ct = Console_Port_Tbl [tty->minor];
  console_data *cd = &Console_Port_Data [tty->minor];
  volatile stm32f4_usart *usart = usart_get_regs(ct);
  uint32_t sr=usart->sr, clr=0;

  if(cd->bActive && (sr & STM32F4_USART_SR_TXE)) {
      clr |= STM32F4_USART_CR1_TXEIE;
      usart->cr1 &= ~STM32F4_USART_CR1_TXEIE;
      rtems_termios_dequeue_characters(cd->termios_data, 1);
  }

  if(sr & (STM32F4_USART_SR_RXNE | STM32F4_USART_SR_ORE)) {
      char c = STM32F4_USART_DR_GET(usart->dr);
      clr |= STM32F4_USART_SR_RXNE | STM32F4_USART_SR_ORE;
      rtems_termios_enqueue_raw_characters(cd->termios_data, &c, 1);
  }

  usart->sr = (~clr)&0x3FF;
}
#endif

static int usart_first_open(int major, int minor, void *arg)
{
  rtems_libio_open_close_args_t *oc = (rtems_libio_open_close_args_t *) arg;
  struct rtems_termios_tty *tty = (struct rtems_termios_tty *) oc->iop->data1;
  const console_tbl *ct = Console_Port_Tbl [minor];
  console_data *cd = &Console_Port_Data [minor];
  uint32_t pclk = usart_get_pclk(ct);
  uint32_t baud = usart_get_baud(ct);
  volatile stm32f4_usart *usart = usart_get_regs(ct);

  cd->termios_data = tty;
  rtems_termios_set_initial_baud(tty, ct->ulClock);

  usart->bbr = usart_get_bbr(usart, pclk, baud);

  usart->cr1 =
      STM32F4_USART_CR1_UE
      | STM32F4_USART_CR1_TE
      | STM32F4_USART_CR1_RE;

#if STM32_CONSOLE_USE_INTERRUPTS
   rtems_interrupt_handler_install(
      usart_get_irq_number(ct),
      "UART",
      RTEMS_INTERRUPT_UNIQUE,
      usart_interrupt_handler,
      (void *) tty
   );
  usart->cr1 |= STM32F4_USART_CR1_RXNEIE;
#endif

  return RTEMS_SUCCESSFUL;
}

static int usart_last_close(int major, int minor, void *arg)
{
  const console_tbl *ct = Console_Port_Tbl [minor];
  volatile stm32f4_usart *usart = usart_get_regs(ct);
#if STM32_CONSOLE_USE_INTERRUPTS
  rtems_libio_open_close_args_t *oc = (rtems_libio_open_close_args_t *) arg;
  struct rtems_termios_tty *tty = (struct rtems_termios_tty *) oc->iop->data1;
#endif

  usart->cr1 = 0;

#if STM32_CONSOLE_USE_INTERRUPTS
  rtems_interrupt_handler_remove(
    usart_get_irq_number(ct),
    usart_interrupt_handler,
    (void *) tty
  );
#endif

  return RTEMS_SUCCESSFUL;
}

#if STM32_CONSOLE_USE_INTERRUPTS
static ssize_t usart_write_interrupt_driven(
  int minor,
  const char *buf,
  size_t len
)
{
  const console_tbl *ct = Console_Port_Tbl [minor];
  console_data *cd = &Console_Port_Data[minor];
  volatile stm32f4_usart *usart = usart_get_regs(ct);

  if (len > 0) {
    usart->dr = STM32F4_USART_DR(buf[0]);
    usart->cr1 |= STM32F4_USART_CR1_TXEIE;
    cd->bActive = true;
  } else {
    cd->bActive = false;
  }

  return 0;
}
#else

static int usart_read_polled(int minor)
{
  const console_tbl *ct = Console_Port_Tbl [minor];
  volatile stm32f4_usart *usart = usart_get_regs(ct);

  if ((usart->sr & STM32F4_USART_SR_RXNE) != 0) {
    return STM32F4_USART_DR_GET(usart->dr);
  } else {
    return -1;
  }
}

static void usart_write_polled(int minor, char c)
{
  const console_tbl *ct = Console_Port_Tbl [minor];
  volatile stm32f4_usart *usart = usart_get_regs(ct);

  while ((usart->sr & STM32F4_USART_SR_TXE) == 0) {
    /* Wait */
  }

  usart->dr = STM32F4_USART_DR(c);
}

static ssize_t usart_write_support_polled(
  int minor,
  const char *s,
  size_t n
)
{
  ssize_t i = 0;

  for (i = 0; i < n; ++i) {
    usart_write_polled(minor, s [i]);
  }

  return n;
}
#endif

static int usart_set_attributes(int minor, const struct termios *term)
{
  return RTEMS_SUCCESSFUL;
}

const console_fns stm32f4_usart_fns = {
  .deviceProbe = libchip_serial_default_probe,
  .deviceInitialize = usart_initialize,
  .deviceFirstOpen = usart_first_open,
  .deviceLastClose = usart_last_close,
  .deviceSetAttributes = usart_set_attributes,
#if STM32_CONSOLE_USE_INTERRUPTS
  .deviceWrite = usart_write_interrupt_driven,
  .deviceOutputUsesInterrupts = true
#else
  .deviceRead = usart_read_polled,
  .deviceWritePolled = usart_write_polled,
  .deviceWrite = usart_write_support_polled,
  .deviceOutputUsesInterrupts = false
#endif
};

