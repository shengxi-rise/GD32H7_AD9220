/*!
    \file    main.c
    \brief   running LED

    \version 2023-06-21, V1.0.0, demo for GD32H7xx
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32h7xx.h"
#include "systick.h"
#include <stdio.h>
#include "gd32h7xx_gpio.h"

// 这个值也要改
__attribute__ ((aligned(32))) uint32_t AD9920Buf[2048];

/*!
    \brief      enable the CPU Chache
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void cache_enable(void) {
    /* Enable I-Cache */
    SCB_EnableICache();

    /* Enable D-Cache */
    SCB_EnableDCache();
}

/// UART
static void usart_config(void) {
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* connect port to USART0 TX */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);

    /* connect port to USART0 RX */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);

    /* configure USART TX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_9);

    /* configure USART RX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

// DMA init
/// TODO: 具体对应通道还没确定
///
void dma_config(void) {
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMAMUX);
//    nvic_config();

    // 多数据
    dma_multi_data_parameter_struct dma0;
    dma_deinit(DMA0, DMA_CH0);
    dma_multi_data_para_struct_init(&dma0);
    dma0.request = DMA_REQUEST_TIMER0_UP;
    dma0.periph_addr = (uint32_t) (&GPIO_ISTAT(GPIOB));
    dma0.periph_width = DMA_PERIPH_WIDTH_32BIT;
    dma0.periph_inc = DMA_PERIPH_INCREASE_DISABLE;   // 开启地址自增
    dma0.memory0_addr = (uint32_t) AD9920Buf;
    dma0.memory_width = DMA_PERIPH_WIDTH_32BIT;
    dma0.memory_inc = DMA_MEMORY_INCREASE_ENABLE;   // 地址自增
    dma0.circular_mode = DMA_CIRCULAR_MODE_DISABLE;     // 循环关闭
    dma0.direction = DMA_PERIPH_TO_MEMORY; // P T M
    dma0.number = 2048;           // 这个是什么配置
    dma0.priority = DMA_PRIORITY_ULTRA_HIGH; // very high
    dma_multi_data_mode_init(DMA0, DMA_CH0, &dma0);

    // DMA0_CH0
}

// TIM init
/* TIM configuration */
void tim_config(void) {
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_GPIOC);
    timer_deinit(TIMER0);

    timer_parameter_struct timer0;
    timer_oc_parameter_struct timer_ocintpara;

    timer0.prescaler = 5 - 1;       // PSC
    timer0.alignedmode = TIMER_COUNTER_EDGE;    // 这个是什么
    timer0.counterdirection = TIMER_COUNTER_UP;    // 向上计数
    timer0.period = 6-1;           // ARR
    timer0.clockdivision = TIMER_CKDIV_DIV1;    // 不分频
    timer0.repetitioncounter = 0U;
    timer_init(TIMER0, &timer0);

    /* CH0 configuration in PWM mode */
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);

//    timer_autoreload_value_config(TIMER0, 5);   // autoreload arr   加了这个就输出不了了
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 3);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    timer_dma_enable(TIMER0, TIMER_DMA_UPD);
    timer_primary_output_config(TIMER0, ENABLE);

    // auto preload enable
    timer_auto_reload_shadow_enable(TIMER0);
//    timer_interrupt_enable(TIMER0, TIMER_INT_UP);
    timer_enable(TIMER0);

    /// PWM GPIO
    gpio_af_set(GPIOC, GPIO_AF_1, GPIO_PIN_7);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_7);

}
///  串口输出重定向
int _write (int fd, char *pBuffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        while(RESET == usart_flag_get(USART0, USART_FLAG_TC));//等待上一次串口数据发送完成
        usart_data_transmit(USART0, pBuffer[i]);//写DR,串口1将发送数据
    }
    return size;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {
    /* enable the CPU Cache */
    cache_enable();
    /* configure systick */
    systick_config();

    /* enable the LED GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOA);
    // GPIOB
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_ALL);

    /* configure LED1 GPIO pin */
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_10);
    // /* configure LED2 GPIO pin */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_6);

    // /* reset LED1 GPIO pin */
    gpio_bit_set(GPIOF, GPIO_PIN_10);
    /* reset LED2 GPIO pin */
    gpio_bit_set(GPIOA, GPIO_PIN_6);

    tim_config();
    dma_config();
    usart_config();
    /* output a message on hyperterminal using printf function */

    // 开启dma中断
    AD9920Buf[0] = 1;
    dma_channel_enable(DMA0, DMA_CH0);

    delay_1ms(2000);        // 2s能采集完了
    dma_channel_disable(DMA0, DMA_CH0);

    /// 写成数组的形式（测试某个东西用的）
//    uint16_t tmp[2048];
//    for (int i = 0; i < 2048; i++) {
//        tmp[i] = AD9920Buf[i] & 0X1FFF;
//        tmp[i] = ((tmp[i] << 1) & 0X1FFF) | (tmp[i] >> 12);
//    }
//
//    for (int i = 0; i < 2048; i++) {
//        printf("%d: %f\r\n",i, (float) tmp[i]* 5000.0f / 4096.0f);
//        while (RESET == usart_flag_get(USART0, USART_FLAG_TC)) {
//        }
//    }

    /// 单个变量
    for (int i = 0; i < 2048; i++) {
        uint16_t tmp = AD9920Buf[i] & 0X1FFF;
        tmp = ((tmp << 1) & 0X1FFF) | (tmp >> 12);
        printf("%d: %f\r\n",i, (float) tmp* 5000.0f / 4096.0f);
        while (RESET == usart_flag_get(USART0, USART_FLAG_TC)) {
        }
    }

    /* waiting for the transfer to complete*/

    while (1) {
        /* turn on LED2 and LED1 */
        /// 闪一下灯
        gpio_bit_set(GPIOA, GPIO_PIN_6);
        gpio_bit_set(GPIOF, GPIO_PIN_10);
        delay_1ms(500);

        /* turn off LED1 and LED2 */
        gpio_bit_set(GPIOA, GPIO_PIN_6);
        gpio_bit_reset(GPIOF, GPIO_PIN_10);
        delay_1ms(500);
    }
}
