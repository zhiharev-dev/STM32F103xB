/*
 * Copyright (C) 2025 zhiharev-dev <zhiharev.dev@mail.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Includes ---------------------------------------------------------------- */

#include "main.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define VTOR_ADDRESS            0x08000000

#define HSI_CLOCK               8000000

#define RCC_HSERDY_TIMEOUT      100

#define RCC_LSERDY_TIMEOUT      5000

#define RCC_CPU_CLOCK           72000000

#define RCC_AHB_CLOCK           (RCC_CPU_CLOCK / 1)

#define RCC_APB1_CLOCK          (RCC_AHB_CLOCK / 2)

#define RCC_APB2_CLOCK          (RCC_AHB_CLOCK / 1)

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Системный таймер (1 мс) */
volatile uint32_t systick;

/* Состояние VDD */
volatile bool vdd_is_lower;

/* Private function prototypes --------------------------------------------- */

static void setup_hardware(void);

static void setup_vector_table(void);

static void app_main(void);

static void systick_init(const uint32_t frequency);

static void pwr_init(void);

static void flash_init(void);

static void rcc_init(void);

/* Private user code ------------------------------------------------------- */

int main(void)
{
    setup_hardware();
    app_main();
}
/* ------------------------------------------------------------------------- */

void error(void)
{
    __disable_irq();

    while (true) {
        continue;
    }
}
/* ------------------------------------------------------------------------- */

static void app_main(void)
{
    while (true) {
        continue;
    }
}
/* ------------------------------------------------------------------------- */

static void setup_hardware(void)
{
    setup_vector_table();

    systick_init(HSI_CLOCK);
    pwr_init();
    flash_init();
    rcc_init();
    systick_init(RCC_CPU_CLOCK);
}
/* ------------------------------------------------------------------------- */

static void setup_vector_table(void)
{
    __disable_irq();
    __set_PRIMASK(1);

    WRITE_REG(SCB->VTOR, VTOR_ADDRESS);

    __set_PRIMASK(0);
    __enable_irq();
}
/* ------------------------------------------------------------------------- */

static void systick_init(const uint32_t frequency)
{
    /* Сбросить регистр управления */
    CLEAR_REG(SysTick->CTRL);

    /* Установить значение перезагрузки счетчика = 1 мс */
    WRITE_REG(SysTick->LOAD, (frequency / 1000) - 1);

    /* Установить текущее значение счетчика = 0 */
    CLEAR_REG(SysTick->VAL);

    /* Настроить тактирование от CPU и запустить таймер */
    WRITE_REG(SysTick->CTRL,
              SysTick_CTRL_CLKSOURCE_Msk
            | SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk);
}
/* ------------------------------------------------------------------------- */

static void pwr_init(void)
{
    /* Включить тактирование */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Msk);

    /* Отключить защиту домена резервного копирования */
    SET_BIT(PWR->CR, PWR_CR_DBP_Msk);

    /* Включить и настроить уровень PVD 2.9V */
    SET_BIT(PWR->CR,
            PWR_CR_PVDE_Msk
          | PWR_CR_PLS_Msk);

    /* Разрешить прерывание EXTI PVD output */
    SET_BIT(EXTI->IMR, EXTI_IMR_MR16_Msk);
    /* Включить Rising Trigger */
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR16_Msk);
    /* Включить Falling Trigger */
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR16_Msk);

    /* Настроить NVIC */
    NVIC_SetPriority(PVD_IRQn, 5);
    NVIC_EnableIRQ(PVD_IRQn);
}
/* ------------------------------------------------------------------------- */

static void flash_init(void)
{
    /* Настроить задержку чтения флэш-памяти = 2WS */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY_Msk,
               0x02 << FLASH_ACR_LATENCY_Pos);

    /* Включить предварительную выборку данных */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE_Msk);
    while (!READ_BIT(FLASH->ACR, FLASH_ACR_PRFTBS_Msk)) {
        continue;
    }
}
/* ------------------------------------------------------------------------- */

static void rcc_init(void)
{
    uint32_t tickstart;

    /* Включить HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON_Msk);

    tickstart = systick;
    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
        if (systick - tickstart > RCC_HSERDY_TIMEOUT) {
            error();
        }
    }

    /* Включить CSS HSE */
    SET_BIT(RCC->CR, RCC_CR_CSSON_Msk);

    /* Выключить PLL перед настройкой */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON_Msk);

    /*
     * Источник тактирования PLL = HSE (8MHz)
     * Делитель PLL HSE = /1 (8MHz / 1 = 8MHz)
     * Множитель PLL = x9 (8MHz * 9 = 72MHz)
     */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_PLLXTPRE_Msk
             | RCC_CFGR_PLLMULL_Msk,
               RCC_CFGR_PLLSRC_Msk
             | 0x07 << RCC_CFGR_PLLMULL_Pos);

    /* Включить PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON_Msk);
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY_Msk)) {
        continue;
    }

    /*
     * Делитель AHB = /1 (72MHz / 1 = 72MHz)
     * Делитель APB1 = /2 (72MHz / 2 = 72MHz)
     * Делитель APB2 = /1 (72MHz / 1 = 72MHz)
     * Делитель ADC = /6 (72MHz / 6 = 12MHz)
     * Делитель USB = /1.5 (72MHz / 1.5 = 48MHz)
     */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_HPRE_Msk
             | RCC_CFGR_PPRE1_Msk
             | RCC_CFGR_PPRE2_Msk
             | RCC_CFGR_ADCPRE_Msk
             | RCC_CFGR_USBPRE_Msk,
               0x04 << RCC_CFGR_PPRE1_Pos
             | 0x02 << RCC_CFGR_ADCPRE_Pos);

    /* Источник тактирования CPU = PLL */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               0x02 << RCC_CFGR_SW_Pos);
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) !=
            0x02 << RCC_CFGR_SWS_Pos) {
        continue;
    }
}
/* ------------------------------------------------------------------------- */
