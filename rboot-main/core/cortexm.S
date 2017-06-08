//if you've got error in this line, than this file is compiled wrong

#include "core.h"
#include "config.h"
#include "board/ST/stm32.h"
#include "stm32_config.h"

/* Define constants used in low-level initialization.  */

  .equ   ICER0,                 0xe000e180
  .equ   ICPR0,                 0xe000e280

/* exported global constant and functions */
    .global delay_ms
    .global delay_us

    .extern main

/* interrupt vectors */
    .section  .isr_vectors, "ax"

int_vectors:
    .word  (SRAM_BASE + SRAM_SIZE)
    .word  Reset_Handler
    .word  NMI_Handler
    .word  HardFault_Handler
#if defined(CORTEX_M3) || defined(CORTEX_M4)
    .word  MemManage_Handler
    .word  BusFault_Handler
    .word  UsageFault_Handler
#else
    .word  0
    .word  0
    .word  0
#endif
    .word  0
    .word  0
    .word  0
    .word  0
    .word  SVC_Handler
#if defined(CORTEX_M3) || defined(CORTEX_M4)
    .word  DebugMon_Handler
#else
    .word  0
#endif
    .word  0
    .word  PendSV_Handler
    .word  SysTick_Handler

    .rept IRQ_VECTORS_COUNT
        .word  irq_handler
    .endr

/*********************** reset vector handler *********************/
    .section  .reset, "ax"
    .syntax unified
#if defined(CORTEX_M3)
    .cpu cortex-m3
#elif defined(CORTEX_M4)
    .cpu cortex-m4
#else
    .cpu cortex-m0
#endif
    .thumb

    .thumb_func
Reset_Handler:
    cpsid i

    @disable all interrupts
    ldr r0, =0xffffffff
    ldr r1, =ICER0
    str r0, [r1]
#if defined(CORTEX_M3) || defined(CORTEX_M4)
    str r0, [r1, #0x4]
    str r0, [r1, #0x8]
    str r0, [r1, #0xc]
    str r0, [r1, #0x10]
    str r0, [r1, #0x14]
    str r0, [r1, #0x18]
    str r0, [r1, #0x1c]
#endif

    @clear all pending interrupts
    ldr r1, =ICPR0
    str r0, [r1]
#if defined(CORTEX_M3) || defined(CORTEX_M4)
    str r0, [r1, #0x4]
    str r0, [r1, #0x8]
    str r0, [r1, #0xc]
    str r0, [r1, #0x10]
    str r0, [r1, #0x14]
    str r0, [r1, #0x18]
    str r0, [r1, #0x1c]
#endif

    bl    system_early_init

    @switch to MSP
    ldr r0, =0x0;
    mrs r0, control
    ldr r0, =SP_PTR
    mov sp, r0

    @go to main
    b       main

    .thumb_func
delay_ms:
    ldr r1, =DELAY_COEF_MS
    muls    r0, r1

loop_ms:
    subs    r0, #1
    bge loop_ms

    bx      lr

    .thumb_func
delay_us:
    ldr r1, =DELAY_COEF_US
    muls    r0, r1

loop_us:
    subs    r0, #1
    bge loop_us

    bx      lr

/*********************** exception vectors handlers *********************/
        .thumb_func
HardFault_Handler:
    b   on_hard_fault

#if defined(CORTEX_M3) || defined(CORTEX_M4)
    .thumb_func
MemManage_Handler:
    b   on_mem_manage

    .thumb_func
BusFault_Handler:
    b   on_bus_fault

    .thumb_func
UsageFault_Handler:
    b   on_usage_fault
#endif

/*********************** weak unhandled vectors *********************/
.macro weak_vector vector
    .weak   \vector
    .thumb_set \vector, default_handler
.endm

weak_vector   NMI_Handler
#if defined(CORTEX_M3) || defined(CORTEX_M4)
weak_vector   DebugMon_Handler
#endif
weak_vector   SVC_Handler
weak_vector   PendSV_Handler
weak_vector   SysTick_Handler
