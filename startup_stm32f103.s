.syntax unified
.cpu cortex-m3
.thumb
.global Reset_Handler
.global __initial_sp
.global __Vectors
__initial_sp = 0x20005000
.section .isr_vector
__Vectors:
.word __initial_sp
.word Reset_Handler
.text
.thumb_func
Reset_Handler:
    bl main
    b .
