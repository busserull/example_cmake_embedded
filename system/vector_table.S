.syntax unified
@ .arch armv7+nofp

.section VectorTable, "ax"
.align 2
.global _vector_table

_vector_table:
    .long _stack_top
    .long Reset_Handler
