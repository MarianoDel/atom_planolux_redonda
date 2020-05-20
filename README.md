# Kirno Placa Redonda - Control de Luces y Alarmas
Parte alta control redondo
micro STM32F051C8T6

Antes de empezar revisar seleccion del micro y configuracion
------------------------------------------------------------

* work area size y flash image
>stm32f0_flash.cfg

* work area size y flash image
>stm32f0_flash_lock.cfg

* work area size
>stm32f0_gdb.cfg

* work area size
>stm32f0_reset.cfg

* end of ram; stack size; memory lenght
>./cmsis_boot/startup/stm32_flash.ld

* elegir procesador definiciones en linea 68 -> pero modificar en Makefile
>./cmsis_boot/stm32f0xx.h
>.Makefile








