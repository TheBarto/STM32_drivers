/* Definimos el punto de entrada. No es obligatorio, pero para el debug es necesario */
ENTRY(Reset_Handler)

/* !CONSULTAR EL DATASHEET DE LA PLACA¡, Y ENTONCES IR A LA SECCION DE MAPEO DE MEMORIA Y 
VER COMO ESTA DISTRIBUIDA LA MEMORIA.*/
MEMORY
{
	FLASH(rx): ORIGIN = 0x08000000, LENGTH = 1024K
	SRAM(rwx): ORIGIN = 0x20000000, LENGTH = 128K
}

/* Marcamos, dentro de cada memoria, las diferentes secciones.
 * ESTAS SECCIONES SERAN DESPUES LAS QUE SE MUESTREN EN LA
 * CABECERA DEL FICHERO .elf QUE GENEREMOS. ESTAS SECCIONES
 * AGLUTINARAN LAS DIFERENTES SECCIONES QUE SE MUESTRAN EN LOS
 * DIAGRAMAS DE LOS MANUALES DE REFERENCIA, YA QUE ALGUNAS SECCIONES
 * MOSTRADAS EN ESTOS SITIOS SE LLAMAN DE LA MISMA MANERA QUE LAS
 * ESCRITAS AQUI.*/
SECTIONS
{
/* Fusionamos todas las diferentes secciones de los .o que conforman nuestro programa, y las fijamos aquí,
   poniendo según nos sea conveniente los elementos en el orden y memoria adecuados. */

/* Tambien ponemos simbolos para que el compilador los procese, y podamos emplearlos en los ficheros .c.
   Concretamente en el fichero de arranque, y poder gestionar las diferentes secciones. */
  .text :
  {
    _stext = .; /* Este valor sería el del inicio de la FLASH, que hemos marcado anteriormente como 0x08000000 */
    *(.isr_vector)
    *(.text)
    *(.text.*)
    *(.init)
    *(.fini)
    *(.ARM.exidx)
    *(.rodata)
    *(.eh_frame)
    _etext = .;
  } > FLASH

  _la_data = LOADADDR(.data);
  .data :
  {
    _sdata = .;
    *(.data)
    *(.data.*)
    *(.init_array)
    *(.fini_array)
    _edata = .;
  } > SRAM AT> FLASH

  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss.*)
    _ebss = .;
    __bss_end__ = _ebss;
    end = .;
  } > SRAM
}
