# UBA_CESE_PCeSE
Repositorio del curso de Protocolos Comunicación en Sistemas Embebidos de la CESE-UBA

Trabajo final Integrador PCeSE

Alumno Bach. Carlos Herrera de la 17Cohorte del CESE

Se implementa el desarrollo de un driver genérico para el IC CODEC WM8960, el cual utiliza un protocolo serial 2Wire (I2C) 
para la configuración y el protocolo I2S para la transmisión de audio digital desde una variable de audio a traves de la 
placa NUCLEO F429.

Las funciones principales disponibles es la inicialización del CODEC y la reproducción de música.

La localización del driver se encuentra en la ruta "Driver / WM8960"

  -WM8960_InitDriver(estructuraAudio)<- Requiere una estructura de datos como parámetro de entrada del tipo "wm8960_t"
                                        Su función es la de configurar los registros respectivos para la reproducción de audio (DAC) 
										en el CODEC como, el reloj para la frecuencia de muestreo, ganancia de las lineas de salida, 
										interfaz de audio (word length & interface), ganancia del DAC, habilitación de salidas, 
										función mixer , detección de jack, configuración de controles adicionales.
                                        

  -WM8960_audioPlayVar(vector de audio, tamaño del vector)<-	Utiliza una maquina de estados interna para la transmisión por bloques los datos
																almacenados en el vector de audio, es llamado de forma manual mediante el establecimiento
																del flag  "pr_start" a fin de establecer el inicio de la transmisión desde la posición 0 del vector
																o mediante interrupciones por finalización de transmisión I2C, en cuyo caso se realiza el 
																recorriendo desde la posición inicial hasta el último elemento del vector por bloques de tamaño máximo de
																65535 elementos
																
Funcionamiento actual del ejemplo

Mediante el uso de un vector de tipo uint16_t se reliza:
1) La reproducción del mismo utilizando la función WM8960_audioPlayVar para la transmisión de datos
2) Las funciones WM8960_setStatePlayer y WM8960_getStatePlayer para la configuración y lectura del estado de la FSM
3) La función WM8960_setIRQplay es llamado por interrupción cada vez que se termina de realizar una transmisión I2S
4) Por último la función WM8960_setIRQpause permite pausar y resumir en el momento de transmisión de datos.

Se deberá poder escuchar el sonido de la canción y a su vez mediante el boton USER (azul) de la placa NUCLEO F429 permitir realizar una pausa en plena reproducción y la 
continuación de la salida de sonido. El programa de ejemplo actual se encuentra en un bucle infinito con una pausa de 2 segundos entre el final e inicio de la reproducción.
