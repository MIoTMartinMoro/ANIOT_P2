/*
 * Copyright (C) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/stimer.h"
#include "sys/timer.h"
#include "sys/rtimer.h"

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>


/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 20)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF

#define CC26XX_DEMO_SENSOR_1     &button_left_sensor
#define CC26XX_DEMO_SENSOR_2     &button_right_sensor

#if BOARD_SENSORTAG
#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     &reed_relay_sensor
#endif


static process_event_t process_event_p1a2;
static process_event_t evento_imprime;

PROCESS(process1, "Lee cada 5 segundos el sensor MPU");
PROCESS(process2, "Callback Timer deadline 5 segs");
AUTOSTART_PROCESSES(&process1, &process2);

static int contador1=0;
static int contador2=0;

static struct ctimer timer_ctimer;
static struct etimer timer_etimer;

#if BOARD_SENSORTAG

#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)

/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}
/*---------------------------------------------------------------------------*/
static void
get_mpu_reading(int valores[])
{

printf("Entra a leer\n");
  valores[0] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  valores[1] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  valores[2] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  // Creo que solo nos interesan los tres de  abajo
  valores[3] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  valores[4] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  valores[5] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  
  printf("Valores[0]: %d\n", valores[0]);
  printf("Valores[1]: %d\n", valores[1]);
  printf("Valores[2]: %d\n", valores[2]);
  printf("Valores[3]: %d\n", valores[3]);
  printf("Valores[4]: %d\n", valores[4]);
  printf("Valores[5]: %d\n", valores[5]);

}
/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}
#endif

void
send_event_read()
{
  printf("Entra al evento\n");
  int valores[6];
  get_mpu_reading(valores);
  process_post(&process2, evento_imprime, valores);
  ctimer_reset(&timer_ctimer);
}
/*---------------------------------------------------------------------------*/
void
do_timeout2()
{
  process_post(&process1, evento_imprime, NULL);
  ctimer_reset(&timer_ctimer);
}
void
do_timeout3(process_event_t event)
{
  if(event==evento_imprime){
    printf("Proceso2 ha recibido: %d\n", contador2);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(process1, ev, data)
{
  

  PROCESS_BEGIN();
  init_mpu_reading(NULL);
  evento_imprime = process_alloc_event();
  ctimer_set(&timer_ctimer, 5 * CLOCK_SECOND, send_event_read, NULL);

  PROCESS_END();

  SENSORS_DEACTIVATE(mpu_9250_sensor);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(process2, ev, data)
{
  PROCESS_BEGIN();

  if (ev == evento_imprime) {
    int i;
    /*for (i = 0; i < sizeof(data); i++) {
      printf("%d\n", data[i]);
    }*/
  }

  PROCESS_END();
}
