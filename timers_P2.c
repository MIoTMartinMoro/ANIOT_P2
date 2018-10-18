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

static process_event_t process_event_p1a2;
static process_event_t process_event_p2a1;

PROCESS(process1, "Event Timer deadline 4 segs");
PROCESS(process2, "Callback Timer deadline 5 segs");
AUTOSTART_PROCESSES(&process1, &process2);

static int contador1;
static int contador2;

static struct ctimer timer_ctimer;
static struct etimer timer_etimer;

void
do_timeout1(process_event_t event)
{
  if(event==process_event_p2a1){
    contador1++;
    printf("--Proceso1 ha recibido: %d eventos del proceso 2\n", contador1);
  }else if(event==PROCESS_EVENT_TIMER){
    printf("ETIMER Timeout1 envia evento al proceso 2\n");
    process_post_synch(&process2, process_event_p1a2, NULL);
    etimer_reset(&timer_etimer);  
  }
}
/*---------------------------------------------------------------------------*/
void
do_timeout2()
{
  printf("CTIMER Timeout2 envia evento al proceso 1 \n");
  process_post(&process1, process_event_p2a1, NULL);
  ctimer_reset(&timer_ctimer);
}
void
do_timeout3(process_event_t event)
{
  if(event==process_event_p1a2){
    contador2++;
    printf("--Proceso2 ha recibido: %d eventos del proceso 1\n", contador2);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(process1, ev, data)
{
  

  PROCESS_BEGIN();

  process_event_p1a2 = process_alloc_event();

  while(1) {
    etimer_set(&timer_etimer, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev==process_event_p2a1 || ev== PROCESS_EVENT_TIMER);
    do_timeout1(ev);
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(process2, ev, data)
{
  PROCESS_BEGIN();

  process_event_p2a1 = process_alloc_event();

  while(1) {
    ctimer_set(&timer_ctimer, 5 * CLOCK_SECOND, do_timeout2, NULL);
    PROCESS_WAIT_EVENT_UNTIL(ev==process_event_p1a2); 
    do_timeout3(ev);
    PROCESS_YIELD();
  }

  PROCESS_END();
}

