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

#define PROCESS_EVENT_P1a2  0x8b
#define PROCESS_EVENT_P2a1  0x8c

PROCESS(process1, "Event Timer deadline 4 segs");
PROCESS(process2, "Callback Timer deadline 5 segs");
AUTOSTART_PROCESSES(&process1, &process2);

static int contador1;
static int contador2;

static struct timer timer_timer;
static struct stimer timer_stimer;
static struct ctimer timer_ctimer;
static struct rtimer timer_rtimer;
static struct etimer timer_etimer;
static rtimer_clock_t timeout_rtimer = RTIMER_SECOND / 2;

void
do_timeout1(int event)
{
  if(event==PROCESS_EVENT_P2a1){
    printf("Proceso 1: %d\n", contador1);
    contador1++;
  }else if(event==PROCESS_EVENT_TIMER){
    process_post_synch(&process2, PROCESS_EVENT_P1a2, NULL);
    etimer_reset(&timer_etimer);  
  }
}
/*---------------------------------------------------------------------------*/
void
do_timeout2()
{
    process_post(&process1, PROCESS_EVENT_P2a1, NULL);
    ctimer_reset(&timer_ctimer);
}
void
do_timeout3(int event)
{
  if(event==PROCESS_EVENT_P1a2){
    printf("Proceso 2: %d\n", contador2);
    contador2++;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(process1, ev, data)
{
  

  PROCESS_BEGIN();

  while(1) {
    etimer_set(&timer_etimer, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_P2a1 || ev== PROCESS_EVENT_TIMER);
    do_timeout1(ev);
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(process2, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    ctimer_set(&timer_ctimer, 5 * CLOCK_SECOND, do_timeout2, NULL);
    PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_P1a2); 
    do_timeout3(ev);
  }

  PROCESS_END();
}

