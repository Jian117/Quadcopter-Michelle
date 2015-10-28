#ifndef INT_H
#define INT_H

#include "routines.h"

void interrupt_init(void);
void global_interrupt(void);
void myISR(void);

#endif