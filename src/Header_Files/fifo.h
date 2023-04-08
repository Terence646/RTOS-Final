#include "os.h"
#include "stdio.h"
#include "stdlib.h"
#include "cmu.h"

#ifndef SRC_HEADER_FILES_FIFO_H_
#define SRC_HEADER_FILES_FIFO_H_



struct node{

  uint32_t button_state; // 0: none pressed, 1: b0 pressed, 2: b1 pressed, 3: both pressed
  struct node * next;

};


void write_fifo(uint32_t button_state);
uint32_t del_node();
uint32_t read_fifo();

#endif /* SRC_HEADER_FILES_FIFO_H_ */
