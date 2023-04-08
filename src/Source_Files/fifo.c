
#include "fifo.h"



static struct node * fifo_head;

 void write_fifo(uint32_t buttons)
{
  struct node * new_node = (struct node *)malloc(sizeof(struct node));
  new_node->button_state = buttons;
  new_node->next = NULL;
  if(fifo_head == NULL)
    {
      fifo_head = new_node;
    }
  else
    {
      struct node * crawler = fifo_head;
      while(crawler->next != NULL)
      {
        crawler = crawler->next;
      }
      crawler->next = new_node;
    }
}


 uint32_t del_node()
{
   if (fifo_head == NULL)
     {
       return 0;
     }
   else
     {
        struct node * temp = fifo_head;
        uint32_t data = temp->button_state;
        free(fifo_head);
        fifo_head = temp->next;
        return data;
     }

}

 uint32_t read_fifo()
 {
   uint32_t state = fifo_head->button_state;
   del_node();
   return state;
 }

