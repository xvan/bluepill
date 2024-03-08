#include "cdc_console.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "embedded_cli.h"

/* CLI */
typedef struct {
  uint8_t * data;
  size_t buffer_len;
  size_t cursor;
} linear_buffer_object_t;


void init_linear_buffer(linear_buffer_object_t *buffer_object, uint8_t *data, size_t len);
bool linear_buffer_putchar(linear_buffer_object_t * buffer_object, char ch);
char getch(void);
void buffer_putch(void *, char, bool);
void print_buffer(linear_buffer_object_t *);
void cdc_error_handler(void);
void cli_puts(char* str);

/*Globals*/
struct embedded_cli cli;
linear_buffer_object_t buffer_object;
uint8_t buffer[1<<5] = {0};

/*Input Buffer Methods*/
void init_linear_buffer(linear_buffer_object_t *buffer_object, uint8_t *data, size_t len){
  buffer_object->data = data;
  buffer_object->buffer_len = len;
  buffer_object->cursor = 0;
}

bool linear_buffer_putchar(linear_buffer_object_t * buffer_object, char ch){
  if (buffer_object->cursor >= buffer_object->buffer_len){
    return false;
  }
  buffer_object->data[buffer_object->cursor++] = ch;
  return true;
}

/* Public Interface*/
   
void cdc_console_init(){
  MX_USB_DEVICE_Init();
  HAL_Delay(500);
  init_linear_buffer(&buffer_object, buffer, sizeof(buffer));
  embedded_cli_init(&cli, "POSIX> ", buffer_putch, &buffer_object);  
}

int cdc_console_parse(int (*parse_command)(int argc, char **argv, void (* cli_print)(char * str))){
    bool done = false;
    embedded_cli_prompt(&cli);

    while (!done) {      
      char ch = getch();

      /**
       * If we have entered a command, try and process it
       */
      if (embedded_cli_insert_char(&cli, ch)) {
          int cli_argc;
          char **cli_argv;
          cli_argc = embedded_cli_argc(&cli, &cli_argv);

          int ret_val = parse_command(cli_argc, cli_argv, cli_puts);          
          return ret_val;
      }
  }
}


char getch(void)
{
  uint8_t buf;
  while (CDC_GetRxBufferBytesAvailable_FS() == 0);  
  if (CDC_ReadRxBuffer_FS(&buf, 1) != USB_CDC_RX_BUFFER_OK){
    cdc_error_handler();
  }
  return buf;  
}

void cdc_error_handler(){
  while (1);
}


void print_buffer(linear_buffer_object_t * buffer_object){
  while (CDC_Transmit_FS(buffer_object->data, buffer_object->cursor) == USBD_BUSY);
  buffer_object->cursor = 0;
}

void buffer_putch(void *data, char ch, bool is_last)
{
    linear_buffer_object_t * buffer_object = (linear_buffer_object_t *)data;
    if(! linear_buffer_putchar(buffer_object, ch)){
      print_buffer(buffer_object);
      linear_buffer_putchar(buffer_object, ch);
    }
    if (is_last){
      print_buffer(buffer_object);
    }        
}

void cli_puts(char* str){
  while (CDC_Transmit_FS(str, strlen(str)) == USBD_BUSY);
}