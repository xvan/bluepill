#ifndef CDC_CONSOLE_H
#define CDC_CONSOLE_H

  void cdc_console_init(void);    
  int cdc_console_parse(int (*parse_command)(int argc, char **argv, void (* cli_print)(const char * str)));

#endif // CDC_CONSOLE_H