/* https://gist.github.com/ridencww/4e5d10097fee0b0f7f6b
 * Simple printf for writing to an Arduino serial ports eg. Serial..Serial3. Note: format
 * specifier must match actual argument type eg. int32 should be formatted as %l, not %d.
 * Usage: Serial_printf(Serial,"%d\n",vars);
 * %B  binary (d = 0b1000001)  %b  binary (d = 1000001)      %c  character (s = H)
 * %d/%i  integer (d = 65)     %o  boolean on/off (d = On)   %%  escaped percent ("%")
 * %f  float (f = 123.45)      %.3f/%3f  float (f = 123.346) %l  long (d = 65)
 * %s  char* string (s = xyz)  %X  hexadecimal (d = 0x41)    %x  hexadecimal (d = 41)
 * For more float formats, dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
      eg. dtostrf(fpvalue,8,3,fpstr);
 * HardwareSerial Arduino Nano,Uno,Pro Mini,Serial1,...
 * Serial_ Arduino 32U4 Micro,Pro Micro,Leonardo,...
 * USBSerial STM32Duino
*/
template<typename PORT_t,typename... Args>
void Serial_printf(PORT_t& serial, Args... args) {
  Serial_printf_helper(serial, args...);
}

template<typename PORT1_t>
void Serial_printf_main(PORT1_t& serial, const char* fmt, va_list argv) {
    for (int i = 0; fmt[i] != '\0'; i++) {
        if (fmt[i] == '%') {
            // Look for specification of number of decimal places
            int places = 2;
            if (fmt[i+1] == '.') i++;
            if (fmt[i+1] >= '0' && fmt[i+1] <= '9') {
                places = fmt[i+1] - '0';
                i++;
            }
            
            switch (fmt[++i]) {
                case 'B':
                    serial.print("0b"); // Fall through intended
                case 'b':
                    serial.print(va_arg(argv, int), BIN);
                    break;
                case 'c': 
                    serial.print((char) va_arg(argv, int));
                    break;
                case 'd': 
                case 'i':
                    serial.print(va_arg(argv, int), DEC);
                    break;
                case 'f': 
                    serial.print(va_arg(argv, double), places);
                    break;
                case 'l': 
                    if (fmt[i+1] == 'x' || fmt[i+1] == 'X') {
                      serial.print(va_arg(argv, long), HEX);
                      i++;
                    }
                    else
                      serial.print(va_arg(argv, long), DEC);
                    break;
                case 'o':
                    serial.print(va_arg(argv, int) == 0 ? "off" : "on");
                    break;
                case 's': 
                    serial.print(va_arg(argv, const char*));
                    break;
                case 'X':
                    serial.print("0x"); // Fall through intended
                case 'x':
                    serial.print(va_arg(argv, int), HEX);
                    break;
                case '%': 
                    serial.print(fmt[i]);
                    break;
                default:
                    serial.print("?");
                    break;
            }
        } else {
            serial.print(fmt[i]);
        }
    }
}

template<typename PORT2_t>
void Serial_printf_helper(PORT2_t& serial, const char* fmt, ...) { 
    va_list argv;
    va_start(argv, fmt);
    Serial_printf_main(serial,fmt,argv);
    va_end(argv);
}

unsigned int FSHlength(const __FlashStringHelper * FSHinput) {
  PGM_P FSHinputPointer = reinterpret_cast<PGM_P>(FSHinput);
  unsigned int stringLength = 0;
  while (pgm_read_byte(FSHinputPointer++)) {
    stringLength++;
  }
  return stringLength;
}

template<typename PORT3_t>
void Serial_printf_helper(PORT3_t& serial, const __FlashStringHelper* fstr, ...) { 
    int buflen=FSHlength(fstr);
    char fmt[buflen+1];
    memset(fmt, '\0', sizeof(fmt));
    strncpy_P(fmt, (const char PROGMEM *)fstr,buflen);

    va_list argv;
    va_start(argv,fmt);
    Serial_printf_main(serial,fmt,argv);
    va_end(argv);
}