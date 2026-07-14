#ifndef OPENOPL1000_LIBC_H
#define OPENOPL1000_LIBC_H

double OpenOPL1000_ParseDouble(const char *text, char **endptr);
int OpenOPL1000_Scan(const char *text, const char *format, ...);

#endif /* OPENOPL1000_LIBC_H */
