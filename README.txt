Note When Installing Arduino:
============================

In the library file hardware\libraries\wire\utility\twi.c, function twi_init(),
#ifdef out the lines that activate the pullup resistors.  Then delete twi.o to
make sure it compiles.