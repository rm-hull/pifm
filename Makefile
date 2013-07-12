CFLAGS=-Wall -fno-strict-aliasing -fwrapv -Wstrict-prototypes -std=c99 -lm
all: pifm

clean:
	rm -f pifm *.o
