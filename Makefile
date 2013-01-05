all:
	gcc -Wall -fno-strict-aliasing -fwrapv -Wstrict-prototypes -std=c99 -lm -o pifm pifm.c

clean:
	rm -f pifm *.o
