CFLAGS = -W -Wall -O2
LDLIBS = -lglut -lGLU -lGL -lm

qrsim: 		qrsim.o qrdisp.o qrmod.o qrcont.o
		gcc -o qrsim $(CFLAGS) qrsim.o qrdisp.o qrmod.o qrcont.o $(LDLIBS)

qrsim.o: 	qrsim.h qrmod.h qrdisp.h qrcont.h qrsim.c
		gcc -c $(CFLAGS) qrsim.c

qrdisp.o:	qrsim.h qrdisp.c
		gcc -c $(CFLAGS) qrdisp.c

qrmod.o: 	qrsim.h qrmod.c
		gcc -c $(CFLAGS) qrmod.c

qrcont.o: 	qrsim.h qrcont.c
		gcc -c $(CFLAGS) qrcont.c

clean:
		$(RM) *.o *~ qrsim
