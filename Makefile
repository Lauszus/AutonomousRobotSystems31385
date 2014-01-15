#
# The Compiler
#
CC      = gcc
LD      = ${CC}
SMR     = /usr/local/smr
CFLAGS  = -Wall -O2 -I${SMR}/include
LDFLAGS = -L${SMR}/lib

#
# Our program files
#
PROG   = main
HDRS   = control.h constants.h
OBJS   = ${PROG}.o control.o
LIBS   = -lm /usr/local/smr/lib/librhd.a

all:	${PROG}

run:	all
	./${PROG}

${PROG}: ${OBJS}
	${LD} -o $@ ${LDFLAGS} ${OBJS} ${LIBS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
