INSTDIR   = $(prefix)/usr/bin
INSTMODE  = 0755
INSTOWNER = root
INSTGROUP = root

all: test_audio

test_audio: test_audio.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@ -lasound
	$(STRIP) $@

%.o: %.c
	$(CC) -c $(CFLAGS) $^ -o $@ -lasound

install: test_audio
	$(INSTALL) -d $(INSTDIR)
	$(INSTALL) -m $(INSTMODE) -o $(INSTOWNER) -g $(INSTGROUP) test_audio $(INSTDIR)

clean:
	rm -f test_audio *.o core

