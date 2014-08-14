INSTDIR   = $(prefix)/usr/bin
INSTMODE  = 0755
INSTOWNER = root
INSTGROUP = root

all: gsc_update

gsc_update: gsc_update.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@
	$(STRIP) $@

%.o: %.c
	$(CC) -c $(CFLAGS) $^ -o $@

install: gsc_update
	$(INSTALL) -d $(INSTDIR)
	$(INSTALL) -m $(INSTMODE) -o $(INSTOWNER) -g $(INSTGROUP) gsc_update $(INSTDIR)

clean:
	rm -f gsc_update *.o core

