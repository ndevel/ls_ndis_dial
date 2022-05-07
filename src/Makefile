all:ls_ndis_dial
 
OBJS = ls_ndis_dial.o
 
Serial:$(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -lpthread
 
clean:
	rm -f ls_ndis_dial *.o
