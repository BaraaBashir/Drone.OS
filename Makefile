CC = gcc
CFLAGS = -Wall -std=c99 -I./raylib-5.0_linux_amd64/include
LDFLAGS = -L./raylib-5.0_linux_amd64/lib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -Wl,-rpath,./raylib-5.0_linux_amd64/lib

milestone1:
	$(CC) $(CFLAGS) dijkstra.c -o dijkstra $(LDFLAGS)

milestone2:
	$(CC) $(CFLAGS) milestone2.c -o sim $(LDFLAGS)

milestone3:
	$(CC) $(CFLAGS) sim.c -o sim $(LDFLAGS)

clean:
	rm -f dijkstra sim
