LDFLAGS=
LIBS= `pkg-config --libs opencv`
CFLAGS= -O0 -g `pkg-config --cflags opencv`

all: myMarkers

myMarkers: myMarkers.o
	g++ -o myMarkers myMarkers.o $(LDFLAGS) $(LIBS)

myMarkers.o: myMarkers.cpp
	g++ -c $(CFLAGS) myMarkers.cpp

clean:
	rm -f *.o
	rm -f myMarkers
