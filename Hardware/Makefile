
CXXFLAGS=-I. -I.. -g -fpic -O2 -Wall -Wextra -pedantic -Werror -std=c++11
CFLAGSZLIB=-g -fpic -O2 -Wno-implicit-function-declaration
LDFLAGS=-g -shared -Wl,-soname,

LIBS= \
	CS2Tcp.so \
	CS2Udp.so \
	CcSchnitte.so \
	Ecos.so \
	Hsi88.so \
	M6051.so \
	OpenDcc.so \
	RM485.so \
	Virtual.so \
	Z21.so

OBJZLIB=zlib/adler32.o \
	zlib/compress.o \
	zlib/crc32.o \
	zlib/deflate.o \
	zlib/gzclose.o \
	zlib/gzlib.o \
	zlib/gzread.o \
	zlib/gzwrite.o \
	zlib/infback.o \
	zlib/inffast.o \
	zlib/inflate.o \
	zlib/inftrees.o \
	zlib/trees.o \
	zlib/uncompr.o \
	zlib/zutil.o \
	ZLib.o

all: $(LIBS)

zlib: $(OBJZLIB)

amalgamation: zlib

zlib/%.o: zlib/%.c zlib/*.h
	$(CC) $(CFLAGSZLIB) -c -o $@ $<

CS2Tcp.so: CS2Tcp.o ProtocolMaerklinCAN.o $(OBJZLIB) *.h ../Manager.h ../Languages.h
	$(CXX) $(CXXFLAGS) $(LDFLAGS)$@ -o $@ ProtocolMaerklinCAN.o $(OBJZLIB) $<

CS2Udp.so: CS2Udp.o ProtocolMaerklinCAN.o ZLib.o *.h ../Manager.h ../Languages.h
	$(CXX) $(CXXFLAGS) $(LDFLAGS)$@ -o $@ ProtocolMaerklinCAN.o $(OBJZLIB) $<

CcSchnitte.so: CcSchnitte.o ProtocolMaerklinCAN.o ZLib.o *.h ../Manager.h ../Languages.h
	$(CXX) $(CXXFLAGS) $(LDFLAGS)$@ -o $@ ProtocolMaerklinCAN.o $(OBJZLIB) $<

%.so: %.o *.h ../Manager.h ../Languages.h
	$(CXX) $(CXXFLAGS) $(LDFLAGS)$@ -o $@ $<

%.o: %.cpp *.h ../*.h ../Logger/*.h ../Languages.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.so *.o zlib/*.o

