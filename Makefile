program=kvetinac_pi
OBJ=comm.o main.o packet.o tcpserver.o camera.o util.o
CXX?=g++
C?=gcc
OPT :=

ifeq ($(NORPI), true)
	OPT += -DNORPI
else

ifneq ($(RPI_CROSS),)
	CXX = $(RPI_CROSS)/bin/g++
	C = $(RPI_CROSS)/bin/gcc

	OPT += -Wl,-rpath,$(RPI_CROSS)/lib/gcc
	OPT += -Wl,-rpath,$(RPI_CROSS)/lib/gcc/arm-linux-gnueabihf
	OPT += -I$(RPI_CROSS)/chain/include/arm-linux-gnueabihf
	OPT += -lstdc++
endif

	OPT += -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util
endif

OPT += -Os -lrt -pthread -lopencv_highgui -lopencv_core -lopencv_imgproc
OPT += $(CXXFLAGS)

.PHONY: build
.PHONY: clean
.PHONY: deploy

build: ${program}

clean:
	rm -f *.o ${program}

deploy:
	rsync -rvc --exclude=*.o --exclude=*~ --exclude=kvetinac_pi ../raspberry pi@192.168.10.2:/home/pi/
	ssh pi@192.168.10.2 "cd /home/pi/raspberry/ && make"

${program}: ${OBJ}
	${CXX} ${OBJ} -o ${program} ${OPT}

packet.o: packet.cpp
	${CXX} ${OPT} -c  $<

tcpserver.o: tcpserver.cpp
	${CXX} ${OPT} -c  $<

camera.o: camera.cpp
	${CXX} ${OPT} -c  $<

main.o: main.cpp
	${CXX} ${OPT} -c  $<

comm.o: comm.cpp
	${CXX} ${OPT} -c  $<

util.o: util.cpp
	${CXX} ${OPT} -c  $<
