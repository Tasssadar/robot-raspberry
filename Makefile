program=kvetinac_pi
OBJ=comm.o main.o packet.o tcpserver.o camera.o util.o
OPT=-Os -lrt -pthread -lopencv_highgui -lopencv_core -lopencv_imgproc
CXX?=g++
C?=gcc

ifeq ($(NORPI), true)
	OPT += -DNORPI
else
	OPT += -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util
endif

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
