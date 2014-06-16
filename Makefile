program=kvetinac_pi
OBJ=main.o packet.o tcpserver.o camera.o
OPT=-Os -lrt -pthread -lopencv_highgui -lopencv_core -lopencv_imgproc

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
	g++ ${OBJ} -o ${program} ${OPT}

comm.o: comm.cpp
	g++ ${OPT} -c  $<

packet.o: packet.cpp
	g++ ${OPT} -c  $<

tcpserver.o: tcpserver.cpp
	g++ ${OPT} -c  $<

camera.o: camera.cpp
	g++ ${OPT} -c  $<

main.o: main.cpp
	g++ ${OPT} -c  $<
