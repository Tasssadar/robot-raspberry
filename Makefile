program=kvetinac_pi
OBJ=comm.o main.o packet.o tcpserver.o
OPT=-Os -lrt

.PHONY: build
.PHONY: clean
.PHONY: deploy

build: ${program}

clean:
	rm -f *.o ${program}

deploy:
	rsync -r --exclude=*.o ../raspberry pi@192.168.10.2:/home/pi/
	ssh pi@192.168.10.2 "cd /home/pi/raspberry/ && make"

${program}: ${OBJ}
	g++ ${OBJ} -o ${program} ${OPT}

comm.o: comm.cpp
	g++ ${OPT} -c  $<

packet.o: packet.cpp
	g++ ${OPT} -c  $<

tcpserver.o: tcpserver.cpp
	g++ ${OPT} -c  $<

main.o: main.cpp
	g++ ${OPT} -c  $<
