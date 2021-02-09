all:
	g++ Principal.cpp Prueba.cpp -I/usr/local/include/ -L/usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_video -lopencv_videoio -lopencv_objdetect -o ejecutar.bin 
run: 
	./ejecutar.bin
