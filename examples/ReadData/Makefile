SKETCH = ReadData

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

LIBS = ../../..

build: $(SKETCH).ino
	arduino-cli compile  --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

viz:
	./imuviz.py

clean:
	rm -f copilot*
