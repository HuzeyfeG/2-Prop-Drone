#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Bluepad32.h>


ControllerPtr myControllers[BP32_MAX_GAMEPADS];
const uint8_t* addr = BP32.localBdAddress();

RF24 radio(4, 5); // CE, CSN
uint8_t address[] = "ykrc1";
uint8_t data_to_send[4] = {0, 0, 0, 0};


void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      ControllerProperties properties = ctl->getProperties();
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
}

uint8_t mapper(int axis_value, uint8_t min_value, uint8_t max_value) {
  if (axis_value < 50 && axis_value > -50) {
    axis_value = 0;
  } 
  if (axis_value < -500) {
    axis_value = -500;
  } 
  if (axis_value > 500) {
    axis_value = 500;
  }
  return map(axis_value, -500, 500, min_value, max_value);
}

void dumpGamepad(ControllerPtr ctl) {
  data_to_send[0] = mapper(ctl->axisX(), 0, 40);
  data_to_send[1] = mapper(ctl->axisY(), 100, 0);
  data_to_send[2] = mapper(ctl->axisRX(), 0, 40);
  data_to_send[3] = mapper(ctl->axisRY(), 40, 0);
}

void processGamepad(ControllerPtr ctl) {
  if (ctl->a()) {
    static int colorIdx = 0;
    switch (colorIdx % 3) {
      case 0:
        ctl->setColorLED(255, 0, 0);
        break;
      case 1:
        ctl->setColorLED(0, 255, 0);
        break;
      case 2:
        ctl->setColorLED(0, 0, 255);
        break;
    }
    colorIdx++;
  }

  if (ctl->b()) {
    static int led = 0;
    led++;
    ctl->setPlayerLEDs(led & 0x0f);
  }

  if (ctl->x()) {
    ctl->playDualRumble(0, 250, 0x80, 0x40);
  }

  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

void ps4_init() {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}


void nrf24_init() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPayloadSize(10);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(46);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(false);
  radio.stopListening();
}


void setup() {
  //  PS4 init.
  ps4_init();

  // nRF24L01 init.
  nrf24_init();
}


void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
    radio.write(&data_to_send, sizeof(data_to_send));
  }
  delay(1);
}
