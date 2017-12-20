#include <RobotEQ.h>

#define CHANNEL_1 1

// Configure Motor Controllers
RobotEQ controller(&Serial);

void setup() {
    Serial.begin(115200);
}

void loop() {
    int voltage;
    int amps;

    if (controller.isConnected()) {
        voltage = controller.queryBatteryVoltage();
        amps = controller.queryBatteryAmps();

        controller.commandMotorPower(CHANNEL_1, 1000);
    }
}
