#include <RobotEQ.h>

#define CHANNEL_1 1

// Configure Motor Controllers
RobotEQ controller(&Serial3);

void setup() {
    Serial3.begin(115200);
}

void loop() {
    int voltage;
    int amps;

    if (controller.isConnected()) {
        voltage = controller.queryBatteryVoltage();
        amps = controller.queryBatteryAmps();

        controller.commandMotorPower(CHANNEL_1, 500);
    }
}
