#ifndef SERVOS_H
#define SERVOS_H


#define N_SERVOS 2

// This delay in milliseconds determines the pause
// between each one degree step the servo travels.  Increasing
// this number will make the servo sweep more slowly.
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
int stepDelay [N_SERVOS] = { 10, 10 }; // ms

// Pins
uint8_t servoPins [N_SERVOS] = { 4, 5};

// Initial Position
uint8_t servoInitPosition [N_SERVOS] = { 90, 90}; // [0, 180] degrees


class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(uint8_t position);
    Servo getServo();
    uint8_t currentPositionDegrees;

  private:
    Servo servo;
    int stepDelayMs;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif
