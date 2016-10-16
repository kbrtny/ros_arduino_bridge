#ifndef __I2C_H__
#define __I2C_H__

#define USE_I2C

/* pI2C.h - an implementation of the Pololu I2C protocol
 * for the ROSArduinoBridge.  Based on AStarRPiSlaveDemo.ino.
 *
 * TODO:
 * > customize struct Data for ROSArduinoBridge.
 * > remove __I2C_H__ header crap (break into *.h *.cpp?)
 *
 *
 * Jay Salmonson
 * 10/16/2016
 */

#include <Servo.h>
#include <AStar32U4.h>
#include <PololuRPiSlave.h>

struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;

  int16_t leftMotor, rightMotor;
  uint16_t batteryMillivolts;
  uint16_t analog[6];

  bool playNotes;
  char notes[14];
};

PololuRPiSlave<struct Data,0> slave;
PololuBuzzer buzzer;
AStar32U4Motors motors;
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;

void initI2c() {
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");
}

void runI2c() {
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivoltsSV();

  for(uint8_t i=0; i<6; i++)
  {
    slave.buffer.analog[i] = analogRead(i);
  }

  // READING the buffer is allowed before or after finalizeWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);
  motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);

  // Playing music involves both reading and writing, since we only
  // want to do it once.
  if(slave.buffer.playNotes)
  {
    buzzer.play(slave.buffer.notes);
    while(buzzer.isPlaying());
    slave.buffer.playNotes = false;
  }

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();
}

#endif
