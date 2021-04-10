#include "PID.h"
#include "Chrono.h"
#include <Arduino.h>
#include "Data.h"
Chrono PIDTimer;

PID::PID()
{
  firstCompute = true;
  setSetpoint(0.0f);
}

void PID::setSetpoint(float setPt)
{
  Setpoint = setPt;
}

void PID::setInput(float inpt)
{
  Input = inpt;
}

float PID::getPError()
{
  return error;
}

float PID::getDError()
{
  return dErr;
}

float PID::getIError()
{
  return ITerm;
}

float PID::getOutput()
{
  return Output;
}

void PID::compute()
{

  if (firstCompute)
  {
    firstCompute = false;
    lastTime = micros();
    lastError = 0.0;
  }
  else
  {
    unsigned long now = micros();
    deltaT = float((now - lastTime)) / 1000000.0f;
    error = Setpoint - Input;

    ITerm += (ki * error) * deltaT;

    if (lastError < 0.0 && error > 0.0)
    {
      ITerm = 0.0;
    }

    if (lastError > 0.0 && error < 0.0)
    {
      ITerm = 0.0;
    }

    if (error == 0.0)
    {
      ITerm = 0.0;
    }

    if (ITerm > outMax)
      ITerm = outMax;
    else if (ITerm < outMin)
      ITerm = outMin;
    dErr = (error - lastError) / deltaT;

    Output = kp * error + ITerm + kd * dErr;
    if (Output > outMax)
    {
      Output = outMax;
    }
    if (Output < outMin)
    {
      Output = outMin;
    }

    lastError = error;
    lastTime = now;
  }
  return;
}

void PID::setTunings(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void PID::setOutputLimits(int Min, int Max)
{
  if (Min > Max)
    return;
  outMin = Min;
  outMax = Max;
}

void PID::incrementKP(float num)
{
  float val = kp + num;
  if (val >= 0)
  {
    kp = val;
  }
  else
  {
    kp = 0.0f;
  }
}

void PID::incrementKI(float num)
{
  float val = ki + num;
  if (val >= 0)
  {
    ki = val;
  }
  else
  {
    ki = 0.0f;
  }
}
void PID::incrementKD(float num)
{
  float val = kd + num;
  if (val >= 0)
  {
    kd = val;
  }
  else
  {
    kd = 0.0f;
  }
}
