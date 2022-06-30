#ifndef HELPER_H
#define HELPER_H

#include <Arduino.h>
#include <ArduinoJson.h>

struct velocity
{
  float wl = 0.0;
  float wr = 0.0;
};

struct pid_param
{
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
};

struct pwm
{
  int l = 0;
  int r = 0;
};

DynamicJsonDocument input(150);
DynamicJsonDocument output(150);

// Functions
String readMsg()
{
  String result = "";
  if (Serial.available() > 0)
  {
    result = Serial.readStringUntil('\n');
  }
  return result;
}

// {"status":"OK"}
void waitStartCommand(String name)
{
  while (1)
  {
    String res = "";
    output["name"] = "mobile";
    serializeJson(output, res);
    Serial.println(res);
    delay(10);

    if (Serial.available() > 0)
    {
      deserializeJson(input, readMsg());

      if (!input.isNull())
      {
        if (input["status"] == "OK")
          break;
      }
    }
  }
}

void eventHandler(velocity &v_target, pid_param &k, pwm &p, int &mode)
{
  String msg = readMsg();

  if (msg.length() > 0)
  {
    deserializeJson(input, msg);

    JsonObject obj = input.as<JsonObject>();

    if (obj.containsKey("kp"))
    {
      k.p = obj["kp"];
    }
    if (obj.containsKey("ki"))
    {
      k.i = obj["ki"];
    }
    if (obj.containsKey("kd"))
    {
      k.d = obj["kd"];
    }
    if (obj.containsKey("pwm_l"))
    {
      p.l = obj["pwm_l"];
    }
    if (obj.containsKey("pwm_r"))
    {
      p.r = obj["pwm_r"];
    }
    if (obj.containsKey("vl"))
    {
      v_target.wl = obj["vl"];
    }
    if (obj.containsKey("vr"))
    {
      v_target.wr = obj["vr"];
    }
    if (obj.containsKey("mode"))
    {
      mode =  obj["mode"];
    }
    // sentMsg(v);
  }
}

//{"vl":0,"vr":0}
void sentMsg(velocity v, pid_param k, pwm p)
{
  output["vl"] = v.wl;
  output["vr"] = v.wr;

  output["kp"] = k.p;
  output["ki"] = k.i;
  output["kd"] = k.d;

  output["pwm_l"] = p.l;
  output["pwm_r"] = p.r;

  String res;
  serializeJson(output, res);
  Serial.println(res);
}

#endif