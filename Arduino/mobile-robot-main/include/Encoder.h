#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

class Encoder
{
private:
    /* data */
public:
    Encoder();
    void init(int enaIN, int enbIN, bool invertIn);
    void update();

    int ena, enb;
    bool invert;
    volatile int position = 0;
};

Encoder::Encoder()
{
}

void Encoder::init(int enaIN, int enbIN, bool invertIn)
{
    this->ena = enaIN;
    this->enb = enbIN;
    this->invert = invertIn;

    pinMode(ena, INPUT_PULLUP);
    pinMode(enb, INPUT_PULLUP);
}

void Encoder::update()
{
    if (this->invert)
    {
        if (digitalRead(this->enb) == LOW)
        {
            if (this->position == encoder_maximum)
            {
                this->position = encoder_minimum;
            }
            else
            {
                this->position++;
            }
        }
        else
        {
            if (this->position == encoder_minimum)
            {
                this->position = encoder_maximum;
            }
            else
            {
                this->position--;
            }
        }
    }
    else
    {
        if (digitalRead(this->enb) == LOW)
        {
            if (this->position == encoder_minimum)
            {
                this->position = encoder_maximum;
            }
            else
            {
                this->position--;
            }
        }
        else
        {
            if (this->position == encoder_maximum)
            {
                this->position = encoder_minimum;
            }
            else
            {
                this->position++;
            }
        }
    }
}

#endif
