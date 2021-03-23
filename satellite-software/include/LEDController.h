#ifndef __LED_CONTROLLER_H__
#define __LED_CONTROLLER_H__

class LEDController 
{
    public:
        LEDController(const uint8_t pinNumber);
        void switchOn();
        void switchOff();

    private:
        uint8_t pinNumberForLED;
};

#endif