#include "Arduino.h"
#include "uwbTag.h"

u_int8_t achor_id = 0xDE;
UWB_TAG tag(achor_id);

float distance;
void setup()
{
    Serial.begin(115200);
    tag.initSPI();
    delay(2);
    tag.initDWT();
    tag.configure();
    
    while (tag.state != UWB_TAG::dw_state::RDY)
    {
        Serial.println("init error ");
        delay(2000);
    }
    if (tag.state == UWB_TAG::dw_state::RDY)
    {
        Serial.println("ready");
        delay(2000);
    }
}

void loop()
{
    tag.raging(&distance);
    Serial.println(distance);
}