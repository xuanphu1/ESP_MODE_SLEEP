#include <stdio.h>
#include "../../component/lightSleep/light_sleep.h"
#include"../component/deepSleep/deep_sleep.h"

void app_main(void)
{       
        register_timer_wakeup(10000000);
        while (1)
        {
                // light_sleep();
                deep_sleep();
                // vTaskDelay(1000/portTICK_RATE_MS);
        }
        
        
    
    
}
