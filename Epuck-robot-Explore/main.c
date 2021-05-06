/* Program to move robot to explore arena and avoid obstacles*/
#include <p30F6014A.h>
#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <uart/e_uart_char.h>
#include <stdio.h>

void Wait(long);
int GetSelector(void);

int main()
{
    // Initialization. 
    e_init_port(); // Initializes the ports.
    e_init_ad_scan(ALL_ADC); // Initializes the analog to digital converters.
    e_led_clear(); // Turn off all the LEDs in the initialization stage.
    e_start_agendas_processing(); // Allows the robot to execute multiple tasks.
    
   // e_init_uart1(); // Initializes the UART. Uncomment when you use bluetooth communication.
    e_init_motors(); // Initializes the motors. Uncomment when you use the motors.
    e_calibrate_ir(); // calibrate proximity sensors
    
    
    while(1)
    
        
        while(e_get_prox(0)>400 || e_get_prox(1)>400 || e_get_prox(2)>400 || e_get_prox(3)>400|| e_get_prox(4)>400 || e_get_prox(5)>400 || e_get_prox(6)>400|| e_get_prox(7)>400 ) 
        
            int i;
            for(i=0;i<=7;i++){
                if (e_get_prox(i)>=400)
                e_set_led(i,1);
                else e_set_led(i,0);
            
            while(e_get_prox(0)>=350 || e_get_prox(1)>=350 || e_get_prox(6)>=350 || e_get_prox(7)>=350  )
            
                if(e_get_prox(6)> e_get_prox(1))
                
                    do
                    
                        e_set_speed(0,-1000); // move clockwise
                    }while(e_get_prox(6)>e_get_prox(1));          
                
                else if(e_get_prox(1)>e_get_prox(6))
                
                    do
                    
                        e_set_speed(0,1000); // move anti clockwise 
                    } while(e_get_prox(1)>e_get_prox(6));
        
            e_set_speed(1000,0);
            
        
        
    
 return 0;
}

void Wait(long Duration) 
{
    long i;
    for(i=0;i<Duration;i++);
}

int GetSelector() 
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}



