#include <p30F6014A.h>
#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <uart/e_uart_char.h>
#include <stdio.h>


void Wait(long);
int GetSelector(void);
void MoveWheels(int,int);

int main()
{
    // Initialisation. 
    e_init_port(); // Initialises the ports.
    e_init_ad_scan(ALL_ADC); // Initialises the analogue to digital converters.
    e_led_clear(); // Turn off all the LEDs in the initialisation stage.
    e_start_agendas_processing(); // Allows the robot to execute multiple tasks.
    
     e_init_uart1(); // Initialises the UART. Uncomment when you use Bluetooth communication.
    e_calibrate_ir(); // Uncomment when you use Proximity Sensors.
     e_init_motors(); // Initialises the motors. Uncomment when you use the motors.
     
     e_calibrate_ir(); // calibrate proximity sensors
     unsigned int x;
     x = 2;
    while(1)
    {
        if (e_get_prox(x)<=20)
        {
        MoveWheels(-1000,-1000);
        Wait(2000000);
        }
    }
    return 0;
}

void Wait(long Duration) 
{
    long i;
    for(i=0;i<Duration;i++);
}

int GetSelector() {
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void MoveWheels(int LeftSpeed, int RightSpeed){
     e_set_speed_left(LeftSpeed); // Turns on the left wheel.
     e_set_speed_right(RightSpeed); // Turns on the right wheel.
}
