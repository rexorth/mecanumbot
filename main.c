#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include <math.h>

#include "tusb.h"
#include "xinput_host.h"


#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define DEADZONE (1500)
#define AXIS_MAX (32768.0f)
#define WHEEL_DIAMETER (0.086f)
// Mecanum "wheel geometry" term: (L + W) / 2, where L is the longitudinal
// distance between wheels and W is the lateral distance. See
// https://ecam-eurobot.github.io/Tutorials/software/mecanum/mecanum.html.
// NOTE: 0.085 currently reflects only the lateral half-distance; refine when
// chassis dimensions are remeasured.
#define WHEEL_GEOMETRY (0.085f)
#define STEPS_PER_REV (200)

// Actual min is 300 but I want the min to be divisible by the control tick value.
// Note: tick_wheel toggles the step pin every current_delay_us, so a full step
// takes 2 * current_delay_us. MIN_STEP_DELAY was calibrated empirically against
// that toggling behavior — do not "fix" it without re-measuring.
#define MIN_STEP_DELAY (400)

// For delay 400 this is 3.377 m/s.
#define MAX_WHEEL_SPEED (3.14f * WHEEL_DIAMETER * 1000000.0f / ((float)MIN_STEP_DELAY * (float)STEPS_PER_REV))

// Max yaw rate in rad/s. Defined so a full right-stick deflection alone
// saturates a wheel exactly at MAX_WHEEL_SPEED. This intentionally cancels
// WHEEL_GEOMETRY out of the yaw response: full stick = full wheel speed
// regardless of the chassis geometry value. To make yaw tunable independently
// of the geometry, set this to a fixed rad/s target instead.
#define MAX_ANGULAR_SPEED (MAX_WHEEL_SPEED / WHEEL_GEOMETRY)


// define control tick in microseconds and clkdiv as any int to calculate wrap val
//wrap value just needs to be below 65535
#define CONTROL_TICK_US 200
#define CONTROL_PWM_GPIO 15
#define CONTROL_PWM_SLICE pwm_gpio_to_slice_num(CONTROL_PWM_GPIO)
#define CONTROL_CLKDIV 50
#define SYSCLOCK_IN_MHZ 150
#define CONTROL_WRAP 1 + (150*CONTROL_TICK_US)/CONTROL_CLKDIV

typedef struct {
    volatile float x, y, w;
} MotionInput;

typedef struct {
    volatile float current_speed; 
    volatile int current_delay_us, accumulated_us, direction, step_pin_state, default_direction; 
    int StepPin, DirPin;
    const char* name;
} WheelState; 


//declare functions


MotionInput currentInput;

WheelState wheelFL, wheelFR, wheelRL, wheelRR;

void init_wheelState() {
    wheelFL = (WheelState){.current_delay_us = MIN_STEP_DELAY, .default_direction = 0, .name="Front Left", .StepPin = 2, .DirPin = 3};
    wheelRL = (WheelState){.current_delay_us = MIN_STEP_DELAY, .default_direction = 0, .name="Rear Left", .StepPin = 4, .DirPin = 5};
    wheelFR = (WheelState){.current_delay_us = MIN_STEP_DELAY, .default_direction = 1, .name="Front Right", .StepPin = 6, .DirPin = 7};
    wheelRR = (WheelState){.current_delay_us = MIN_STEP_DELAY, .default_direction = 1, .name="Rear Right", .StepPin = 8, .DirPin = 9};
}

void printInputState() {
    TU_LOG1("x: %f, y: %f, w: %f\n", currentInput.x, currentInput.y, currentInput.w);
}

void printWheelState(WheelState* w) {
    TU_LOG1("Wheel:%s Speed: %f, Delay: %d, Direction:%d\n", w->name, w->current_speed, w->current_delay_us, w->direction);
}

void printBotState() {
    printWheelState(&wheelFL);
    printWheelState(&wheelRL);
    printWheelState(&wheelRR);
    printWheelState(&wheelFR);
}

//delay = pi*D / V * s where s is steps per rev(WheelState)
void update_wheel_delay(WheelState* w) {
    if(w->current_speed < 0.01f) {
        w->current_delay_us = -1;
    } else {
        w->current_delay_us = 1e6f*((float)M_PI*WHEEL_DIAMETER)/(w->current_speed * (float)STEPS_PER_REV);
    }
}

void update_all_wheel_delays() {
    update_wheel_delay(&wheelFL);
    update_wheel_delay(&wheelRL);
    update_wheel_delay(&wheelRR);
    update_wheel_delay(&wheelFR);
}

//check current speed, if value is negative then set direction to inverse of default direction
//DIR 0 = counterclockwise (need to verify)
//if absDir == default direction then direction should be 1, otherwise 0
void set_direction(WheelState* w) {
    if(w->current_speed == 0.0f) {
        // Leave direction unchanged at zero crossings so the dir pin doesn't flap.
        return;
    }
    int absDir = w->current_speed > 0;
    if(!absDir) {
        w->current_speed *= -1;
    }
    w->direction = (absDir == w->default_direction);
}

//calculate control signal strength and scale everything down if it's over the max speed
//also set wheel direction
void update_wheel_speeds() {

    wheelFL.current_speed = currentInput.y - currentInput.x - (WHEEL_GEOMETRY*currentInput.w);
    wheelRL.current_speed = currentInput.y + currentInput.x - (WHEEL_GEOMETRY*currentInput.w);
    wheelRR.current_speed = currentInput.y - currentInput.x + (WHEEL_GEOMETRY*currentInput.w);
    wheelFR.current_speed = currentInput.y + currentInput.x + (WHEEL_GEOMETRY*currentInput.w);

    

    // I want all wheel speeds to be positive so I'm going to set the direction here
    set_direction(&wheelFL);
    set_direction(&wheelRL);
    set_direction(&wheelRR);
    set_direction(&wheelFR);

    float wheelSpeeds[4] = {wheelFL.current_speed, wheelRL.current_speed, wheelRR.current_speed, wheelFR.current_speed};
    float scalar = 1.0f;
    float maxWheelSpeed = MAX_WHEEL_SPEED;
    for(int i = 0; i < 4; i++) {
        
        float tempScalar = wheelSpeeds[i] / maxWheelSpeed;
        //TU_LOG1("tempscalar: %f wheelSpeed: %f maxspeed: %f\n", tempScalar, wheelSpeeds[i], MAX_WHEEL_SPEED);
        if(tempScalar > 1.0f && tempScalar > scalar) {
            scalar = tempScalar;
        }
    }
    if(scalar > 1.0f) {
        wheelFL.current_speed /= scalar;
        wheelRL.current_speed /= scalar;
        wheelRR.current_speed /= scalar;
        wheelFR.current_speed /= scalar;
    }

}

void tick_wheel(WheelState* w) {
    if(w->current_delay_us <= 0){return;}
    w->accumulated_us += CONTROL_TICK_US;
    if(w->accumulated_us>w->current_delay_us) {
        w->step_pin_state = !w->step_pin_state;
        gpio_put(w->DirPin, w->direction);
        gpio_put(w->StepPin,w->step_pin_state);
        w->accumulated_us = w->accumulated_us % w->current_delay_us;   
    }

}

void tick_wheels() {
    tick_wheel(&wheelFL);
    tick_wheel(&wheelRL);
    tick_wheel(&wheelRR);
    tick_wheel(&wheelFR);
}

void control_tick() {
    // Wheel speeds and step delays are recomputed in handle_gamepad_input()
    // (main thread) whenever a new gamepad report arrives. Here we just
    // accumulate time, check timers, and toggle step/direction pins.
    tick_wheels();
}

//process pwm isr -> call control handler
void __isr pwm_wrap_isr(void) {
    pwm_clear_irq(CONTROL_PWM_SLICE);
    control_tick();
}

//initialize pwm loop to call the motor handler
void init_pwm_isr_timer() {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CONTROL_CLKDIV);
    pwm_config_set_wrap(&config, CONTROL_WRAP);
    pwm_init(CONTROL_PWM_SLICE, &config, true);

    pwm_clear_irq(CONTROL_PWM_SLICE);
    pwm_set_irq_enabled(CONTROL_PWM_SLICE, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);
}

// Scale a stick axis to a physical command. `max` is the value that corresponds
// to full deflection (m/s for translation axes, rad/s for the yaw axis).
float normalize_axis(int16_t value, float max) {
    if ( (-DEADZONE < value) && (value < DEADZONE) ) { return 0.0f;}
    return max * (float)value / AXIS_MAX;
}

void handle_gamepad_input(const xinput_gamepad_t* p) {
    currentInput.x = normalize_axis(p->sThumbLX, MAX_WHEEL_SPEED);
    currentInput.y = normalize_axis(p->sThumbLY, MAX_WHEEL_SPEED);
    currentInput.w = normalize_axis(p->sThumbRX, MAX_ANGULAR_SPEED);

    // Recompute wheel speeds and step delays here (main thread) instead of in
    // the PWM ISR, so the ISR never observes a torn currentInput and only does
    // the constant-time tick work. Disable IRQs briefly so the ISR can't read
    // a half-updated (direction, current_delay_us) pair across the four wheels.
    uint32_t irq_state = save_and_disable_interrupts();
    update_wheel_speeds();
    update_all_wheel_delays();
    restore_interrupts(irq_state);
}

void init_wheelGPIO(WheelState* w) {
    gpio_init(w->StepPin);
    gpio_init(w->DirPin);
    gpio_set_dir(w->StepPin, GPIO_OUT);
    gpio_set_dir(w->DirPin, GPIO_OUT);
    gpio_put(w->StepPin, 0);
    gpio_put(w->DirPin, 0);
}

usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count){
    *driver_count = 1;
    return &usbh_xinput_driver;
}

void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, xinputh_interface_t const* xid_itf, uint16_t len)
{
    const xinput_gamepad_t *p = &xid_itf->pad;
    const char* type_str;

    if (xid_itf->last_xfer_result == XFER_RESULT_SUCCESS)
    {
        switch (xid_itf->type)
        {
            case XBOXONE:          type_str = "Xbox One";          break;
            case XBOX360_WIRELESS: type_str = "Xbox 360 Wireless"; break;
            case XBOX360_WIRED:    type_str = "Xbox 360 Wired";    break;
            case XBOXOG:           type_str = "Xbox OG";           break;
            default:               type_str = "Unknown";
        }

        if (xid_itf->connected && xid_itf->new_pad_data)
        {
            //TU_LOG1("[%02x, %02x], Type: %s, Buttons %04x, LT: %02x RT: %02x, LX: %d, LY: %d, RX: %d, RY: %d\n",
            //    dev_addr, instance, type_str, p->wButtons, p->bLeftTrigger, p->bRightTrigger, p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);

            //How to check specific buttons
            //if (p->wButtons & XINPUT_GAMEPAD_A) TU_LOG1("You are pressing A\n");
            
            //TU_LOG1("LX: %d, LY: %d, RX: %d, RY: %d\n", p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);
            
            
            //TU_LOG1("x=%f, y=%f, z=%f, FL=%f, FR=%f, RL=%f, RR=%f \n", currentInput.x, currentInput.y, currentInput.w, wheelFL.current_speed, wheelFR.current_speed, wheelRL.current_speed, wheelRR.current_speed);
            handle_gamepad_input(p);
        }
    }
    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t *xinput_itf)
{
    TU_LOG1("XINPUT MOUNTED %02x %d\n", dev_addr, instance);
    // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
    // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
    if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false)
    {
        tuh_xinput_receive_report(dev_addr, instance);
        return;
    }
    tuh_xinput_set_led(dev_addr, instance, 0, true);
    tuh_xinput_set_led(dev_addr, instance, 1, true);
    tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    TU_LOG1("XINPUT UNMOUNTED %02x %d\n", dev_addr, instance);
}

int main() {
    
    stdio_init_all();
    
    uart_init(uart0, BAUD_RATE);

    tuh_init(BOARD_TUH_RHPORT);

    board_init();
    init_wheelState();

    init_wheelGPIO(&wheelFL);
    init_wheelGPIO(&wheelFR);
    init_wheelGPIO(&wheelRR);
    init_wheelGPIO(&wheelRL);
    
    init_pwm_isr_timer();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    TU_LOG1("Max wheel speed %f m/s, max yaw %f rad/s, Control Wrap:%d\n",
        MAX_WHEEL_SPEED, MAX_ANGULAR_SPEED, CONTROL_WRAP);

    while(1) {
        tuh_task();
        //TU_LOG1("1");
    }
    return 0;
}