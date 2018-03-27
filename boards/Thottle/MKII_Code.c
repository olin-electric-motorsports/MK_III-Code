#include <avr/io.h>
#include <avr/interrupt.h>

#include "can_api.h"

/* Defines */
#define PORT_LED1  PORTB
#define PORT_LED2  PORTC
#define LED1       PB1
#define LED2       PC7

#define FLAG_BRAKE          0
#define FLAG_BSPD           1
#define FLAG_IMMEDIATE_CAN  2
#define FLAG_PANIC          3
#define FLAG_MOTOR_ON       4
#define FLAG_THROTTLE_BRAKE 5
#define FLAG_THROTTLE_10    6

#define MOB_DASHBOARD         0
#define MOB_THROTTLE          1
#define MOB_MOTOR_CONTROLLER  2
#define MOB_AIR_CONTROL       3
#define MOB_PANIC             4

#define THROTTLE_DEADZONE  10
#define THROTTLE_MAX_ADJUST_AMOUNT  50


/* Global Variables */
volatile uint8_t gFlags = 0x00;
uint8_t throttle_10_count = 0x00;
uint8_t gThrottle[2] = {0x00, 0x00};
uint8_t gThrottle_smoothed = 0x00;
uint16_t gFilter_reg = 0x00;

uint8_t gCanMsg[8] = {0x00};
uint8_t gCanMsgMotorController[8] = {0x00};

uint8_t timer_counter = 0; // Timer scale down variables
uint8_t timer_scale_factor = 40; // Timer from 4MHz -> 4KHz -> 100Hz

// Throttle mapping defaults
uint8_t Throttle_1_HIGH = 0xE7;
uint8_t Throttle_1_LOW  = 0xA2;
uint8_t Throttle_2_HIGH = 0xA1;
uint8_t Throttle_2_LOW  = 0x6D;


/* Function Prototypes */
void handleFlags(void);
void initADC(void);
void initPWM(void);
void initTimer(void);
void readAndStoreThrottle(void);
void sendCanMessages(void);
void updateBrake(void);


/* Function Definitions */

void initADC(void) {
    //ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);

    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);

    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0); //sets the reference voltage

    //Reads by default from ADC0 (pin 11); this line
    //  is redundant. the timer
    ADMUX |= _BV( 0x00 );
}

void initPWM(void) {
    //No prescaling on PWM clock
    TCCR0B |= _BV(CS00);

    //Set up phase-correct PWM on OC0B
    TCCR0A |= _BV(COM0B1) | _BV(WGM00);

    //Reset the other PWM pin
    TCCR0A &= ~_BV(COM0B0);
}

void initTimer(void) {
    // CTC Mode
    TCCR1B |= _BV(WGM12);

    // 1/1024 prescaler
    TCCR1B |= _BV(CS12);// | _BV(CS10);

    // Trigger on 0xFF (about twice a second)
    OCR1AL = 0x0F;
    OCR1AH = 0x00;

    // Give us that good interrupt
    TIMSK1 |= _BV(OCIE1A);
}

void readAndStoreThrottle(void) {
    // Unroll our loop for unecessary optimization
    ADMUX = _BV(REFS0);
    ADMUX |= 8;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t throttle1 = (uint8_t) (ADC >> 2);

    ADMUX = _BV(REFS0);
    ADMUX |= 9;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t throttle2 = (uint8_t) (ADC >> 2);

    // Do some Math

    // Adjust for Throttle slop
    if (throttle1 > Throttle_1_HIGH) {
        if (throttle1 > (Throttle_1_HIGH + THROTTLE_MAX_ADJUST_AMOUNT)) {
            gFlags |= _BV(FLAG_PANIC);
            return;
        }
        throttle1 = Throttle_1_HIGH;
    }

    if (throttle2 > Throttle_2_HIGH) {
        if (throttle2 > (Throttle_2_HIGH + THROTTLE_MAX_ADJUST_AMOUNT)) {
            gFlags |= _BV(FLAG_PANIC);
            return;
        }
        throttle2 = Throttle_2_HIGH;
    }

    if (throttle1 < Throttle_1_LOW) {
        if (throttle1 < (Throttle_1_LOW - THROTTLE_MAX_ADJUST_AMOUNT)) {
            gFlags |= _BV(FLAG_PANIC);
            return;
        }
        throttle1 = Throttle_1_LOW;
    }

    if (throttle2 < Throttle_2_LOW) {
        if (throttle2 < (Throttle_2_LOW - THROTTLE_MAX_ADJUST_AMOUNT)) {
            gFlags |= _BV(FLAG_PANIC);
            return;
        }
        throttle2 = Throttle_2_LOW;
    }

    // Map both to [0x00, 0xff] range
    uint8_t throttle1_mapped = ((throttle1 - Throttle_1_LOW) * 0xFF) / (Throttle_1_HIGH - Throttle_1_LOW);
    uint8_t throttle2_mapped = ((throttle2 - Throttle_2_LOW) * 0xFF) / (Throttle_2_HIGH - Throttle_2_LOW);

    // Rolling average
    static uint8_t rolling1[32];
    static uint8_t rolling2[32];

    for (int i=0; i < 31; i++) {
        rolling1[i] = rolling1[i+1];
        rolling2[i] = rolling2[i+1];
    }
    rolling1[31] = throttle1_mapped;
    rolling2[31] = throttle2_mapped;

    uint16_t avg1 = 0;
    uint16_t avg2 = 0;
    for (int i=0; i < 32; i++) {
        avg1 += rolling1[i];
        avg2 += rolling2[i];
    }

    throttle1_mapped = avg1 >> 5;
    throttle2_mapped = avg2 >> 5;

    /*
    static uint16_t filt1;
    static uint16_t filt2;
    filt1 = filt1 - (filt1 << 4) + throttle1_mapped;
    throttle1 = filt1 >> 4;

    filt2 = filt2 - (filt2 << 4) + throttle2_mapped;
    throttle2 = filt2 >> 4;
    */



    // Check if they are within 10%
    uint8_t err = 0;
    if (throttle1_mapped > throttle2_mapped && (throttle1_mapped - throttle2_mapped) >= (0xFF/10)) {
        err = 1;
        throttle_10_count++;
        gFlags |= _BV(FLAG_THROTTLE_10);
    }
    else if (throttle2_mapped > throttle1_mapped && (throttle2_mapped - throttle1_mapped) >= (0xFF/10)) {
        err = 1;
        throttle_10_count++;
        gFlags |= _BV(FLAG_THROTTLE_10);
    }

    // Oops we got an error
    if (err) {
        gFlags |= _BV(FLAG_PANIC);
        return;
    }

    // Deadzone
    if (throttle1_mapped < THROTTLE_DEADZONE) {
        throttle1_mapped = 0x00;
    } else if (throttle1_mapped > (0xFF - THROTTLE_DEADZONE)) {
        throttle1_mapped = 0xFF;
    }

    if (throttle2_mapped < THROTTLE_DEADZONE) {
        throttle2_mapped = 0x00;
    } else if (throttle2_mapped > (0xFF - THROTTLE_DEADZONE)) {
        throttle2_mapped = 0xFF;
    }

    if (throttle1_mapped < 38 || throttle2_mapped < 38) {
        gFlags &= ~_BV(FLAG_THROTTLE_BRAKE);
    }

    updateBrake();
    // Set throttle if brake is not pressed
    if (bit_is_clear(gFlags, FLAG_BRAKE) && bit_is_clear(gFlags, FLAG_THROTTLE_BRAKE)) {
        gThrottle[0] = throttle1_mapped;
        gThrottle[1] = throttle2_mapped;
        //gThrottle[0] = throttle1;
        //gThrottle[1] = throttle2;

        //gFilter_reg = gFilter_reg - (gFilter_reg >> 2) + gThrottle[0];
        //gThrottle_smoothed = gFilter_reg >> 2;
    } else {
        gThrottle[0] = 0x00;
        gThrottle[1] = 0x00;
        //gThrottle_smoothed = 0x00;
        gFlags |= _BV(FLAG_THROTTLE_BRAKE);
    }
}

void updateBrake(void) {
    if (bit_is_set(PIND, PD5)) {
        gFlags |= _BV(FLAG_BRAKE);
    } else {
        gFlags &= ~_BV(FLAG_BRAKE);
        //gFlags |= _BV(FLAG_IMMEDIATE_CAN);
    }
}

void sendCanMessages(void) {
    // Note: this is called by an interrupt; keep it succinct

    // Don't try to do anything if panic'ed
    if (bit_is_set(gFlags, FLAG_PANIC)) {
        return;
    }

    // Send out on Throttle Node
    gCanMsg[0] = gThrottle[0];
    gCanMsg[1] = gThrottle[1];
    gCanMsg[2] = bit_is_set(gFlags, FLAG_BRAKE) ? 0xFF : 0x00;
    gCanMsg[3] = gThrottle_smoothed;
    gCanMsg[4] = bit_is_set(gFlags, FLAG_THROTTLE_10) ? 0xFF : 0x00;
    gCanMsg[5] = bit_is_set(gFlags, FLAG_THROTTLE_BRAKE) ? 0xFF : 0x00;
    gCanMsg[6] = throttle_10_count; // TODO: Shutdown
    gCanMsg[7] = 0x00; // TODO: Shutdown

    CAN_transmit(MOB_THROTTLE,
                 CAN_IDT_THROTTLE,
                 CAN_IDT_THROTTLE_L,
                 gCanMsg);

    // Send out Motor controller info
    // REMAP
    uint16_t thrott = (uint16_t) gThrottle[0];
    //uint16_t thrott = (uint16_t) gThrottle_smoothed;
    uint16_t mc_remap = (uint16_t)(thrott * 9);
    gCanMsgMotorController[0] = mc_remap; // Speed mode
    gCanMsgMotorController[1] = mc_remap >> 8; // Speed mode
    gCanMsgMotorController[2] = 0x00; // Speed mode
    gCanMsgMotorController[3] = 0x00; // Speed mode
    gCanMsgMotorController[4] = 1; // CW; 1=CCW
    gCanMsgMotorController[5] = bit_is_set(gFlags, FLAG_MOTOR_ON) ? 0x01 : 0x00; // Inverter
    gCanMsgMotorController[6] = 0x00; // Torque limits
    gCanMsgMotorController[7] = 0x00; // Torque limits

    CAN_transmit(MOB_MOTOR_CONTROLLER,
                 0xC0,
                 8,
                 gCanMsgMotorController);
}

void handleFlags(void) {
    if (bit_is_set(gFlags, FLAG_PANIC)) {
        gThrottle[0] = 0x00;
        gThrottle[1] = 0x00;
        gThrottle_smoothed = 0x00;

        uint8_t msg[] = {0x21};
        CAN_transmit(MOB_PANIC,
                     CAN_IDT_PANIC,
                     CAN_IDT_PANIC_L,
                     msg);

        gFlags &= ~_BV(FLAG_PANIC);
    }

    if (bit_is_set(gFlags, FLAG_IMMEDIATE_CAN)) {
        sendCanMessages();
    }
}


/* Interrupt Vectors */

ISR(TIMER1_COMPA_vect) {
    timer_counter ++;
    if (timer_counter > timer_scale_factor) {
        sendCanMessages();
        timer_counter = 0;
    }
}

ISR(CAN_INT_vect) {
    CANPAGE = (MOB_DASHBOARD << MOBNB0);
    if (bit_is_set(CANSTMOB, RXOK)) {
        // Unrolled read for 5th byte
        volatile uint8_t msg = CANMSG;

        // Startup status
        if (msg == 0xFF) {
            gFlags |= _BV(FLAG_MOTOR_ON);
        } else {
            gFlags &= ~_BV(FLAG_MOTOR_ON);
        }

        // Reset status
        CANSTMOB = 0x00;
        CAN_wait_on_receive(MOB_DASHBOARD,
                            CAN_IDT_DASHBOARD,
                            CAN_IDT_DASHBOARD_L,
                            CAN_IDM_single);
    }

    // Air Control
    CANPAGE = (MOB_AIR_CONTROL << MOBNB0);
    if (bit_is_set(CANSTMOB, RXOK)) {
        // Unrolled read for 5th byte
        volatile uint8_t msg = CANMSG;
        msg = CANMSG;

        // Startup status
        if (msg == 0xFF) {
            //gFlags |= _BV(FLAG_MOTOR_ON);
        } else {
            gFlags &= ~_BV(FLAG_MOTOR_ON);
        }

        // Reset status
        CANSTMOB = 0x00;
        CAN_wait_on_receive(MOB_AIR_CONTROL,
                            CAN_IDT_AIR_CONTROL,
                            CAN_IDT_AIR_CONTROL_L,
                            CAN_IDM_single);
    }
}


/* Main Function */

int main(void) {
    sei();

    // Initialize things
    initADC();
    initPWM();
    initTimer();

    CAN_init(CAN_ENABLED);
    // Set up our CAN receives
    CAN_wait_on_receive(MOB_DASHBOARD,
                        CAN_IDT_DASHBOARD,
                        CAN_IDT_DASHBOARD_L,
                        CAN_IDM_single);

    CAN_wait_on_receive(MOB_AIR_CONTROL,
                        CAN_IDT_AIR_CONTROL,
                        CAN_IDT_AIR_CONTROL_L,
                        CAN_IDM_single);

    // Love me some infinite while loops
    while(1) {
        // We will just constantly update our values
        updateBrake();
        readAndStoreThrottle();

        // Act on flags
        handleFlags();

        // Listen to Dashboard!
        CAN_wait_on_receive(MOB_DASHBOARD,
                            CAN_IDT_DASHBOARD,
                            CAN_IDT_DASHBOARD_L,
                            CAN_IDM_single);
    }
}
