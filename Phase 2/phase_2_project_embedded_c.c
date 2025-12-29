#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// pins

// Motor Driver Pins (Port D and Port B)
#define ENA_PIN     PD3     // OC2B - PWM (Arduino D3)
#define ENB_PIN     PB1     // OC1A - PWM (Arduino D9)
#define IN1_PIN     PD5     // Arduino D5
#define IN2_PIN     PD4     // Arduino D4
#define IN3_PIN     PD7     // Arduino D7
#define IN4_PIN     PD6     // Arduino D6

// Ultrasonic Pins
#define TRIG_PIN    PB4     // Arduino D12
#define ECHO_PIN    PB0     // Arduino D8

// Headlight Pin
#define HEADLIGHT_PIN PD2   // Arduino D2

// ADC Channels
#define LDR_CHANNEL         0   // A0
#define SEATBELT_CHANNEL    1   // A1
#define DOORLOCK_CHANNEL    2   // A2

// I2C LCD Address
#define LCD_I2C_ADDR        0x20    // PCF8574 default address (Adafruit uses 0x20)

// ============================================================================
//                          CONSTANTS
// ============================================================================

#define HEADLIGHT_THRESHOLD     400
#define MIN_STOPPING_DISTANCE   25.0f

// TTC Parameters
#define WHEEL_DIAMETER          0.065f      // meters
#define MAX_RPM                 200.0f
#define DECEL_RATE              0.5f        // m/s^2

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

volatile char bt_cmd = 'S';
volatile int target_speed = 0;
volatile int current_speed = 0;
volatile uint32_t millis_counter = 0;

float prev_distance = 400.0f;
uint32_t prev_dist_time = 0;
float approach_velocity = 0.0f;

uint32_t last_update = 0;
uint32_t last_bt_send = 0;


ISR(TIMER0_OVF_vect) {
    millis_counter++;
}

uint32_t millis(void) {
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = millis_counter;
    SREG = sreg;
    return m;
}

void millis_init(void) {
    TCCR0A = 0;
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64
    TIMSK0 = (1 << TOIE0);               // Enable overflow interrupt
}

// Bluetooth HC-05)


void uart_init(uint32_t baud) {
    uint16_t ubrr = (F_CPU / 16 / baud) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // Enable RX interrupt
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data
}

// Circular buffer for received data
#define UART_RX_BUFFER_SIZE 32
volatile char uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uart_rx_head = 0;
volatile uint8_t uart_rx_tail = 0;

// UART RX Interrupt - stores received bytes in buffer
ISR(USART_RX_vect) {
    char c = UDR0;
    uint8_t next_head = (uart_rx_head + 1) % UART_RX_BUFFER_SIZE;
    if (next_head != uart_rx_tail) {  // Buffer not full
        uart_rx_buffer[uart_rx_head] = c;
        uart_rx_head = next_head;
    }
}

bool uart_available(void) {
    return (uart_rx_head != uart_rx_tail);
}

char uart_read(void) {
    while (!uart_available());  // Wait for data
    char c = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return c;
}

char uart_read_nonblocking(void) {
    if (!uart_available()) return 0;
    char c = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return c;
}

void uart_putchar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char *str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

void uart_println(const char *str) {
    uart_print(str);
    uart_putchar('\r');
    uart_putchar('\n');
}

void uart_print_int(int val) {
    char buf[12];
    itoa(val, buf, 10);
    uart_print(buf);
}

void uart_print_float(float val, uint8_t decimals) {
    char buf[16];
    dtostrf(val, 0, decimals, buf);
    uart_print(buf);
}

// ============================================================================
//                          I2C (TWI) INTERFACE
// ============================================================================

#define I2C_FREQ    100000UL

void i2c_init(void) {
    // Set SCL frequency
    TWSR = 0;  // Prescaler = 1
    TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
    TWCR = (1 << TWEN);  // Enable TWI
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

// ============================================================================
//                          LCD (I2C via PCF8574)
// ============================================================================

// PCF8574 bit mapping for LCD
#define LCD_RS      0x01
#define LCD_RW      0x02
#define LCD_EN      0x04
#define LCD_BL      0x08    // Backlight
#define LCD_D4      0x10
#define LCD_D5      0x20
#define LCD_D6      0x40
#define LCD_D7      0x80

static uint8_t lcd_backlight = LCD_BL;

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble & 0xF0) | lcd_backlight;
    if (rs) data |= LCD_RS;
    
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    
    // Enable high
    i2c_write(data | LCD_EN);
    _delay_us(1);
    
    // Enable low
    i2c_write(data & ~LCD_EN);
    _delay_us(50);
    
    i2c_stop();
}

void lcd_write_byte(uint8_t byte, uint8_t rs) {
    lcd_write_nibble(byte & 0xF0, rs);
    lcd_write_nibble((byte << 4) & 0xF0, rs);
}

void lcd_command(uint8_t cmd) {
    lcd_write_byte(cmd, 0);
    if (cmd < 4) _delay_ms(2);  // Clear/Home commands need more time
}

void lcd_data(uint8_t data) {
    lcd_write_byte(data, 1);
}

void lcd_init(void) {
    _delay_ms(50);
    
    // 4-bit initialization sequence
    lcd_write_nibble(0x30, 0);
    _delay_ms(5);
    lcd_write_nibble(0x30, 0);
    _delay_us(150);
    lcd_write_nibble(0x30, 0);
    _delay_us(150);
    lcd_write_nibble(0x20, 0);  // 4-bit mode
    _delay_us(150);
    
    lcd_command(0x28);  // 4-bit, 2 lines, 5x8 font
    lcd_command(0x0C);  // Display on, cursor off
    lcd_command(0x06);  // Entry mode: increment
    lcd_command(0x01);  // Clear display
    _delay_ms(2);
}

void lcd_clear(void) {
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = col + (row == 0 ? 0x00 : 0x40);
    lcd_command(0x80 | addr);
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

void lcd_print_int(int val) {
    char buf[12];
    itoa(val, buf, 10);
    lcd_print(buf);
}

// ============================================================================
//                          ADC
// ============================================================================

void adc_init(void) {
    // AVCC reference, right adjust
    ADMUX = (1 << REFS0);
    // Enable ADC, prescaler 128 (125kHz ADC clock at 16MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);              // Start conversion
    while (ADCSRA & (1 << ADSC));       // Wait for completion
    return ADC;
}

// ============================================================================
//                          PWM (Timer1 and Timer2)
// ============================================================================

void pwm_init(void) {
    // Timer2 for ENA (PD3/OC2B) - Fast PWM
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS21);  // Prescaler 8
    OCR2B = 0;
    
    // Timer1 for ENB (PB1/OC1A) - Fast PWM 8-bit
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler 8
    OCR1A = 0;
}

void pwm_set_ena(uint8_t duty) {
    OCR2B = duty;
}

void pwm_set_enb(uint8_t duty) {
    OCR1A = duty;
}

// ============================================================================
//                          GPIO INITIALIZATION
// ============================================================================

void gpio_init(void) {
    // Motor control pins as outputs
    DDRD |= (1 << ENA_PIN) | (1 << IN1_PIN) | (1 << IN2_PIN) | 
            (1 << IN3_PIN) | (1 << IN4_PIN) | (1 << HEADLIGHT_PIN);
    DDRB |= (1 << ENB_PIN) | (1 << TRIG_PIN);
    
    // Echo pin as input
    DDRB &= ~(1 << ECHO_PIN);
    
    // ADC pins (A0, A1, A2) - configured by ADC init
    // Enable internal pullups for safety sensors
    PORTC |= (1 << PC1) | (1 << PC2);
}

// ============================================================================
//                          ULTRASONIC SENSOR
// ============================================================================

float get_distance(void) {
    // Send trigger pulse
    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);
    
    // Wait for echo start (with timeout)
    uint16_t timeout = 10000;
    while (!(PINB & (1 << ECHO_PIN)) && timeout--) {
        _delay_us(1);
    }
    if (timeout == 0) return 400.0f;
    
    // Measure echo duration
    uint32_t duration = 0;
    timeout = 20000;
    while ((PINB & (1 << ECHO_PIN)) && timeout--) {
        duration++;
        _delay_us(1);
    }
    if (timeout == 0) return 400.0f;
    
    // Calculate distance in cm
    float distance = (float)duration * 0.034f / 2.0f;
    if (distance > 400.0f) distance = 400.0f;
    
    return distance;
}

// ============================================================================
//                          TTC SYSTEM
// ============================================================================

float pwm_to_velocity(int pwm_value) {
    float rpm = ((float)pwm_value / 255.0f) * MAX_RPM;
    float wheel_circumference = M_PI * WHEEL_DIAMETER;
    float velocity = (rpm * wheel_circumference) / 60.0f;
    return velocity;
}

void calculate_approach_velocity(float current_dist) {
    uint32_t current_time = millis();
    uint32_t delta_time = current_time - prev_dist_time;
    
    if (delta_time >= 50) {
        float dist_change = (prev_distance - current_dist) / 100.0f;
        float time_sec = (float)delta_time / 1000.0f;
        
        if (time_sec > 0) {
            float new_velocity = dist_change / time_sec;
            approach_velocity = 0.7f * approach_velocity + 0.3f * new_velocity;
        }
        
        prev_distance = current_dist;
        prev_dist_time = current_time;
    }
}

int calculate_ttc_speed(float distance, int target_spd) {
    float dist_meters = distance / 100.0f;
    float car_velocity = pwm_to_velocity(current_speed);
    float closing_velocity = car_velocity + approach_velocity;
    
    if (closing_velocity <= 0.01f) {
        return target_spd;
    }
    
    float ttc = dist_meters / closing_velocity;
    float time_to_stop = car_velocity / DECEL_RATE;
    float stopping_distance = (car_velocity * car_velocity) / (2.0f * DECEL_RATE);
    
    float safety_margin = 1.5f;
    float required_ttc = time_to_stop * safety_margin;
    float required_distance = stopping_distance * safety_margin * 100.0f;
    
    if (ttc < required_ttc || distance < required_distance) {
        float ttc_ratio = ttc / required_ttc;
        float dist_ratio = distance / required_distance;
        
        float ratio = (ttc_ratio < dist_ratio) ? ttc_ratio : dist_ratio;
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        
        int new_speed = (int)((float)target_spd * ratio);
        
        if (distance < MIN_STOPPING_DISTANCE) {
            return 0;
        }
        return (new_speed > 0) ? new_speed : 0;
    }
    
    return target_spd;
}

// ============================================================================
//                          SAFETY CHECK
// ============================================================================

bool safety_ok(void) {
    // Read with digital threshold (>512 = HIGH with pullup means OK)
    uint16_t belt = adc_read(SEATBELT_CHANNEL);
    uint16_t door = adc_read(DOORLOCK_CHANNEL);
    
    bool belt_ok = (belt > 512);
    bool door_ok = (door > 512);
    
    return (belt_ok && door_ok);
}

// ============================================================================
//                          SPEED COMMAND HANDLER
// ============================================================================

void handle_speed_command(char cmd) {
    switch (cmd) {
        case '0': target_speed = 80;  break;
        case '1': target_speed = 100; break;
        case '2': target_speed = 120; break;
        case '3': target_speed = 140; break;
        case '4': target_speed = 160; break;
        case '5': target_speed = 180; break;
        case '6': target_speed = 200; break;
        case '7': target_speed = 220; break;
        case '8': target_speed = 240; break;
        case '9': target_speed = 255; break;
        case 'q': target_speed = 255; break;
        default: break;
    }
}

// ============================================================================
//                          SPEED UPDATE
// ============================================================================

void update_speed(int target) {
    if (millis() - last_update < 40) return;
    last_update = millis();
    
    if (current_speed < target) current_speed += 5;
    else if (current_speed > target) current_speed -= 5;
    
    if (current_speed < 0) current_speed = 0;
    if (current_speed > 255) current_speed = 255;
    
    pwm_set_ena((uint8_t)current_speed);
    pwm_set_enb((uint8_t)current_speed);
}

// ============================================================================
//                          MOTOR CONTROL
// ============================================================================

void motor_forward(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTD |= (1 << IN2_PIN);
    PORTD &= ~(1 << IN3_PIN);
    PORTD |= (1 << IN4_PIN);
}

void motor_backward(void) {
    PORTD |= (1 << IN1_PIN);
    PORTD &= ~(1 << IN2_PIN);
    PORTD |= (1 << IN3_PIN);
    PORTD &= ~(1 << IN4_PIN);
}

void motor_left(void) {
    PORTD |= (1 << IN1_PIN);
    PORTD &= ~(1 << IN2_PIN);
    PORTD &= ~(1 << IN3_PIN);
    PORTD |= (1 << IN4_PIN);
}

void motor_right(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTD |= (1 << IN2_PIN);
    PORTD |= (1 << IN3_PIN);
    PORTD &= ~(1 << IN4_PIN);
}

void motor_stop(void) {
    PORTD &= ~((1 << IN1_PIN) | (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN));
}

void move_car(char cmd) {
    switch (cmd) {
        case 'F': motor_forward();  break;
        case 'B': motor_backward(); break;
        case 'L': motor_left();     break;
        case 'R': motor_right();    break;
        default:  motor_stop();     break;
    }
}

// ============================================================================
//                          LCD UPDATE
// ============================================================================

void update_lcd(int spd, bool light_on, float dist, bool safe) {
    lcd_set_cursor(0, 0);
    lcd_print("SPD:");
    if (spd < 100) lcd_print(" ");
    if (spd < 10) lcd_print(" ");
    lcd_print_int(spd);
    
    lcd_print(" L:");
    lcd_print(light_on ? "ON " : "OFF");
    
    lcd_set_cursor(0, 1);
    if (!safe) {
        lcd_print("SAFETY FAIL    ");
    } else if (dist < MIN_STOPPING_DISTANCE) {
        lcd_print("EMERG STOP!    ");
    } else if (current_speed < target_speed && bt_cmd == 'F') {
        lcd_print("TTC BRAKING    ");
    } else {
        lcd_print("D:");
        lcd_print_int((int)dist);
        lcd_print("cm SAFE   ");
    }
}

// ============================================================================
//                    BLUETOOTH DATA LOGGING
// ============================================================================

void send_bluetooth_log(bool light_on, float dist, bool safe) {
    if (millis() - last_bt_send < 500) return;
    last_bt_send = millis();
    
    uart_print("SPD=");
    uart_print_int(current_speed);
    uart_println("");
    
    uart_print("TARGET_SPD=");
    uart_print_int(target_speed);
    uart_println("");
    
    uart_print("LIGHT=");
    uart_println(light_on ? "ON" : "OFF");
    
    uart_print("DIST=");
    uart_print_float(dist, 1);
    uart_println("");
    
    uart_print("APPROACH_VEL=");
    uart_print_float(approach_velocity, 3);
    uart_println("");
    
    uart_print("TTC_ACTIVE=");
    uart_println((current_speed < target_speed && bt_cmd == 'F') ? "YES" : "NO");
    
    uart_print("SAFETY=");
    uart_println(safe ? "OK" : "FAIL");
    
    uart_print("CMD=");
    char cmd_str[2] = {bt_cmd, '\0'};
    uart_println(cmd_str);
    
    uart_println("---");  // Separator for readability
}

// ============================================================================
//                          MAIN FUNCTION
// ============================================================================

int main(void) {
    // Initialize peripherals
    gpio_init();
    uart_init(9600);      // Hardware UART for Bluetooth
    i2c_init();
    adc_init();
    pwm_init();
    millis_init();
    
    // Enable global interrupts
    sei();
    
    // Initialize LCD
    lcd_init();
    lcd_print("   SMART  CAR");
    lcd_set_cursor(0, 1);
    lcd_print("   By Shadow");
    _delay_ms(1500);
    lcd_clear();
    
    uart_println("Smart Car Ready");
    uart_println("Send F/B/L/R/S to move");
    uart_println("Send 0-9 for speed");
    
    // Main loop
    while (1) {
        // Check for Bluetooth commands (using hardware UART with interrupt buffer)
        if (uart_available()) {
            char received = uart_read_nonblocking();
            if (received != 0) {
                bt_cmd = received;
                
                // Echo back received command
                uart_print("CMD: ");
                char cmd_str[2] = {bt_cmd, '\0'};
                uart_println(cmd_str);
                
                handle_speed_command(bt_cmd);
            }
        }
        
        // Light control by LDR
        uint16_t light_level = adc_read(LDR_CHANNEL);
        bool light_on = (light_level > HEADLIGHT_THRESHOLD);
        if (light_on) {
            PORTD |= (1 << HEADLIGHT_PIN);
        } else {
            PORTD &= ~(1 << HEADLIGHT_PIN);
        }
        
        // Get ultrasonic distance
        float dist = get_distance();
        
        // Calculate approach velocity
        calculate_approach_velocity(dist);
        
        // TTC-based speed adjustment
        int adjusted_speed = target_speed;
        
        if (bt_cmd == 'F') {
            adjusted_speed = calculate_ttc_speed(dist, target_speed);
        }
        
        // Update speed
        update_speed(adjusted_speed);
        
        // Safety check
        bool allowed = safety_ok();
        
        // Movement control
        if (allowed) {
            if (bt_cmd == 'F' && dist < MIN_STOPPING_DISTANCE) {
                motor_stop();
            } else {
                move_car(bt_cmd);
            }
        } else {
            motor_stop();
        }
        
        // Update LCD
        update_lcd(current_speed, light_on, dist, allowed);
        
        // Send Bluetooth log
        send_bluetooth_log(light_on, dist, allowed);
    }
    
    return 0;
}
