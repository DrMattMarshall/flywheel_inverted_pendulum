/*
 * State-space controller for inverted flywheel pendulum (IFP) using a DC motor
 *
 * Note: This uses (as of 200323) measurement and calculation of state vector.
 * An observer (or minimum-state observer) might be better.
 */

#include <BasicLinearAlgebra.h>
#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint8_t my_timer_flag = 0;  // Execute control loop in main() via polling
volatile float current_count = 0;  // Encoder on rod should --> mag. of value < 1000

// Variables for 7-segment display
volatile uint8_t my_array[] = {0xFA, 0x60, 0xD9, 0xF1, 0x63, 0xB3, 0xBB, 0xE0, 0xFB, 0xF3};  // Patterns for digits on 7-segment
volatile uint8_t hundreds, tens, ones;
const uint8_t is_displaying_encoder_count = 0;  // NOTE: enabling this slows the control calculations

// Variables for data logging
const uint16_t num_log_values = 1750;
const uint8_t is_logging_more_than_just_encoder_count = 1;
volatile int log_encoder_count[num_log_values];
volatile uint8_t log_OCR2A[num_log_values];
volatile uint8_t log_PB5[num_log_values];
//volatile uint8_t log_OCR2A[0];
//volatile uint8_t log_PB5[0];
/*
volatile float log_theta_1[num_log_values];
volatile float log_x_hat_1[num_log_values];
volatile float log_x_hat_2[num_log_values];
volatile float log_x_hat_3[num_log_values];
volatile float log_control_action[num_log_values];
*/

// Variables for control
const float pi = 3.14159;
float theta_1;  // Position of pendulum rod (radians)
const float ticks_per_rev = 2000;  // Quadrature encoder data sheet shows 500 counts per revolution
const float max_motor_voltage = 33.4;  // Voltage of batteries powering system
const float quantization_e_in = max_motor_voltage / 255;  // Observer needs to know actual voltage
const uint8_t dead_band = 0;  // OCR2A must be 1 more than this value for driver to output
//const uint8_t is_using_observer = 0;


void serial_print_log() {
  TIMSK1 = 0;  // OCIE1A for control loop (disable timer interrupts)
  Serial.begin(9600);

  Serial.print("encoder_count = [");
  for (uint16_t i = 0; i < num_log_values; i++) {
    Serial.print(log_encoder_count[i]);
    if ((num_log_values - 1) > i)
      Serial.print(", ");
  }

  if (is_logging_more_than_just_encoder_count) {
    Serial.print("];\n\nOCR2A = [");
    for (uint16_t i = 0; i < num_log_values; i++) {
      Serial.print(log_OCR2A[i]);
      if ((num_log_values - 1) > i)
	Serial.print(", ");
    }
    
    Serial.print("];\n\nPB5 = [");
    for (uint16_t i = 0; i < num_log_values; i++) {
      Serial.print(log_PB5[i]);
      if ((num_log_values - 1) > i)
	Serial.print(", ");
    }
  }  // End branch for logging more than just encoder count
  /*
  Serial.print("theta_1 = [");
  for (uint16_t i = 0; i < num_log_values; i++) {
    Serial.print(log_theta_1[i], 6);
    if ((num_log_values - 1) > i)
      Serial.print(", ");
  }

  Serial.print("];\n\nx_1 = [");
  for (uint16_t i = 0; i < num_log_values; i++) {
    Serial.print(log_x_hat_1[i], 6);
    if ((num_log_values - 1) > i)
      Serial.print(", ");
  }
  
  Serial.print("];\n\nx_2 = [");
  for (uint16_t i = 0; i < num_log_values; i++) {
    Serial.print(log_x_hat_2[i], 6);
    if ((num_log_values - 1) > i)
      Serial.print(", ");
  }
  
  Serial.print("];\n\nx_3 = [");
  for (uint16_t i = 0; i < num_log_values; i++) {
    Serial.print(log_x_hat_3[i], 6);
    if ((num_log_values - 1) > i)
      Serial.print(", ");
  }

  Serial.print("];\n\nu = [");
  for (uint16_t i = 0; i < num_log_values; i++) {
    Serial.print(log_control_action[i], 6);
    if ((num_log_values - 1) > i)
      Serial.print(", ");
  }
  */
  Serial.print("];\n");

  Serial.end();
  TIMSK1 = 1 << 1;  // OCIE1A for control loop
}


int main(void) {
  // Configure built-in LED
  DDRB |= 1 << 7;  // Output for built-in LED
  PORTB &= ~(1 << 7);
  
  // I/O for printing to serial
  DDRB &= ~(1 << 6);  // Input for print-to-serial button
  PORTB |= 1 << 6;  // Enable pull-up resistor
  DDRF |= 1 << 4;  // Output for writing-to-file LED

  // Timer 2 configuration for phase-correct PWM
  DDRB |= 1 << 4;  // OC2A, Arduino digital pin 10
  TCCR2A = 0x81;
  TCCR2B = 0x02;  // Prescaler --- 0x01: p = 1, 0x02: p = 8, 0x03: p = 32
  DDRB |= 1 << 5;  // Output pin for motor direction, Arduino digital pin 11

  // Timer 1 configuration for control period
  TCCR1A = 0;
  TCCR1B = 0x0A;
  TCCR1C = 0;
  /*
   * 0.1 ms period --> TCCR1B = 0x0A, OCR1A = 200
   * 1 ms period --> TCCR1B = 0x0A, OCR1A = 2,000 --- NOTE: it was determined calcs take ~ 600 us
   * 10 ms period --> TCCR1B = 0x0A, OCR1A = 20,000
   * 100 ms period --> TCCR1B = 0x0B, OCR1A = 25,000
   */
  OCR1A = 2000;  // Recommended control period is 1 ms
  DDRF &= ~(1 << 5);  // Input for begin-control button (enable Timer1 interrupt)
  PORTF |= 1 << 5;  // Enable pull-up resistor
  
  // I/O for 7-segment
  DDRC |= 0xFB;  // Outputs for 7-segment anodes
  DDRF |= 0x0F;  // Outputs for 7-segment cathodes and DP

  // Timer configuration for 7-segment display
  TCCR0A = 0x02;  // CTC mode
  TCCR0B = 0x05;  // Prescaler of 1024
  OCR0A = 50;

  // Use interrupt pins for tracking rod position
  /*
   * NEED TO CHANGE THIS TO LOGIC CHANGE !!!!!!!!!
   */
  EICRA |= 0x01;  // Interrupt on logic change of INT0
  EICRA |= 0x04;  // Interrupt on logic change of INT1

  // Enable interrupts (TIMSK1 set in polling branch for start button, see main())
  if (is_displaying_encoder_count)
    TIMSK0 = 0x02;  // OCIE0A for 7-segment display
  EIMSK |= 0x01;  // INT0 for encoder count
  EIMSK |= 0x02;  // INT1 for encoder count
  sei();
  
  int32_t temp_count;  // This is for 7-segment display

  /*
   * Discretized state matrices, gains, etc.
   */
  BLA::Matrix<1, 3> C = {1.0, 0.0, 0.0};

  /*


K_d =

  Columns 1 through 2

   -261.071324746105e+000   -15.4739066468442e+000

  Column 3

   -183.817899275421e-003


K_e =

    522.763970713245e-003
    39.2276303111702e+000
   -859.244693600951e+000


>> pd_obs.A

ans =

  Columns 1 through 2

    1.00012270503254e+000    999.985608556819e-006
    245.321833902138e-003    1.00001214339497e+000
   -223.191420032679e-003   -14.6223411579042e-006

  Column 3

    1.11963211232329e-006
    2.16901094485479e-003
    823.267618653811e-003

>> pd_obs.B

ans =

   -21.2401033875826e-006
   -41.1474592506252e-003
    3.35272096111766e+000 
  */
  // --- Observer values from MATLAB script ---
  BLA::Matrix<3,3> G_obs = {1.0, 999.986E-6, 1.11963E-6,
			    0.245322, 1.0, 2.16901E-3,
			    -0.22319, -14.62234E-6, 823.2676E-3};  // Discrete state matrix
  BLA::Matrix<3,1> H_obs = {-21.2401E-6,
			    -41.14746E-3,
			    3.35272};  // Discrete input matrix

  BLA::Matrix<3, 1> K_e = {522.76397E-3, 39.22763, -859.24469};  // Calculated via MATLAB script
  BLA::Matrix<3,1> x_hat_current = {0.0, 0.0, 0.0};  // Estimated state vector
  BLA::Matrix<3,1> x_hat_prev = {0.0, 0.0, 0.0};  // Previous estimate

  // --- Controller values from MATLAB script ---
  BLA::Matrix<1,3> K_d = {-261.0713, -15.47391, -0.183818};  // Gains computed in MATLAB for regulator
  BLA::Matrix<1,1> u = {0};  // Computed control action u = -K x_k

  // Just used for matrix algebra steps (see observer update below)
  BLA::Matrix<3,1> temp_m1;  
  BLA::Matrix<3,1> temp_m2;
  BLA::Matrix<1,1> temp_m3;
  BLA::Matrix<1,1> temp_m4;
  BLA::Matrix<1,1> temp_m5;
  BLA::Matrix<3,1> temp_m6;
  BLA::Matrix<3,1> temp_m7;
  
  float desired_control_action, temp_OCR2A;  // Used in computing duty cycle
  uint8_t has_initialized_x_hat = 0;  // Flag for initializing state estimate
  uint16_t log_index = 0;  // Counter for data logging
  
  while (1) {

    theta_1 = -2 * pi * current_count / ticks_per_rev;  // The encoder is mounted backwards on IFP rev2
    
    if (my_timer_flag) {  // Ready for control cycle

      if (!has_initialized_x_hat) {
	has_initialized_x_hat = 1;
	x_hat_current(0, 0) = theta_1;  // Start with \hat{x}_k = [theta_1; 0; 0]
	x_hat_prev = x_hat_current;  // Start with \hat{x}_{k-1} = [theta_1; 0; 0]

      }

      // Compute control action
      BLA::Multiply(K_d, x_hat_current, u);
      desired_control_action = -u(0, 0);  // u = -K x_k

      // Direction for motor
      if (0 > desired_control_action) {
	PORTB |= 1 << 5;  // Reverse
	desired_control_action = -desired_control_action;  // Get positive val. to compute duty cycle
    
      } else
	PORTB &= ~(1 << 5);  // Forward
    
      // PWM for motor
      temp_OCR2A = 255.0 * desired_control_action / max_motor_voltage + dead_band;
      if (255 < temp_OCR2A) {  // temp_OCR2A is positive because of direction branch above
	OCR2A = 255;

      } else
	OCR2A = temp_OCR2A;

      // For subsequent observer updates, store actual input not ideal computed
      u(0, 0) = OCR2A * quantization_e_in;
      if (dead_band == OCR2A)
	u(0, 0) = 0;
	
      //if (!(PORTB & (1 << 5)))  // This is a way of keeping track of if desired_control_action has undergone sign change
      if (PORTB & (1 << 5))  // This is a way of keeping track of if desired_control_action has undergone sign change
	u(0, 0) = -u(0, 0);

      // Store values for data logging
      if (num_log_values > log_index) {
	log_encoder_count[log_index] = (int) current_count;
	if (is_logging_more_than_just_encoder_count) {
	  log_OCR2A[log_index] = OCR2A;
	  log_PB5[log_index] = PORTB & (1 << 5);
	}
	/*
	log_theta_1[log_index] = theta_1;
	log_x_hat_1[log_index] = x_hat_current(0, 0);
	log_x_hat_2[log_index] = x_hat_current(1, 0);
	log_x_hat_3[log_index] = x_hat_current(2, 0);
	log_control_action[log_index] = u(0, 0);
	*/
	++log_index;
  
      } else
	PORTB |= 1 << 7;  // Turn on built-in LED to show array is full
    
      // Estimate state vector using predictive observer
      BLA::Multiply(G_obs, x_hat_prev, temp_m1);
      BLA::Multiply(H_obs, u, temp_m2);
      BLA::Multiply(C, x_hat_prev, temp_m3);
      temp_m4(0, 0) = theta_1;
      BLA::Subtract(temp_m4, temp_m3, temp_m5);
      BLA::Multiply(K_e, temp_m5, temp_m6);
      BLA::Add(temp_m1, temp_m2, temp_m7);
      BLA::Add(temp_m7, temp_m6, x_hat_current);  // Updated state estimate
      //x_hat_current(0, 0) = theta_1;  // This should necessitate a partial state observer!!!!!  REVISIT -- 200609
      x_hat_prev = x_hat_current;  // Store estimate for next iteration
      
      my_timer_flag = 0;  // Reset control-loop flag
  
    }  // End controller/observer branch

    // Monitor button for when to start timer that governs control and data logging
    if (!(PINF & (1 << 5))) {  // Button pressed
      PORTF |= 1 << 4;  // Turn on LED to show control is activated
      TIMSK1 = 1 << 1;  // OCIE1A for control loop
      
    }
    
    // Monitor button for when to write data to serial
    if (!(PINB & (1 << 6)))  // Button pressed
      serial_print_log();
    
    // Get digits of current_count for display
    if (is_displaying_encoder_count) {
      if (0 > current_count) {
	temp_count = -current_count;
	PORTF |= 1 << 3;  // Turn on DP to indicate negative
      
      } else {
	temp_count = current_count;
	PORTF &= ~(1 << 3);  // Turn of DP

      }
      // --- Display different values on seven-segment ---
      //temp_count = OCR2A;  // See what PWM pin is doing
      //temp_count = 1000.0 * 180.0 * theta_1 / pi;  // Display theta_1 in degrees
      //temp_count = 1000.0 * theta_1;  // Display theta_1 in radians
      //if (0 > current_count)
      //temp_count = -temp_count;
      
      ones = temp_count % 10;
      temp_count = temp_count / 10;
      tens = temp_count % 10;
      temp_count = temp_count / 10;
      hundreds = temp_count % 10;
      
    }  // End branch for is_displaying_encoder_count
    
  }  // End while(1)
  
}  // End main()


ISR(TIMER1_COMPA_vect) {  // ISR for control-loop period
  my_timer_flag = 1;
}


ISR(TIMER0_COMPA_vect) {

  if (0x03 == (PORTF & 0x07)) {
    PORTC = my_array[tens];
    PORTF &= ~0x07;
    PORTF |= 0x05;

  } else if (0x05 == (PORTF & 0x07)) {
    PORTC = my_array[ones];
    PORTF &= ~0x07;
    PORTF |= 0x06;

  } else {
    PORTC = my_array[hundreds];
    PORTF &= ~0x07;
    PORTF |= 0x03;

  }
}


ISR(INT0_vect) {

  if (PIND & (1 << 0)) {  // Check Channel A
    
    if (PIND & (1 << 1))  // Check Channel B
      ++current_count;  // CCW rotation
  
    else
      --current_count;  // CW rotation

  } else {
    
    if (PIND & (1 << 1))  // Check Channel B
      --current_count;  // CW rotation
  
    else
      ++current_count;  // CCW rotation
    
  }
}



ISR(INT1_vect) {

  if (PIND & (1 << 1)) {  // Check Channel B
    
    if (PIND & (1 << 0))  // Check Channel A
      --current_count;  // CW rotation
  
    else
      ++current_count;  // CCW rotation

  } else {
    
    if (PIND & (1 << 0))  // Check Channel A
      ++current_count;  // CCW rotation
  
    else
      --current_count;  // CW rotation

  }
}

/*
 * !!!
 * Be sure encoder and motor defined with positive same direction!!!
 * !!!
 */
