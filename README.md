/*

 * MOTION TUNING GUIDE 

 * 1. HOW MICROSTEPPING WORKS & STEPS PER REVOLUTION
 * --------------------------------------------------------------------------------------
 * - A standard NEMA 23 stepper motor has a 1.8° step angle.
 * - Full steps per revolution = 360° / 1.8° = 200 full steps.
 * - Microstepping divides each full step into smaller sub-steps by varying current 
 *   between the two motor coils smoothly. This reduces noise and increases precision.
 * 
 *   Formula: STEPS_PER_REV = 200 * MICROSTEPS
 * 
 *   TB6600 Switch Reference Table:
 *   - 1/1  (Full Step) -> 200 * 1  = 200 steps/rev
 *   - 1/2  (Half Step) -> 200 * 2  = 400 steps/rev
 *   - 1/4  Microstep   -> 200 * 4  = 800 steps/rev
 *   - 1/8  Microstep   -> 200 * 8  = 1600 steps/rev
 *   - 1/16 Microstep   -> 200 * 16 = 3200 steps/rev
 *   - 1/32 Microstep   -> 200 * 32 = 6400 steps/rev (Current Setting)
 * 
 * 
 * 2. WHAT TO CHANGE IF YOU CHANGE MICROSTEPPING ON THE TB6600
 * --------------------------------------------------------------------------------------
 * If you flip the DIP switches on your TB6600 (e.g., from 1/32 down to 1/16 or 1/8):
 *   A) Update `const int MICROSTEPS` in the code to match the switch setting.
 *   B) Adjust `STEP_INTERVAL_MIN_US` proportionally:
 *      - Lower microstepping requires FEWER pulses per second for the same RPM.
 *      - Example: At 1/32 (6400 steps/rev), 100 µs = ~93 RPM.
 *      - If you switch to 1/16 (3200 steps/rev), 100 µs will double your speed to ~187 RPM!
 *        Increase `STEP_INTERVAL_MIN_US` to 200 µs if you want to keep the same ~93 RPM.
 *   C) Update your Auto Mode target coordinates:
 *      - Halving microstepping means the motor travels twice as far per step.
 * 
 * 
 * 3. HOW TO CALCULATE & INCREASE RPM (SPEED MAPPING)
 * --------------------------------------------------------------------------------------
 * Speed is controlled by the time delay between step pulses (`STEP_INTERVAL_MIN_US`).
 * - To INCREASE RPM -> DECREASE `STEP_INTERVAL_MIN_US` (shorter delay = faster pulses).
 * - To DECREASE RPM -> INCREASE `STEP_INTERVAL_MIN_US` (longer delay = slower pulses).
 * 
 *   Exact RPM Formula:
 *   RPM = (60,000,000 / STEP_INTERVAL_MIN_US) / STEPS_PER_REV
 * 
 *   Speed Mapping Table (At 1/32 Microstepping = 6400 steps/rev):
 *   - 800 µs ->  11.7 RPM (Startup speed - high torque, zero step skipping)
 *   - 400 µs ->  23.4 RPM (Slow, precise positioning)
 *   - 200 µs ->  46.8 RPM (Medium-slow speed)
 *   - 100 µs ->  93.7 RPM (Current cruise speed - safe for 3 simultaneous axes)
 *   -  70 µs -> 133.9 RPM (Fast single-axis speed)
 *   -  40 µs -> 234.3 RPM (Very fast - near ESP32 software loop limit)
 * 
 *   *Note: If you set `STEP_INTERVAL_MIN_US` too low (< 40 µs), the motor will stall 
 *   and buzz because the rotor cannot accelerate fast enough to keep up with pulses.
 * 
 * 
 * 4. HOW TO ADJUST EVERY CONSTANT IN THIS PROGRAM
 * --------------------------------------------------------------------------------------
 * - PUL_PINS / DIR_PINS:
 *   Change these GPIO numbers if you rewire the TB6600 PUL+ or DIR+ cables to 
 *   different pins on the ESP32.
 * 
 * - STEP_INTERVAL_START_US (Default: 800):
 *   Controls the initial speed the very millisecond you press a button.
 *   -> Increase (e.g., 1200) if the motor stalls/clicks right at startup under heavy load.
 *   -> Decrease (e.g., 500) for snappier button response on lightweight axes.
 * 
 * - STEP_INTERVAL_MIN_US (Default: 100):
 *   Controls the maximum top speed (cruise speed) after acceleration finishes.
 *   -> Decrease to go faster; Increase to go slower.
 * 
 * - STEP_INTERVAL_ACCEL_US (Default: 2):
 *   Controls how aggressively the motor ramps from START speed to MIN (top) speed.
 *   -> Higher value (e.g., 5) = Faster acceleration (reaches top speed quickly).
 *   -> Lower value (e.g., 1)  = Gentler, smoother ramp (prevents vibration/stalling).
 * 
 * - PULSE_WIDTH_US (Default: 2):
 *   The duration (in microseconds) the PUL pin stays HIGH per step.
 *   -> Keep at 2 to 5 µs. TB6600 optocouplers require at least 1–2 µs to register a pulse.
 * 
 * - SOLDER_DWELL_MS (Default: 1500):
 *   How long (in milliseconds) the Z-axis stays down at each point in Auto Mode.
 *   -> Increase (e.g., 2500) for larger solder joints that need more heat transfer.
 *   -> Decrease (e.g., 800) for tiny SMD pads so you don't overheat the PCB.
 * 
 * - MAX_POINTS (Default: 50):
 *   Maximum number of coordinate rows allowed in the Auto Mode web table.
 *   -> Increase if you are soldering large boards with more than 50 joints.
 * 
 * ======================================================================================
 */
