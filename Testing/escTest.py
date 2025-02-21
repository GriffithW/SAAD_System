from gpiozero import PWMOutputDevice
from time import sleep

class ZMREsc:
    def __init__(self, pin, frequency=50):
        """
        Initialize the ZMR ESC for standard range calibration.

        Parameters:
        - pin: GPIO pin connected to the ESC signal wire.
        - frequency: PWM frequency, typically 50 Hz for ESCs.
        """
        self.esc = PWMOutputDevice(pin, frequency=frequency)
        self.min_pwm = 0.05  # Minimum PWM (1000 µs)
        self.max_pwm = 0.1   # Maximum PWM (2000 µs)

    def set_pwm(self, value):
        """
        Set a raw PWM value to the ESC.

        Parameters:
        - value: Float between 0.0 and 1.0 (maps to GPIOZero's range).
        """
        if 0.0 <= value <= 1.0:
            print(f"Setting PWM to {value:.4f}")
            self.esc.value = value
        else:
            raise ValueError("PWM value must be between 0.0 and 1.0.")

    def calibrate(self):
        """
        Calibrate the ESC by setting maximum and minimum throttle values.
        """
        print("Starting ESC calibration...")

        # Step 1: Full throttle
        print("Step 1: Setting to maximum throttle. Plug in the battery now.")
        self.set_pwm(0.01)  # Send maximum PWM (2000 µs)
        sleep(3)

        # self.set_pwm(self.max_pwm)  # Send maximum PWM (2000 µs)
        # input("Press enter:")

        # # Step 2: Minimum throttle
        # print("Step 2: Setting to minimum throttle.")
        # self.set_pwm(self.min_pwm)  # Send minimum PWM (1000 µs)
        # input("Press enter:")

        # # Step 3: Neutral or idle position (optional)
        # print("Step 3: Setting to neutral position.")
        # self.set_pwm(0.072)  # Neutral (1500 µs)
        # input("Press enter:")

        # print("ESC calibration complete. Ensure the range is fixed.")

    def cleanup(self):
        """Release the GPIO pin."""
        self.esc.close()
        print("ESC control released.")

# Usage
if __name__ == "__main__":
    esc_pin = 13  # GPIO pin connected to the ESC
    esc_pin2 = 12  # GPIO pin connected to the ESC
    esc = ZMREsc(pin=esc_pin)
    esc2 = ZMREsc(pin=esc_pin2)

    while(1):
        speed = float(input("Press enter:"))
        esc.set_pwm(speed)
        esc2.set_pwm(speed)



    try:
        esc.calibrate()
    except Exception as e:
        print(f"Error during calibration: {e}")
    finally:
        esc.cleanup()
