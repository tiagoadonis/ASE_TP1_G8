import RPi.GPIO as GPIO
import time

# Global configurations
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Ultrasonic configuration
TRIG = 18 
ECHO = 24
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

# LED configuration
LED_PIN = 8
GPIO.setup(LED_PIN, GPIO.OUT)

print("Distance Measurement In Progress")

try:
  while True:
    # Force the trigger to false
    GPIO.output(TRIG, False)
    print("Waiting For Sensor To Settle")
    
    # TODO check this sleep time
    # sleep for 2s
    time.sleep(2)

    # Init the trigger
    GPIO.output(TRIG, True)
    # sleep for 0,01s
    time.sleep(0.00001)

    # Close the trigger
    GPIO.output(TRIG, False)

    # Save the start time
    while GPIO.input(ECHO) == 0:
      startTime = time.time()

    # Get the end time
    while GPIO.input(ECHO) == 1:
      endTime = time.time()

    # Calculate the time elapsed 
    timeElapsed = endTime - startTime

    # Calculate the distance: 
    # - multiple by the sonic speed (34300 cm/s)
    # - divide by 2, the going and coming path of the pulse
    distance = (timeElapsed * 34300) / 2

    # Round the distance value with two decimal number
    distance = round(distance, 2)
    
    # If the distance is bigger than 15cm the led changes to high level
    if(distance > 15):
      GPIO.output(LED_PIN, GPIO.HIGH)
      # sleep for 1s
      time.sleep(1)
    # else the led stays on low
    else:
      GPIO.output(LED_PIN, GPIO.LOW)
      # sleep for 1s
      time.sleep(1)

    #TODO try this code block to blink with the distance
    # GPIO.output(LED_PIN, GPIO.HIGH)
    # time.sleep(distance * 0.02)
    # GPIO.output(LED_PIN, GPIO.LOW)
    # time.sleep(distance * 0.02)

    # Print the distance value in cm
    print("Distance: ", distance, "cm")

  GPIO.cleanup()

# If there is a KeyboardInterrupt (when you press ctrl+c), exit the program and cleanup
except KeyboardInterrupt: 
    print("Cleaning up!")
    gpio.cleanup()