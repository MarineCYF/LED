import RPi.GPIO as GPIO
import board 
import neopixel
import time
import threading
from rpi_ws281x import *
import argparse

# Sensor pin setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Distance Sensor Setup
TRIG = 23
ECHO = 24

#IR Sensor Setup
IRsensor = 2
GPIO.setup(IRsensor,GPIO.IN)

#Sensor Distance-Value  
def distance():
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)

    GPIO.output(TRIG,False)
    time.sleep(0.00001)
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)
    
    pulse_start = 0
    pulse_end = 0
    
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration*17150
    distance = round(distance,2)

    return int(distance)

#led setup:
# LED strip configuration:
LED_COUNT      = 150      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 50     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

# Define functions which animate LEDs in various ways.
def colorWipe_open(strip, color, wait_ms=10):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)    

def colorWipe_close(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)

def color():
    if GPIO.input(IRsensor) == 0:
        colorWipe_open(strip, Color(0, 0, 255))
    else:
        colorWipe_close(strip, Color(0, 0, 0))
def IR_sensor():
    Value = GPIO.input(IRsensor)
    print(Value)
threads = []
t1 = threading.Thread(target=IR_sensor)
threads.append(t1)
t2 = threading.Thread(target=color)
threads.append(t2)

if __name__ == '__main__':
    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()
    while True:
        for t in threads:
             t.setDaemon(True)
             t.start()
    GPIO.cleanup()

