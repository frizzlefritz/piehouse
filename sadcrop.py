##############  SADCROP 420 V1  ##############

import  RPi.GPIO as GPIO
import  adafruit_dht
from    MCP3008 import MCP3008
import  time
GPIO.setmode(GPIO.BOARD) ##### DESIGNATE BCM pin-numbering scheme 
import time
import board
import adafruit_dht


################################################################## Edit to reflect the true settings in application. 
##################### CUSTOMIZEABLE SETTINGS #####################  Edits will be made for each of the cycles DOWN LINE
##################################################################
SETTINGS = {
    "LIGHT_GPIO":       17,                     # GPIO Number (BCM) for the Relay
    "LIGHT_FROM":       10,                     # from which time the light can be turned on (hour)
    "LIGHT_UNTIL":      20,                     # until which time (hour)
    "LIGHT_CHANNEL":    0,                      # of MCP3008
    "HUMID_THRESHOLD":  500,                    # if the analog Threshold is below any of those, the light will turn on
    "DHT_GPIO":         4,                      # GPIO Number (BCM) of the DHT Sensor
    "DHT_SENSOR":       adafruit_dht.DHT11,     # DHT11 or DHT22
    "TEMP_THRESHOLD":   23.0,                   # in Celcius. Above this value, intake air
    "SERVO_GPIO":       22,                     # GPIO Number (BCM), which opens the window // may not use
    "SERVO_OPEN_ANGLE": 90.0,                   # degree, how much the servo will open the window // may not use
    "FAN_GPIO":          
    
    "PCF_8591":         

    "PLANTS": [
        {
            "NAME":                 "POT",
            "MOISTURE_CHANNELS":    [0, 1],     # of ADC
            "MOISTURE_THRESHOLD":   450,        # if the average analog value of all sensors is above of this threshold, the Pump will turn on
            "WATER_PUMP_GPIO":      23,         # GPIO Number (BCM) for the Relais
            "WATERING_TIME":        10,         # Seconds, how long the pump should be turned on
        },
        {
            "NAME":                 "Salat",
            "MOISTURE_CHANNELS":    [3, 4],
            "MOISTURE_THRESHOLD":   450,
            "PUMP_GPIO":            24,
            "WATERING_TIME":        12,
        }
    ]
}
##################################################################
################# END OF CUSTOMIZEABLE SETTINGS ##################
##################################################################
dhtDevice = adafruit_dht.DHT11(board.D4) 
while True:
    try:
        # Print the values to the serial port
        temperature_c = dhtDevice.temperature
        temperature_f = temperature_c * (9 / 5) + 32
        humidity = dhtDevice.humidity
        print(
            "Temp: {:.1f} F / {:.1f} C    Humidity: {}% ".format(
                temperature_f, temperature_c, humidity
            )
        )
 
    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])
        time.sleep(2.0)
        continue
    except Exception as error:
        dhtDevice.exit()
        raise error
time.sleep(3)


##########################  lcd display  #########################
import smbus
import time

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address, if any error, change this address to 0x3f
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line
LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def main():
  # Main program block

  # Initialise display
  lcd_init()

  while True:

    # Send some test
    lcd_string("welcome to      ",LCD_LINE_1)
    lcd_string("SADCROP 420    ",LCD_LINE_2)

    time.sleep(3)
  
    # Send some more text
    lcd_string("(
            "Temp: {:.1f} F / {:.1f} C    Humidity: {}% ".format(
                temperature_f, temperature_c, humidity)",LCD_LINE_1)
    lcd_string("I2C 1602        ",LCD_LINE_2)

    time.sleep(3)

if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)

################
def get_moisture()                      ##### add pcf settings file
     adc = PCF8591()
 
def get_last_watered():
    try:
        f = open("last_watered.txt", "r" #r/read w/write a/append
        return f.readline()
    except:
        return "NO FILE FOUND!"


def get_status(pin = 8):
    GPIO.setup(pin, GPIO.IN) 
    return GPIO.input(pin)

def init_output(pin):
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    GPIO.output(pin, GPIO.HIGH)
    
def auto_water(delay = 5, pump_pin = 7, water_sensor_pin = 8): #changing dlay changeduration
    consecutive_water_count = 0
    init_output(pump_pin)
    print("Here we go! Press CTRL+C to exit")
    try:
        while 1 and consecutive_water_count < 10:
            time.sleep(delay)
            wet = get_status(pin = water_sensor_pin) == 0
            if not wet:
                if consecutive_water_count < 5:
                    pump_on(pump_pin, 1)
                consecutive_water_count += 1
            else:
                consecutive_water_count = 0
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        GPIO.cleanup() # cleanup all GPI

def pump_on(pump_pin = 7, delay = 1):
    init_output(pump_pin)
    f = open("last_watered.txt", "a")
    f.write("Last watered {}".format(datetime.datetime.now()))
    f.close()
    GPIO.output(pump_pin, GPIO.LOW)
    time.sleep(1)
    GPIO.output(pump_pin, GPIO.HIGH)
    ####end of check
  
def wateringPlants():
    ###### READING MOISTURE LEVELS #####
    adc = PCF8591()
    for plantObject in SETTINGS["PLANTS"]:
        value = 0
        for ch in plantObject["MOISTURE_CHANNELS"]:
            # read 10 times to avoid measuring errors
            v = 0
            for i in range(10):
                v += adc.read( channel = ch )
            v /= 10.0
            value += v
        
        value /= float(len(plantObject["MOISTURE_CHANNELS"]))
        
        if value > plantObject["MOISTURE_THRESHOLD"]:
            # turn pump on for some seconds
            GPIO.setup(plantObject["WATER_PUMP_GPIO"], GPIO.OUT, initial=GPIO.LOW)
            time.sleep(plantObject["WATERING_TIME"])
            GPIO.output(plantObject["WATER_PUMP_GPIO"], GPIO.HIGH)
            
def checkWindow():
    # read remperature
    humidity, temperature = Adafruit_DHT.read_retry(SETTINGS["DHT_SENSOR"], SETTINGS["DHT_GPIO"])
    
   # GPIO.setup(SETTINGS["SERVO_GPIO"], GPIO.OUT)
   # pwm = GPIO.PWM(SETTINGS["SERVO_GPIO"], 50)
    
    if temperature > SETTINGS["TEMP_THRESHOLD"]:
        #####Change to fan engage

    else:
        # close window
        pwm.start(2.5)
    # save current
    time.sleep(2)
    pwm.ChangeDutyCycle(0)           
            
            
            
