import json
from time import sleep

import RPi.GPIO as GPIO
import Mfrc522
import signal

continue_reading = True


def end_read(signal, frame):
    global continue_reading
    print("Exit!")
    continue_reading = False
    GPIO.cleanup()


signal.signal(signal.SIGINT, end_read)

reader_device = Mfrc522.Mfrc522()

# Loop Principal
while continue_reading:
    # Check Tag
    (status, tag_type) = reader_device.request(reader_device.PICC_REQIDL)

    # Tag Found
    #if (status == reader_device.OK):
    #    print("Tag Found")

    # get UID
    (status, uid) = reader_device.anticollision()
    card = {}

    # Dumping
    if status == reader_device.OK:
       #Card Data
        card = {"UID": uid}

        # Key
        key = [0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7]

        # Select Tag
        reader_device.select_tag(uid)

        #Read Card Data
        for block in range(64):
            if block == 0 or block % 4 == 0:
                reader_device.auth(reader_device.PICC_AUTHENT1B, block, key, uid)
            card[block] = reader_device.read_data(block)

        #Export data to Json
        print(json.dumps(card))
        reader_device.stop_crypto()

        #Sleep 5 seconds
        sleep(5)
