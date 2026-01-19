# test_ultrasoon.py
import time
import ultrasoon

try:
    while True:
        cm, kind = ultrasoon.read_cm()
        if kind == 'ok':
            print("Afstand:", f"{cm:.1f} cm")
        elif kind == 'overflow':
            print("Max/overflow:", f"{cm:.1f} cm")
        else:
            print("Timeout: geen echo")
        time.sleep_ms(100)
except KeyboardInterrupt:
    ultrasoon.stop()
    print("Gestopt.")

