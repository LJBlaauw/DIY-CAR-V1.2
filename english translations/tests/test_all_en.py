from servo_crl import ServoController
import stepper
import ldr_scan_isr as ldr
import time
import ultrasoon
import rp2

sc = ServoController()          # all servos are immediately at their rest positions
# Current limit during position control:
sc.servo_cur_limit(200)

# Everything to rest with sequence 4 -> 1 -> 2 -> 3, @ 20 °/s:
#sc.servo_rest(1,20)
#sc.servo_rest()
stepper.reset_PIO_distance()
stepper.status()
stepper.disable()
#stepper.status()

# Move relative to rest (never below rest):
sc.servo_pos(2, 30, 20)         # Servo 2 to rest+30° 20°/sec
stepper.rotate('l',10, 30)		# rotate left, speed= 10cm/sec, distance=30cm
while stepper.sm0.active() or stepper.sm1.active(): # wait until the stepper (rotation) is finished
    pass
sc.servo_pos(1, 20, 20)
stepper.mov('b', 5, 10)			# move backwards speed=5cm/sec, distance=10cm
while stepper.sm0.active() or stepper.sm1.active():
    pass
sc.servo_pos(4, 90, 20)         # gripper: rest+15°
sc.laser_power(10)				# set laser to 10% output
sc.servo_pos(2, 50, 10);sc.servo_pos(1, 50, 10)
stepper.s1('f',1,10)			# only stepper 1 forward 1cm/sec, 10 cm
while stepper.sm0.active() or stepper.sm1.active():
    pass
sc.laser_power(50)

# sc.servo_cur(100, 60)# current mA, degrees/sec
# Current-SETPOINT (position ignored for servo 4):
# sc.servo_cur(100, 60)

sc.servo_pos(4, 60, 20)
stepper.distance()
ldr.attach_stepper_reader(stepper.pio_pos1)
print(ldr.measure_now())
print('start scan')

res = ldr.scan('l', 5.0, 100.0, start_graden=10.0,
               go_max=True,
               excel=True,
               out_csv="/scan.csv")
print(res)
sc.servo_rest()

#ultrasoon.reset_ultrasoon()

try:
    while True:
        cm, kind = ultrasoon.read_cm()
        if kind == 'ok':
            print("Distance:", f"{cm:.1f} cm", "LDR", (ldr.measure_now()))
        elif kind == 'overflow':
            print("Max/overflow:", f"{cm:.1f} cm")
        else:
            print("Timeout: no echo")
        time.sleep_ms(100)
except KeyboardInterrupt:
    ultrasoon.stop()
    print("Stopped.")
