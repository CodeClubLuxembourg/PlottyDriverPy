#!/usr/bin/python3
import RPi.GPIO as GPIO
import math
import pigpio
import random
import socket
import threading
import time
import sys
import json
import socketio
import requests

gondola = False
f = open( "/var/mode", "r" )
mode = f.read()
f.close()
if mode.strip()=="gondola":
    gondola = True
gondola_calibration_length = 111.3
gondola_steps_per_cm = 402
gondola_reserve_margin = 20.0 # units
gondola_max_travel = 1.0 # units

stepper_bottom_enable = 26
stepper_bottom_step = 19
stepper_bottom_dir = 13
stepper_bottom_ms1 = 6
stepper_bottom_ms2 = 5

stepper_top_enable = 21
stepper_top_step = 20
stepper_top_dir = 16
stepper_top_ms1 = 12
stepper_top_ms2 = 1

limit_switch_top = 17
limit_switch_bottom = 4
limit_switch_left = 3
limit_switch_right = 2

servo = 23
pen_up_or_down = True # True for down, False for up
cw = True
ccw = False
current_x = False
current_y = False
canvas_max_x_steps = 0
canvas_max_y_steps = 0
canvas_max_x = False
canvas_max_y = False
rotate_to_maximize_drawing = False # we now do this on the front end
get_out_of_the_way_after_draw = False

pen_down_pulse_width_default = 500
pen_up_pulse_width_default = 1100
pen_down_action_time_default = 300 # milliseconds
pen_up_action_time_default = 300 # milliseconds
pen_down_sleep_before_move_time_default = 150 # milliseconds
pen_up_sleep_before_move_time_default = 0 # milliseconds
default_step_sleep_default = 0.0002
if gondola:
    default_step_sleep_default = 0.0016
# TODO need to figure out how to change acceleration based on angle
acceleration_steps_default = 100 # 100
if gondola:
    acceleration_steps_default = 160
deceleration_steps_default = 20 # 20
if gondola:
    deceleration_steps_default = 80
acceleration_slow_steps_sleep_default = 0.0005
if gondola:
    acceleration_slow_steps_sleep_default = 0.004
acceleration_fast_steps_sleep_default = 0.0002
if gondola:
    acceleration_fast_steps_sleep_default = 0.0018

pen_down_pulse_width = pen_down_pulse_width_default
pen_up_pulse_width = pen_up_pulse_width_default
pen_down_action_time = pen_down_action_time_default
pen_up_action_time = pen_up_action_time_default
pen_down_sleep_before_move_time = pen_down_sleep_before_move_time_default
pen_up_sleep_before_move_time = pen_up_sleep_before_move_time_default
default_step_sleep = default_step_sleep_default
acceleration_steps = acceleration_steps_default
deceleration_steps = deceleration_steps_default
acceleration_slow_steps_sleep = acceleration_slow_steps_sleep_default
acceleration_fast_steps_sleep = acceleration_fast_steps_sleep_default

x_steps_skew = 0.0
y_steps_skew = 0.0
current_pen_pulse_width = 0

GPIO.setwarnings( False )
GPIO.setmode( GPIO.BCM )

GPIO.setup( stepper_bottom_enable, GPIO.OUT )
GPIO.output( stepper_bottom_enable, 0 )
GPIO.setup( stepper_bottom_step, GPIO.OUT )
GPIO.output( stepper_bottom_step, 0 )
GPIO.setup( stepper_bottom_dir, GPIO.OUT )
GPIO.output( stepper_bottom_dir, 0 )
GPIO.setup( stepper_bottom_ms1, GPIO.OUT )
GPIO.output( stepper_bottom_ms1, 0 )
GPIO.setup( stepper_bottom_ms2, GPIO.OUT )
GPIO.output( stepper_bottom_ms2, 0 )

GPIO.setup( stepper_top_enable, GPIO.OUT )
GPIO.output( stepper_top_enable, 0 )
GPIO.setup( stepper_top_step, GPIO.OUT )
GPIO.output( stepper_top_step, 0 )
GPIO.setup( stepper_top_dir, GPIO.OUT )
GPIO.output( stepper_top_dir, 0 )
GPIO.setup( stepper_top_ms1, GPIO.OUT )
GPIO.output( stepper_top_ms1, 0 )
GPIO.setup( stepper_top_ms2, GPIO.OUT )
GPIO.output( stepper_top_ms2, 0 )

GPIO.setup( limit_switch_top, GPIO.IN, pull_up_down=GPIO.PUD_UP )
GPIO.setup( limit_switch_bottom, GPIO.IN, pull_up_down=GPIO.PUD_UP )
GPIO.setup( limit_switch_left, GPIO.IN, pull_up_down=GPIO.PUD_UP )
GPIO.setup( limit_switch_right, GPIO.IN, pull_up_down=GPIO.PUD_UP )

GPIO.setup( servo, GPIO.OUT )
pwm = pigpio.pi()
pwm.set_mode( servo, pigpio.OUTPUT )
pwm.set_PWM_frequency( servo, 50 )
pwm.set_servo_pulsewidth( servo, 0 )
#pwm.gpioDelay( 500 ) # todo not the right pigpio library version?
#current_pen_pulse_width = pen_up_pulse_width

current_step_sleep = 1.0 # something really high so it'll be obvious if this ever isn't set properly later

step_size = 1.0/8 # possible step sizes 1.0, 1.0/2, 1.0/4, 1.0/8
def set_step_size( new_step_size ):
    global step_size, default_step_sleep

    step_size = new_step_size

    if step_size==1.0:
        GPIO.output( stepper_top_ms1, 0 )
        GPIO.output( stepper_top_ms2, 0 )
        GPIO.output( stepper_bottom_ms1, 0 )
        GPIO.output( stepper_bottom_ms2, 0 )
        default_step_sleep = default_step_sleep_default * 8
    elif step_size==1.0/2:
        GPIO.output( stepper_top_ms1, 1 )
        GPIO.output( stepper_top_ms2, 0 )
        GPIO.output( stepper_bottom_ms1, 1 )
        GPIO.output( stepper_bottom_ms2, 0 )
        default_step_sleep = default_step_sleep_default * 4
    elif step_size==1.0/4:
        GPIO.output( stepper_top_ms1, 0 )
        GPIO.output( stepper_top_ms2, 1 )
        GPIO.output( stepper_bottom_ms1, 0 )
        GPIO.output( stepper_bottom_ms2, 1 )
        default_step_sleep = default_step_sleep_default * 2
    elif step_size==1.0/8:
        GPIO.output( stepper_top_ms1, 1 )
        GPIO.output( stepper_top_ms2, 1 )
        GPIO.output( stepper_bottom_ms1, 1 )
        GPIO.output( stepper_bottom_ms2, 1 )
        default_step_sleep = default_step_sleep_default
    else:
        print( "> error: unknown step_size: " + str(step_size) )
        exit( 1 )
set_step_size( step_size )

calibration_ongoing = False
calibration_done = False
safety_step_padding = 50

command_server_thread = False
command_server_server = False
command_server_client = False
command_server_address = "127.0.0.1"
command_server_port = 1337
command_running = False

calibrate_manually_step_down = False
calibrate_manually_step_up = False
calibrate_manually_step_left = False
calibrate_manually_step_right = False
calibrate_manually_down_reached = False
calibrate_manually_up_reached = False
calibrate_manually_left_reached = False
calibrate_manually_right_reached = False

draw_going = False
pause_draw = False
stop_draw = False

clean_up_before_exit = False

mg_link_session_id = ""
mg_link_socket = False
mg_penstrokes = []
mg_penstrokes_processing_thread = False
mg_penstrokes_processing_thread_stop = False
mg_canvas_width = False
mg_canvas_height = False

ht_live_keyboard_on = False
ht_penstrokes = []
ht_penstrokes_processing_thread = False
ht_penstrokes_processing_thread_stop = False
ht_penstrokes_mutex = False

ink_refill_routine_enabled_default = False
ink_refill_routine_enabled = ink_refill_routine_enabled_default
ink_refill_every_penstroke = False
ink_refill_every_x = 10
ink_refill_routine = ""

previous_go_to_angle_ratio = False
next_go_to_angle_ratio = False

acquire_instructions_from_web_on = False
stop_acquire_instructions_from_web = False


def main():
    global command_server_thread

    load_variables_from_disk()

    command_server_thread = threading.Thread( target=command_server )
    command_server_thread.start()
    # keep the main thread running, otherwise signals are ignored
    while True:
        time.sleep(0.5)

def run_ink_refill_routine():
    global command_running, ink_refill_routine

    flip_back_command_running = False
    if command_running==False:
        command_running = True
        flip_back_command_running = True
    print( "* ink refill routine start" )
    routine = ink_refill_routine.split( "\n" )
    for line in routine:
        orig_line = line
        line = line.split( "(" )
        if len(line)==2:
            command = line[0]
            params = line[1].replace( ")", "" ).split( "," )
            if command=="go_to" and len(params)==2:
                to_x = (float(params[0]))
                to_y = (float(params[1]))
                if to_x<0.0 or to_x>canvas_max_x or to_y<0.0 or to_y>canvas_max_y:
                    print( "*   WARNING: out of bound " + str(to_x) + "," + str(to_y) )
                else:
                    go_to( to_x, to_y )
            elif command=="pen_up":
                pen_up()
            elif command=="pen_down":
                pen_down()
            else:
                print( "*   WARNING: can't parse line: " + orig_line )

    if flip_back_command_running:
        command_running = False
    print( "* ink refill routine end" )


def pen_down():
    global pwm, current_pen_pulse_width, pen_up_pulse_width, pen_down_pulse_width, command_running, pen_up_or_down, previous_go_to_angle_ratio, next_go_to_angle_ratio

    if pen_up_or_down==True:
        # it's already down
        return False

    command_running = True
    pen_up_or_down = True
    previous_go_to_angle_ratio = False
    next_go_to_angle_ratio = False
    print( "> pen down" )

    if pen_down_action_time>0:
        pulse_width_diff = abs( pen_up_pulse_width - pen_down_pulse_width )
        pulse_width_change_per_millisecond = pulse_width_diff/pen_down_action_time
        add_or_remove = True # True for add, False for remove
        if current_pen_pulse_width>pen_down_pulse_width:
            add_or_remove = False
        for i in range( pen_down_action_time ):
            if add_or_remove:
                current_pen_pulse_width += pulse_width_change_per_millisecond
            else:
                current_pen_pulse_width -= pulse_width_change_per_millisecond
            if current_pen_pulse_width>=500 and current_pen_pulse_width<=2500:
                pwm.set_servo_pulsewidth( servo, current_pen_pulse_width )
            time.sleep( 0.001 )
    # to make sure we don't accumulate any rounding
    pwm.set_servo_pulsewidth( servo, pen_down_pulse_width )
    current_pen_pulse_width = pen_down_pulse_width
    time.sleep( pen_down_sleep_before_move_time/1000 )
    command_running = False


def pen_up():
    global pwm, current_pen_pulse_width, pen_up_pulse_width, pen_down_pulse_width, command_running, pen_up_or_down, previous_go_to_angle_ratio, next_go_to_angle_ratio

    if pen_up_or_down==False:
        # it's already up
        return False

    command_running = True
    pen_up_or_down = False
    previous_go_to_angle_ratio = False
    next_go_to_angle_ratio = False
    print("> pen up")

    if pen_up_action_time>0:
        pulse_width_diff = abs( pen_up_pulse_width - pen_down_pulse_width )
        pulse_width_change_per_millisecond = pulse_width_diff/pen_up_action_time
        add_or_remove = True # True for add, False for remove
        if current_pen_pulse_width>pen_up_pulse_width:
            add_or_remove = False
        for i in range( pen_up_action_time ):
            if add_or_remove:
                current_pen_pulse_width += pulse_width_change_per_millisecond
            else:
                current_pen_pulse_width -= pulse_width_change_per_millisecond
            if current_pen_pulse_width>=500 and current_pen_pulse_width<=2500:
                pwm.set_servo_pulsewidth( servo, current_pen_pulse_width )
            time.sleep( 0.001 )
    # to make sure we don't accumulate any rounding
    pwm.set_servo_pulsewidth( servo, pen_up_pulse_width )
    current_pen_pulse_width = pen_up_pulse_width
    time.sleep( pen_up_sleep_before_move_time/1000 )
    command_running = False


def test_bottom_stepper():
    global command_running
    command_running = True

    saved_step_size = step_size

    set_step_size( 1.0 )
    GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    for i in range( 200 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    for i in range( 200 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 2 )

    set_step_size( 1.0/2 )
    GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    for i in range( 200*2 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    for i in range( 200*2 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 2 )

    set_step_size( 1.0/4 )
    GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    for i in range( 200*4 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    for i in range( 200*4 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 2 )

    set_step_size( 1.0/8 )
    GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    for i in range( 200*8 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    time.sleep( 1 )
    for i in range( 200*8 ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( default_step_sleep )

    set_step_size( saved_step_size )
    command_running = False


def test_top_stepper():
    global command_running
    command_running = True

    saved_step_size = step_size

    set_step_size( 1.0 )
    GPIO.output( stepper_top_dir, GPIO.HIGH )
    for i in range( 200 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( 200 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 2 )

    set_step_size( 1.0/2 )
    GPIO.output( stepper_top_dir, GPIO.HIGH )
    for i in range( 200*2 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( 200*2 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 2 )

    set_step_size( 1.0/4 )
    GPIO.output( stepper_top_dir, GPIO.HIGH )
    for i in range( 200*4 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( 200*4 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 2 )

    set_step_size( 1.0/8 )
    GPIO.output( stepper_top_dir, GPIO.HIGH )
    for i in range( 200*8 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )
    time.sleep( 1 )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( 200*8 ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( default_step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( default_step_sleep )

    set_step_size( saved_step_size )
    command_running = False


def acquire_instructions_from_web( url ):
    global command_running, stop_acquire_instructions_from_web, calibration, acquire_instructions_from_web_on
    print( "- acquire_instructions_from_web url: " + url )
    first_response = True
    loop_with_nothing = 0
    while not stop_acquire_instructions_from_web:
        print( "- getting new instructions from: " + url )
        drew_something = False
        response = requests.get( url )
        if response.status_code!=200 and first_response:
            print( "- error: non 200 response code " + str(response.status_code) )
            stop_acquire_instructions_from_web = True
        else:
            if first_response:
                first_response = False
                acquire_instructions_from_web_on = True
                try:
                    f = open( "/root/web_instructions_ids_processed.json", "w" )
                    f.write( json.dumps([]) )
                    f.close()
                except:
                    # oops
                    print( "- file IO error" )
            try:
                f = open( "/root/web_instructions_ids_processed.json", "r" )
                web_instructions_ids_processed = json.loads( f.read() )
                f.close()
            except:
                    # oops
                    print( "- file IO error" )
            pcodes = json.loads( response.text )
            for pcode in pcodes:
                if str(pcode['id']) not in web_instructions_ids_processed:
                    print( "- drawing ID: " + str(pcode['id']) )
                    draw( pcode['pcode'] )
                    drew_something = True
                    web_instructions_ids_processed.append( str(pcode['id']) )
                    try:
                        f = open( "/root/web_instructions_ids_processed.json", "w" )
                        f.write( json.dumps(web_instructions_ids_processed) )
                        f.close()
                    except:
                        # oops
                        print( "- file IO error" )
        if not drew_something:
            loop_with_nothing += 1
            if loop_with_nothing==1:
                print( "- moving away from the last drawing" )
                go_to( random.uniform(10.0, 90.0), random.uniform(10.0, 90.0), True)
            else:
                time.sleep( 1 )
        else:
            loop_with_nothing = 0


def gondola_step_left( steps, step_sleep, direction ):
    if direction:
        GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    else:
        GPIO.output( stepper_bottom_dir, GPIO.LOW )
    for i in range( steps ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        time.sleep( step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        time.sleep( step_sleep )


def gondola_step_right( steps, step_sleep, direction ):
    if direction:
        GPIO.output( stepper_top_dir, GPIO.HIGH )
    else:
        GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( steps ):
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( step_sleep )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( step_sleep )


def step_down( steps, step_sleep ):
    GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    GPIO.output( stepper_top_dir, GPIO.HIGH )
    for i in range( steps ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        if not calibration_ongoing and (limit_switch_top_on() or limit_switch_bottom_on() or limit_switch_left_on() or limit_switch_right_on()):
            print("error: limit switch on")
            print("x_steps_skew:" + str(x_steps_skew) + ", y_steps_skew:" + str(y_steps_skew))
            exit( 0 )


def step_up( steps, step_sleep ):
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( steps ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        if not calibration_ongoing and (limit_switch_top_on() or limit_switch_bottom_on() or limit_switch_left_on() or limit_switch_right_on()):
            print("error: limit switch on")
            print("x_steps_skew:" + str(x_steps_skew) + ", y_steps_skew:" + str(y_steps_skew))
            exit( 0 )


def step_right( steps, step_sleep ):
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    GPIO.output( stepper_top_dir, GPIO.HIGH )
    for i in range( steps ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        if not calibration_ongoing and (limit_switch_top_on() or limit_switch_bottom_on() or limit_switch_left_on() or limit_switch_right_on()):
            print("error: limit switch on")
            print("x_steps_skew:" + str(x_steps_skew) + ", y_steps_skew:" + str(y_steps_skew))
            exit( 0 )


def step_left( steps, step_sleep ):
    GPIO.output( stepper_bottom_dir, GPIO.HIGH )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    for i in range( steps ):
        GPIO.output( stepper_bottom_step, GPIO.HIGH )
        GPIO.output( stepper_top_step, GPIO.HIGH )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        GPIO.output( stepper_bottom_step, GPIO.LOW )
        GPIO.output( stepper_top_step, GPIO.LOW )
        time.sleep( step_sleep )
        #pigpio.gpioDelay( step_sleep )
        if not calibration_ongoing and (limit_switch_top_on() or limit_switch_bottom_on() or limit_switch_left_on() or limit_switch_right_on()):
            print("error: limit switch on")
            print("x_steps_skew:" + str(x_steps_skew) + ", y_steps_skew:" + str(y_steps_skew))
            exit( 0 )


def limit_switch_top_on():
    if GPIO.input( limit_switch_top )==0:
        return True
    return False


def limit_switch_bottom_on():
    if GPIO.input( limit_switch_bottom )==0:
        return True
    return False


def limit_switch_left_on():
    if GPIO.input( limit_switch_left )==0:
        return True
    return False


def limit_switch_right_on():
    if GPIO.input( limit_switch_right )==0:
        return True
    return False


def get_potential_errors():
    try:
        f = open( "/var/log/plottybot.stderr.log", "r" )
        errors = f.read()
        f.close

        return errors
    except:
        return ""
    return ""


def load_variables_from_disk():
    global default_step_sleep, pen_down_pulse_width, pen_up_pulse_width, pen_down_action_time, pen_up_action_time, pen_down_sleep_before_move_time, pen_up_sleep_before_move_time, acceleration_steps, deceleration_steps, acceleration_slow_steps_sleep, acceleration_fast_steps_sleep, ink_refill_routine_enabled, ink_refill_every_penstroke, ink_refill_every_x, ink_refill_routine, gondola_steps_per_cm, gondola_reserve_margin, gondola_max_travel

    print( "> load_variables_from_disk" )
    # some variables are preserved accross reboots
    try:
        f = open( "/root/variables.json", "r" )
        variables = f.read()
        f.close

        variables = json.loads( variables )

        default_step_sleep = variables['default_step_sleep']
        pen_down_pulse_width = variables['pen_down_pulse_width']
        pen_up_pulse_width = variables['pen_up_pulse_width']
        pen_down_action_time = variables['pen_down_action_time']
        pen_up_action_time = variables['pen_up_action_time']
        pen_down_sleep_before_move_time = variables['pen_down_sleep_before_move_time']
        pen_up_sleep_before_move_time = variables['pen_up_sleep_before_move_time']
        acceleration_steps = variables['acceleration_steps']
        deceleration_steps = variables['deceleration_steps']
        acceleration_slow_steps_sleep = variables['acceleration_slow_steps_sleep']
        acceleration_fast_steps_sleep = variables['acceleration_fast_steps_sleep']
        ink_refill_routine_enabled = variables['ink_refill_routine_enabled']
        ink_refill_every_penstroke = variables['ink_refill_every_penstroke']
        ink_refill_every_x = variables['ink_refill_every_x']
        ink_refill_routine = variables['ink_refill_routine']
        gondola_steps_per_cm = variables['gondola_steps_per_cm']
        gondola_reserve_margin = variables['gondola_reserve_margin']
        gondola_max_travel = variables['gondola_max_travel']
    except:
        # I guess we'll just keep the defaults
        print( "no variables on disk, must be a first run" )
        pass


def save_variables_to_disk():
    global default_step_sleep, pen_down_pulse_width, pen_up_pulse_width, pen_down_action_time, pen_up_action_time, pen_down_sleep_before_move_time, pen_up_sleep_before_move_time, acceleration_steps, deceleration_steps, acceleration_slow_steps_sleep, acceleration_fast_steps_sleep, ink_refill_routine_enabled, ink_refill_every_penstroke, ink_refill_every_x, ink_refill_routine, gondola_steps_per_cm, gondola_reserve_margin, gondola_max_travel

    print( "> save_variables_to_disk" )
    # some variables are preserved accross reboots
    variables = {}
    variables['default_step_sleep'] = default_step_sleep
    variables['pen_down_pulse_width'] = pen_down_pulse_width
    variables['pen_up_pulse_width'] = pen_up_pulse_width
    variables['pen_down_action_time'] = pen_down_action_time
    variables['pen_up_action_time'] = pen_up_action_time
    variables['pen_down_sleep_before_move_time'] = pen_down_sleep_before_move_time
    variables['pen_up_sleep_before_move_time'] = pen_up_sleep_before_move_time
    variables['acceleration_steps'] = acceleration_steps
    variables['deceleration_steps'] = deceleration_steps
    variables['acceleration_slow_steps_sleep'] = acceleration_slow_steps_sleep
    variables['acceleration_fast_steps_sleep'] = acceleration_fast_steps_sleep
    variables['ink_refill_routine_enabled'] = ink_refill_routine_enabled
    variables['ink_refill_every_penstroke'] = ink_refill_every_penstroke
    variables['ink_refill_every_x'] = ink_refill_every_x
    variables['ink_refill_routine'] = ink_refill_routine
    variables['gondola_steps_per_cm'] = gondola_steps_per_cm
    variables['gondola_reserve_margin'] = gondola_reserve_margin
    variables['gondola_max_travel'] = gondola_max_travel
    try:
        f = open( "/root/variables.json", "w" )
        f.write( json.dumps(variables) )
        f.close()
    except:
        # oops
        pass


# taken from https://stackoverflow.com/a/30034847
def get_split_lines(file_path):
    with open(file_path, "r") as f:
        for line in f:
            yield line


def draw( turtle_code ):
    global command_running, pause_draw, pen_up_or_down, stop_draw, ink_refill_routine_enabled, ink_refill_every_x, ink_refill_every_penstroke, draw_going, current_step_sleep

    command_running = True
    draw_going = True
    print("> draw")

    if turtle_code=="file":
        turtle_code = list( get_split_lines("/data/tcode") )
        for i in range(len(turtle_code)):
            turtle_code[i] = turtle_code[i].replace( " ", "" )
            turtle_code[i] = turtle_code[i].lower()
    else:
        turtle_code = turtle_code.replace( " ", "" )
        turtle_code = turtle_code.lower()
        turtle_code = turtle_code.split( "\n" )

    shift_by_x = 0.0
    shift_by_y = 0.0

    x_ratio = 1.0
    y_ratio = 1.0
    ratio = 1.0

    min_x = False
    min_y = False
    max_x = False
    max_y = False

    for line in turtle_code:
        line = line.split( "(" )
        if len(line)==2:
            command = line[0]
            params = line[1].replace( ")", "" ).split( "," )
            if command=="go_to" and len(params)==2:
                to_x = float( params[0] )
                to_y = float( params[1] )

                if min_x is False or to_x<min_x:
                    min_x = to_x
                if max_x is False or to_x>max_x:
                    max_x = to_x
                if min_y is False or to_y<min_y:
                    min_y = to_y
                if max_y is False or to_y>max_y:
                    max_y = to_y

    print(">   min_x " + str( min_x ))
    print(">   max_x " + str( max_x ))
    print(">   min_y " + str( min_y ))
    print(">   max_y " + str( max_y ))

    if rotate_to_maximize_drawing==True and (max_x-min_x)>(max_y-min_y) and canvas_max_x<canvas_max_y:
        print(">   rotating drawing 90 degrees to maximize drawing area")
        new_turtle_code = ""
        counter = 1
        for line in turtle_code:
            # if counter%1000==0:
            #     print str(counter) + "/" + str(len(turtle_code))
            counter = counter + 1
            orig_line = line
            line = line.split( "(" )
            if len(line)==2:
                command = line[0]
                params = line[1].replace( ")", "" ).replace( " ", "" ).split( "," )
                if command=="go_to" and len(params)==2:
                    to_x = float(params[1])
                    to_y = max_y - float(params[0])
                    new_turtle_code += "\n" + "go_to(" + str(to_y) + "," + str(to_x) + ")"
                else:
                    new_turtle_code += "\n" + orig_line
        draw( new_turtle_code ) ;
        return 0


    # fit to canvas
    # x_ratio = canvas_max_x / (max_x-min_x)
    # y_ratio = canvas_max_y / (max_y-min_y)
    # override, we do this on the front end now
    x_ratio = 1.0
    y_ratio = 1.0
    ratio = min( x_ratio, y_ratio )
    print("x_ratio " + str( x_ratio ))
    print("y_ratio " + str( y_ratio ))
    print("ratio " + str( ratio ))

    previous_x = current_x
    previous_y = current_y
    ink_refill_traveled_distance_since_last_refill = 0

    steps = []
    current_step_sleep = acceleration_slow_steps_sleep
    pen_down_after_processing = False
    for i in range(len(turtle_code)):
        line = turtle_code[i]
        if pause_draw:
            saved_pen_up_or_down = pen_up_or_down
            if pen_up_or_down:
                pen_up()
            while pause_draw:
                time.sleep( 0.1 )
            if saved_pen_up_or_down:
                pen_down()
        orig_line = line
        #print line

        # we need an ink refill if we're getting started
        if i==0 and ink_refill_routine_enabled:
            ink_refill_traveled_distance_since_last_refill = 0
            run_ink_refill_routine()

        line = line.split( "(" )
        if len(line)==2:
            command = line[0]
            params = line[1].replace( ")", "" ).split( "," )
            if command=="go_to" and len(params)==2:
                # to_x = (float(params[0]) - min_x) * ratio
                # to_y = (float(params[1]) - min_y) * ratio
                # no need for shifting anymore, we do this on the front end
                to_x = (float(params[0])) * ratio
                to_y = (float(params[1])) * ratio
                if to_x<0.0 or to_x>canvas_max_x or to_y<0.0 or to_y>canvas_max_y:
                    print(">   WARNING: out of bound " + str(to_x) + "," + str(to_y))
                else:
                    ink_refill_traveled_distance_since_last_refill += math.sqrt( (to_x-previous_x)**2 + (to_y-previous_y)**2 )
                    previous_x = to_x
                    previous_y = to_y
                    next_x = False
                    next_y = False
                    # trying to find next_x and next_y to refine acceleration based on angle
                    # disabled for now
                    # for j in range( i+1, len(turtle_code) ):
                    #     potential_line = turtle_code[j]
                    #     potential_line = potential_line.split( "(" )
                    #     if len(potential_line)==2:
                    #         potential_command = potential_line[0]
                    #         potential_params = potential_line[1].replace( ")", "" ).split( "," )
                    #         if potential_command=="go_to" and len(potential_params)==2:
                    #             potential_to_x = (float(potential_params[0])) * ratio
                    #             potential_to_y = (float(potential_params[1])) * ratio
                    #             if potential_to_x<0.0 or potential_to_x>canvas_max_x or potential_to_y<0.0 or potential_to_y>canvas_max_y:
                    #                 print(">   WARNING: potential next out of bound " + str(potential_to_x) + "," + str(potential_to_y))
                    #             else:
                    #                 next_x = potential_to_x
                    #                 next_y = potential_to_y
                    #                 break
                    #         elif command=="pen_up":
                    #             break
                    #         elif command=="pen_down":
                    #             break
                    steps += go_to( to_x, to_y, False, pen_down_after_processing, next_x, next_y )
            elif command=="pen_up":
                if pen_down_after_processing:
                    pen_down_after_processing = False
                    pen_down()
                gondola_actuate( steps )
                pen_up()
                if ink_refill_routine_enabled and i<(len(turtle_code)-1) and ink_refill_traveled_distance_since_last_refill>0:
                    if ink_refill_every_penstroke or ink_refill_traveled_distance_since_last_refill>=ink_refill_every_x:
                        ink_refill_traveled_distance_since_last_refill = 0
                        run_ink_refill_routine()
            elif command=="pen_down":
                if pen_down_after_processing is not True:
                    gondola_actuate( steps )
                    pen_down_after_processing = True
                    print( "> will pen down after processing stroke to come" )
                else:
                    print( "> extraneous pen down" )
                # pen_down()
            elif command=="pause":
                pause_draw = True
            else:
                print(">   WARNING: can't parse line: " + orig_line)
        if stop_draw:
            stop_draw = False
            pause_draw = False
            break ;

    if pen_down_after_processing:
            pen_down_after_processing = False
            pen_down()
    gondola_actuate( steps )

    # getting out of the way
    if pen_up_or_down:
        pen_up()

    if get_out_of_the_way_after_draw:
        get_out_of_the_way_x = min_x - 10
        get_out_of_the_way_y = max_y + 10
        if get_out_of_the_way_x<10.0 or get_out_of_the_way_x>(canvas_max_x-10.0) or get_out_of_the_way_y<10.0 or get_out_of_the_way_y>(canvas_max_y-10.0):
            get_out_of_the_way_x = 10.0
            get_out_of_the_way_y = canvas_max_y - 10
        go_to( get_out_of_the_way_x, get_out_of_the_way_y )

    command_running = False
    draw_going = False


def gondola_get_l( x, y ):
    return math.sqrt( math.pow(x, 2) + math.pow(100-y, 2) )


def gondola_get_r( x, y ):
    return math.sqrt( math.pow(100-x, 2) + math.pow(100-y, 2) )


def gondola_get_l_diff( x_origin, y_origin, x_destination, y_destination ):
    return gondola_get_l( x_destination, y_destination ) - gondola_get_l( x_origin, y_origin )


def gondola_get_r_diff( x_origin, y_origin, x_destination, y_destination ):
    return gondola_get_r( x_destination, y_destination ) - gondola_get_r( x_origin, y_origin )


def gondola_units_to_steps( units ):
    return units * (gondola_calibration_length/100) * gondola_steps_per_cm


def gondola_go_to( x, y, depth=0, acceleration=True, deceleration=True, actuate=True, gondola_travel_break=False, next_x=False, next_y=False ):
    global current_x, current_y, x_steps_skew, y_steps_skew, command_running, previous_go_to_angle_ratio, next_go_to_angle_ratio, current_step_sleep

    command_running = True
    if depth==0:
        print( "> gondola_go_to( " + str(x) + ", " + str(y) + " )" )
    x_orig = x
    y_orig = y
    reserve_margin_ratio = (100.0-2*gondola_reserve_margin)/100.0
    x = x * reserve_margin_ratio + gondola_reserve_margin
    y = y * reserve_margin_ratio + gondola_reserve_margin
    if depth==0:
        print( ">   including reserved margins: ( " + str(x) + ", " + str(y) + " )" )
    else:
        print( ">       ( " + str(x) + ", " + str(y) + " )" )
    if gondola_travel_break:
        # https://math.stackexchange.com/questions/153238/finding-a-point-on-the-line-joining-two-points-in-cartesian-coordinate-system
        gondola_distance = math.sqrt( (x-current_x)**2 + (y-current_y)**2 )
        if depth==0 and gondola_distance>gondola_max_travel:
            print( ">     travel break with distance: " + str(gondola_distance) + " greater than: " + str(gondola_max_travel) )
        if gondola_distance>gondola_max_travel and abs(gondola_distance-gondola_max_travel)>0.0000001: # removes rounding errors
            alpha = gondola_max_travel / gondola_distance
            new_x = current_x + alpha * ( x - current_x )
            new_y = current_y + alpha * ( y - current_y )
            new_x = ( new_x - gondola_reserve_margin ) / reserve_margin_ratio
            new_y = ( new_y - gondola_reserve_margin ) / reserve_margin_ratio
            if depth==0:
                return gondola_go_to( new_x, new_y, depth+1, True, False, actuate, gondola_travel_break ) + gondola_go_to( x_orig, y_orig, depth+1, False, True, actuate, gondola_travel_break )
            else:
                return gondola_go_to( new_x, new_y, depth+1, False, False, actuate, gondola_travel_break ) + gondola_go_to( x_orig, y_orig, depth+1, False, True, actuate, gondola_travel_break )

    l_steps = gondola_units_to_steps( gondola_get_l_diff(current_x, current_y, x, y) ) - x_steps_skew
    r_steps = gondola_units_to_steps( gondola_get_r_diff(current_x, current_y, x, y) ) - y_steps_skew

    x_steps_skew = round( l_steps ) - l_steps
    y_steps_skew = round( r_steps ) - r_steps

    l_steps = int( round(l_steps) )
    r_steps = int( round(r_steps) )

    l_direction = False
    if l_steps<0:
        l_direction = True
        l_steps = abs( l_steps )
    r_direction = True
    if r_steps<0:
        r_direction = False
        r_steps = abs( r_steps )
    steps_to_take = [] ;
    # for i in range( l_steps ):
    #     steps_to_take.append( False )
    # for i in range( r_steps ):
    #     steps_to_take.append( True )
    # random.shuffle( steps_to_take )
    # method 3
    if l_steps==0:
        for i in range(r_steps):
            steps_to_take.append( [True, acceleration_fast_steps_sleep, r_direction] )
    elif r_steps==0:
        for i in range(l_steps):
            steps_to_take.append( [False, acceleration_fast_steps_sleep, l_direction] )
    elif l_steps==r_steps:
        for i in range(l_steps):
            steps_to_take.append( [False, acceleration_fast_steps_sleep, l_direction] )
            steps_to_take.append( [True, acceleration_fast_steps_sleep, r_direction] )
    elif l_steps>r_steps:
        for i in range(l_steps):
            steps_to_take.append( [False, acceleration_fast_steps_sleep, l_direction] )
        r_remaining = r_steps
        while (l_steps+r_steps)-len(steps_to_take)>0:
            every_nth = math.ceil( float(len(steps_to_take)) / float(r_remaining) )
            every_nth = int( every_nth ) ;
            new_steps_to_take = []
            for i in range( len(steps_to_take) ):
                new_steps_to_take.append( steps_to_take[i] )
                if (i+1)%every_nth==0 and r_remaining>0:
                    new_steps_to_take.append( [True, acceleration_fast_steps_sleep, r_direction] )
                    r_remaining = r_remaining - 1
            steps_to_take = new_steps_to_take
    else: # r_steps>l_steps
        for i in range(r_steps):
            steps_to_take.append( [True, acceleration_fast_steps_sleep, r_direction] )
        l_remaining = l_steps
        while (l_steps+r_steps)-len(steps_to_take)>0:
            every_nth = math.ceil( float(len(steps_to_take)) / float(l_remaining) )
            every_nth = int( every_nth )
            new_steps_to_take = []
            for i in range( len(steps_to_take) ):
                new_steps_to_take.append( steps_to_take[i] )
                if (i+1)%every_nth==0 and l_remaining>0:
                    new_steps_to_take.append( [False, acceleration_fast_steps_sleep, l_direction] )
                    l_remaining = l_remaining - 1
            steps_to_take = new_steps_to_take

    # acceleration model
    if depth==0 and next_x is not False and next_y is not False:
        next_x = next_x * (100.0-2*gondola_reserve_margin)/100.0 + gondola_reserve_margin
        next_y = next_y * (100.0-2*gondola_reserve_margin)/100.0 + gondola_reserve_margin
        print( ">   acceleration angle adjust" )
        print( ">     current_x: " + str(current_x) )
        print( ">     current_y: " + str(current_y) )
        print( ">     x: " + str(x) )
        print( ">     y: " + str(y) )
        print( ">     next_x: " + str(next_x) )
        print( ">     next_y: " + str(next_y) )
        angle_0 = math.degrees( math.atan2(y-current_y, x-current_x) )
        angle_1 = math.degrees( math.atan2( next_y-y, next_x-x ) )
        next_go_to_angle_ratio = abs( ( angle_1 - angle_0 ) / 180.0 )
        if next_go_to_angle_ratio>1.0:
            next_go_to_angle_ratio %= 1.0
            next_go_to_angle_ratio = 1.0 - next_go_to_angle_ratio
        print( ">       next_go_to_angle_ratio: " + str(next_go_to_angle_ratio) )

    acceleration_step_sleep_delta = (acceleration_fast_steps_sleep - acceleration_slow_steps_sleep) / acceleration_steps
    deceleration_step_sleep_delta = (acceleration_slow_steps_sleep - acceleration_fast_steps_sleep) / deceleration_steps
    # print( "acceleration_step_sleep_delta: " + str( acceleration_step_sleep_delta ) )
    # print( "deceleration_step_sleep_delta: " + str( deceleration_step_sleep_delta ) )

    acceleration_times_to_sleep = []
    if acceleration:
        acceleration_from = current_step_sleep
        acceleration_to = acceleration_fast_steps_sleep
        i = 0
        while i<len(steps_to_take) and current_step_sleep>acceleration_fast_steps_sleep:
            current_step_sleep += acceleration_step_sleep_delta
            if current_step_sleep<acceleration_fast_steps_sleep:
                current_step_sleep = acceleration_fast_steps_sleep
            steps_to_take[i][1] = current_step_sleep
            i += 1

    if deceleration:
        deceleration_to = acceleration_slow_steps_sleep
        if next_go_to_angle_ratio is not False:
            deceleration_to *= next_go_to_angle_ratio
        i = len(steps_to_take)-1
        temp_reverse_current_step_sleep = deceleration_to
        while i>=0 and temp_reverse_current_step_sleep>steps_to_take[i][1]:
            steps_to_take[i][1] = temp_reverse_current_step_sleep
            temp_reverse_current_step_sleep -= deceleration_step_sleep_delta
            i -= 1

    if len(steps_to_take)>0:
        current_step_sleep = steps_to_take[len(steps_to_take)-1][1]



    # for i in range( len(steps_to_take) ):
    #     print( steps_to_take[i] )
    # exit( 1 )



    # acceleration_times_to_sleep = []
    # acceleration_step_count = 0
    # if acceleration:
    #     if previous_go_to_angle_ratio is not False:
    #         acceleration_step_count = round( acceleration_steps * previous_go_to_angle_ratio )
    #     else:
    #         acceleration_step_count = acceleration_steps

    # deceleration_times_to_sleep = []
    # deceleration_step_count = 0
    # if deceleration:
    #     if next_go_to_angle_ratio is not False:
    #         deceleration_step_count = round( deceleration_steps * next_go_to_angle_ratio )
    #     else:
    #         deceleration_step_count = deceleration_steps

    # if acceleration_step_count>0:
    #     accelerate_from = current_step_sleep
    #     accelerate_to = acceleration_fast_steps_sleep
    #     tts_diff = abs( accelerate_from-accelerate_to ) / acceleration_step_count
    #     if (acceleration_step_count+deceleration_step_count)<=len(steps_to_take):
    #         # easy
    #         for i in range( acceleration_step_count ):
    #             acceleration_times_to_sleep.append( round(accelerate_from-(i+1)*tts_diff, 4) )
    #             current_step_sleep = acceleration_times_to_sleep[len(acceleration_times_to_sleep)-1]
    #     else:
    #         # we can only do a partial acceleration
    #         for i in range( acceleration_step_count-int((len(steps_to_take)-acceleration_step_count-deceleration_step_count)/2) ):
    #             acceleration_times_to_sleep.append( round(accelerate_from-(i+1)*tts_diff, 4) )
    #             current_step_sleep = acceleration_times_to_sleep[len(acceleration_times_to_sleep)-1]

    # if deceleration_step_count>0:
    #     decelerate_from = current_step_sleep
    #     decelerate_to = acceleration_slow_steps_sleep + ((acceleration_fast_steps_sleep - acceleration_slow_steps_sleep) * next_go_to_angle_ratio)
    #     tts_diff = abs( decelerate_from-decelerate_to ) / deceleration_step_count
    #     if (acceleration_step_count+deceleration_step_count)<=len(steps_to_take):
    #         # easy
    #         for i in range( deceleration_step_count ):
    #             deceleration_times_to_sleep.append( round(decelerate_from+(i+1)*tts_diff, 4) )
    #             current_step_sleep = deceleration_times_to_sleep[len(deceleration_times_to_sleep)-1]
    #     else:
    #         # we can only do a partial acceleration
    #         for i in range( deceleration_step_count-int((len(steps_to_take)-acceleration_step_count-deceleration_step_count)/2) ):
    #             deceleration_times_to_sleep.append( round(accelerate_from-(i+1)*tts_diff, 4) )
    #             current_step_sleep = deceleration_times_to_sleep[len(deceleration_times_to_sleep)-1]

    ###################################################################

    # if acceleration_step_count>0:
    #     tts_diff = abs( accelerate_from-accelerate_to ) / acceleration_step_count
    #     for i in range( min(acceleration_step_count, len(steps_to_take)) ):
    #         acceleration_times_to_sleep.append( round(accelerate_from-(i+1)*tts_diff, 4) )
    #         cur


    # acceleration_times_to_sleep = []

    # effective_acceleration_steps = acceleration_steps
    # if previous_go_to_angle_ratio is not False:
    #     acceleration_ratio = previous_go_to_angle_ratio
    #     effective_acceleration_steps = round( effective_acceleration_steps * acceleration_ratio )
    #     print( ">         acceleration_ratio: " + str(acceleration_ratio) )
    # if effective_acceleration_steps>0:
    #     print( ">         effective_acceleration_steps: " + str(effective_acceleration_steps) )
    #     tts_diff = abs(current_step_sleep-acceleration_fast_steps_sleep)/effective_acceleration_steps
    #     for i in range( min(effective_acceleration_steps, len(steps_to_take)) ):
    #         acceleration_times_to_sleep.append( round(current_step_sleep-(i+1)*tts_diff, 4) )
    # print( "acceleration_times_to_sleep" )
    # print( acceleration_times_to_sleep )

    # deceleration_times_to_sleep = []
    # decelerate_from_sleep = current_step_sleep

    # if len(acceleration_times_to_sleep)>0:
    #     decelerate_from_sleep = acceleration_times_to_sleep[len(acceleration_times_to_sleep)-1]

    # effective_deceleration_steps = deceleration_steps
    # if next_go_to_angle_ratio is not False:
    #     deceleration_ratio = next_go_to_angle_ratio
    #     effective_deceleration_steps = round( effective_deceleration_steps * deceleration_ratio )
    #     print( ">         deceleration_ratio: " + str(deceleration_ratio) )

    # if effective_deceleration_steps>0:
    #     print( ">         effective_deceleration_steps: " + str(effective_deceleration_steps) )
    #     tts_diff = abs(decelerate_from_sleep-acceleration_slow_steps_sleep)/(min(effective_deceleration_steps, len(steps_to_take))+1)
    #     for i in range( min(effective_deceleration_steps, len(steps_to_take)) ):
    #         deceleration_times_to_sleep.append( round(decelerate_from_sleep+(i+1)*tts_diff, 4) )
    # print( "deceleration_times_to_sleep" )
    # print( deceleration_times_to_sleep )

    #############################################################################

    # times_to_sleep = []
    # overlap = len(acceleration_times_to_sleep)+len(deceleration_times_to_sleep)-len(steps_to_take)
    # j = 0
    # if overlap<=0:
    #     # easy peasy
    #     for i in range( len(acceleration_times_to_sleep) ):
    #         if acceleration:
    #             steps_to_take[j][1] = acceleration_times_to_sleep[i]
    #         else:
    #             steps_to_take[j][1] = acceleration_fast_steps_sleep
    #         j += 1
    #     for i in range( len(steps_to_take)-len(acceleration_times_to_sleep)-len(deceleration_times_to_sleep) ):
    #         steps_to_take[j][1] = acceleration_fast_steps_sleep
    #         j += 1
    #     for i in range( len(deceleration_times_to_sleep) ):
    #         if deceleration:
    #             steps_to_take[j][1] = deceleration_times_to_sleep[i]
    #         else:
    #             steps_to_take[j][1] = acceleration_fast_steps_sleep
    #         j += 1
    # elif overlap>0:
    #     print( "> error #tywnethe: we should never encounter this case" )
    #     print( len(steps_to_take) )
    #     print( len(acceleration_times_to_sleep) )
    #     print( len(deceleration_times_to_sleep) )
    #     exit( 1 )

    if actuate:
        gondola_actuate( steps_to_take )
    # for i in range( len(steps_to_take) ):
    #     print( steps_to_take[i] )

    previous_go_to_angle_ratio = next_go_to_angle_ratio
    next_go_to_angle_ratio = False
    current_x = x
    current_y = y
    command_running = False

    return steps_to_take


def gondola_actuate( steps_to_take ):
    global current_step_sleep

    i = 0
    while i<len(steps_to_take):
        if steps_to_take[i][0]==False:
            # print( "l" )
            if i<(len(steps_to_take)-1) and steps_to_take[i+1][0]==True:
                gondola_step_left( 1, 0, steps_to_take[i][2] )
                gondola_step_right( 1, max(steps_to_take[i][1],steps_to_take[i+1][1]), steps_to_take[i+1][2] )
                i += 1
            else:
                gondola_step_left( 1, steps_to_take[i][1], steps_to_take[i][2] )
        else:
            # print( "r" )
            if i<(len(steps_to_take)-1) and steps_to_take[i+1][0]==False:
                gondola_step_right( 1, 0, steps_to_take[i][2] )
                gondola_step_left( 1, max(steps_to_take[i][1],steps_to_take[i+1][1]), steps_to_take[i+1][2] )
                i += 1
            else:
                gondola_step_right( 1, steps_to_take[i][1], steps_to_take[i][2] )
        i += 1
    steps_to_take.clear()
    current_step_sleep = acceleration_slow_steps_sleep


def go_to( x, y, actuate=True, gondola_travel_break=False, next_x=False, next_y=False ):
    global current_x, current_y, x_steps_skew, y_steps_skew, command_running, gondola

    if gondola:
        return gondola_go_to( x, y, 0, True, True, actuate, gondola_travel_break, next_x, next_y )

    command_running = True
    print( "> go_to( " + str(x) + ", " + str(y) + " )" )

    x_diff = x - current_x
    y_diff = y - current_y
    # print "y_diff: " + str( y_diff )
    # print "canvas_max_y_steps: " + str( canvas_max_y_steps )
    # print "canvas_max_y: " + str( canvas_max_y )
    # print "y_steps_skew: " + str( y_steps_skew )
    x_steps = ( x_diff * canvas_max_x_steps / canvas_max_x ) - x_steps_skew
    y_steps = ( y_diff * canvas_max_y_steps / canvas_max_y ) - y_steps_skew

    x_steps_skew = round( x_steps ) - x_steps
    y_steps_skew = round( y_steps ) - y_steps

    x_steps = int( round(x_steps) )
    y_steps = int( round(y_steps) )

    steps_to_take = []
    horizontal = 6
    if x_steps<0:
        horizontal = 4
        x_steps = abs( x_steps )
    vertical = 2
    if y_steps<0:
        vertical = 8
        y_steps = abs( y_steps )

    # method 3
    if x_steps==0:
        for i in range(y_steps):
            steps_to_take.append( vertical )
    elif y_steps==0:
        for i in range(x_steps):
            steps_to_take.append( horizontal )
    elif x_steps==y_steps:
        for i in range(x_steps):
            steps_to_take.append( horizontal )
            steps_to_take.append( vertical )
    elif x_steps>y_steps:
        for i in range(x_steps):
            steps_to_take.append( horizontal )
        y_remaining = y_steps
        while (x_steps+y_steps)-len(steps_to_take)>0:
            every_nth = math.ceil( float(len(steps_to_take)) / float(y_remaining) )
            every_nth = int( every_nth ) ;
            new_steps_to_take = []
            for i in range( len(steps_to_take) ):
                new_steps_to_take.append( steps_to_take[i] )
                if (i+1)%every_nth==0 and y_remaining>0:
                    new_steps_to_take.append( vertical )
                    y_remaining = y_remaining - 1
            steps_to_take = new_steps_to_take
    else: # y_steps>x_steps
        # print x_steps
        # print y_steps
        for i in range(y_steps):
            steps_to_take.append( vertical )
        x_remaining = x_steps
        while (x_steps+y_steps)-len(steps_to_take)>0:
            every_nth = math.ceil( float(len(steps_to_take)) / float(x_remaining) )
            every_nth = int( every_nth )
            new_steps_to_take = []
            for i in range( len(steps_to_take) ):
                new_steps_to_take.append( steps_to_take[i] )
                if (i+1)%every_nth==0 and x_remaining>0:
                    new_steps_to_take.append( horizontal )
                    x_remaining = x_remaining - 1
            steps_to_take = new_steps_to_take

    # acceleration model
    current_step_sleep = acceleration_slow_steps_sleep
    acceleration_times_to_sleep = []
    if acceleration_steps>0:
        tts_diff = abs(current_step_sleep-acceleration_fast_steps_sleep)/acceleration_steps
        for i in range( min(acceleration_steps, len(steps_to_take)) ):
            acceleration_times_to_sleep.append( round(current_step_sleep-(i+1)*tts_diff, 4) )
    deceleration_times_to_sleep = []
    decelerate_from_sleep = current_step_sleep
    if len(acceleration_times_to_sleep)>0:
        decelerate_from_sleep = acceleration_times_to_sleep[len(acceleration_times_to_sleep)-1]
    if deceleration_steps>0:
        tts_diff = abs(decelerate_from_sleep-acceleration_slow_steps_sleep)/(min(deceleration_steps, len(steps_to_take))+1)
        for i in range( min(deceleration_steps, len(steps_to_take)) ):
            deceleration_times_to_sleep.append( round(decelerate_from_sleep+(i+1)*tts_diff, 4) )
    times_to_sleep = []
    overlap = len(acceleration_times_to_sleep)+len(deceleration_times_to_sleep)-len(steps_to_take)
    overlap_cutoff = int(overlap/2) #int((acceleration_steps/deceleration_steps)*overlap)
    if overlap<=0:
        # easy peasy
        for i in range( len(acceleration_times_to_sleep) ):
            times_to_sleep.append( acceleration_times_to_sleep[i] )
        for i in range( len(steps_to_take)-len(acceleration_times_to_sleep)-len(deceleration_times_to_sleep) ):
            times_to_sleep.append( acceleration_fast_steps_sleep )
        for i in range( len(deceleration_times_to_sleep) ):
            times_to_sleep.append( deceleration_times_to_sleep[i] )
    elif overlap>0:
        for i in range( len(acceleration_times_to_sleep)-overlap_cutoff ):
            times_to_sleep.append( acceleration_times_to_sleep[i] )
        if overlap%2!=0:
            overlap_cutoff += 1
        for i in range( overlap_cutoff, len(deceleration_times_to_sleep) ):
            times_to_sleep.append( deceleration_times_to_sleep[i] )

    if actuate:
        tabletop_actuate( steps_to_take )

    current_x = x
    current_y = y
    command_running = False


def tabletop_actuate( steps_to_take ):
    for i in range(len(steps_to_take)):
        if steps_to_take[i]==8:
            step_down( 1, times_to_sleep[i] )
        elif steps_to_take[i]==2:
            step_up( 1, times_to_sleep[i] )
        elif steps_to_take[i]==4:
            step_left( 1, times_to_sleep[i] )
        elif steps_to_take[i]==6:
            step_right( 1, times_to_sleep[i] )
        else:
            print("> error #epwiroj948o: what in the heck are we doing here?")
            exit( 1 )


def calibrate_manually():
    global canvas_max_x_steps, canvas_max_y_steps, canvas_max_x, canvas_max_y, current_x, current_y, calibration_ongoing, calibration_done, command_running
    global calibrate_manually_step_down, calibrate_manually_step_up, calibrate_manually_step_left, calibrate_manually_step_right
    global calibrate_manually_down_reached, calibrate_manually_up_reached, calibrate_manually_left_reached, calibrate_manually_right_reached

    command_running = True
    print("> calibrate_manually")

    if gondola:
        canvas_max_x = 100.0
        canvas_max_y = 100.0
        current_x = 50.0
        current_y = 50.0
        print(">   head assumed to be currently at " + str(current_x) + "," + str(current_y))
        calibration_ongoing = False
        calibration_done = True
        command_running = False
        return

    calibration_done = False
    calibrate_manually_down_reached = False
    calibrate_manually_up_reached = False
    calibrate_manually_left_reached = False
    calibrate_manually_right_reached = False
    canvas_max_x_steps = 0
    canvas_max_y_steps = 0
    canvas_max_x = False
    canvas_max_y = False
    calibration_ongoing = True

    if limit_switch_bottom_on(): # we need to move away from the bottom
        print(">   too close to bottom edge, looks like we need to get away from it")
        while True:
            if limit_switch_top_on():
                print("error: calibration issues")
                exit( 1 )
            step_up( 1, default_step_sleep )
            if not limit_switch_bottom_on():
                break ;
        step_up( 100, default_step_sleep )
    if limit_switch_top_on(): # we need to move away from the top
        print(">   too close to top edge, looks like we need to get away from it")
        while True:
            if limit_switch_bottom_on():
                print("error: calibration issues")
                exit( 1 )
            step_down( 1, default_step_sleep )
            if not limit_switch_top_on():
                break ;
        step_down( 100, default_step_sleep )
    if limit_switch_left_on(): # we need to move away from the left
        print(">   too close to left edge, looks like we need to get away from it")
        while True:
            if limit_switch_right_on():
                print("error: calibration issues")
                exit( 1 )
            step_right( 1, default_step_sleep )
            if not limit_switch_left_on():
                break ;
        step_right( 100, default_step_sleep )
    if limit_switch_right_on(): # we need to move away from the right
        print(">   too close to right edge, looks like we need to get away from it")
        while True:
            if limit_switch_left_on():
                print("error: calibration issues")
                exit( 1 )
            step_left( 1, default_step_sleep )
            if not limit_switch_right_on():
                break ;
        step_left( 100, default_step_sleep )

    print( ">   down" )
    while True:
        if calibrate_manually_step_down:
            step_down( 1, default_step_sleep*2 )
        elif calibrate_manually_step_up:
            step_up( 1, default_step_sleep*2 )
        elif calibrate_manually_step_left:
            step_left( 1, default_step_sleep*2 )
        elif calibrate_manually_step_right:
            step_right( 1, default_step_sleep*2 )
        if limit_switch_bottom_on():
            while limit_switch_bottom_on():
                step_up( 1, default_step_sleep )
        if limit_switch_top_on():
            while limit_switch_top_on():
                step_down( 1, default_step_sleep )
        if limit_switch_left_on():
            while limit_switch_left_on():
                step_right( 1, default_step_sleep )
        if limit_switch_right_on():
            while limit_switch_right_on():
                step_left( 1, default_step_sleep )
        if calibrate_manually_down_reached:
            print( ">     down position reached" )
            break
        # time.sleep( 0.05 )

    print(">   counting steps up")
    while True:
        if calibrate_manually_step_down:
            step_down( 1, default_step_sleep*2 )
            canvas_max_y_steps -= 1
        elif calibrate_manually_step_up:
            step_up( 1, default_step_sleep*2 )
            canvas_max_y_steps += 1
        elif calibrate_manually_step_left:
            step_left( 1, default_step_sleep*2 )
        elif calibrate_manually_step_right:
            step_right( 1, default_step_sleep*2 )
        if limit_switch_bottom_on():
            while limit_switch_bottom_on():
                step_up( 1, default_step_sleep )
        if limit_switch_top_on():
            while limit_switch_top_on():
                step_down( 1, default_step_sleep )
        if limit_switch_left_on():
            while limit_switch_left_on():
                step_right( 1, default_step_sleep )
        if limit_switch_right_on():
            while limit_switch_right_on():
                step_left( 1, default_step_sleep )
        if calibrate_manually_up_reached:
            print( ">     up position reached" )
            break
        # time.sleep( 0.05 )

    print(">   canvas_max_y_steps: " + str( canvas_max_y_steps ))
    # TODO, what do we do if canvas_max_y_steps is negative? (the user totally borked the process)

    # going back to center
    print(">   going back to y center")
    step_down( int(round(canvas_max_y_steps/2)), default_step_sleep )

    print( ">   left" )
    while True:
        if calibrate_manually_step_down:
            step_down( 1, default_step_sleep*2 )
        elif calibrate_manually_step_up:
            step_up( 1, default_step_sleep*2 )
        elif calibrate_manually_step_left:
            step_left( 1, default_step_sleep*2 )
        elif calibrate_manually_step_right:
            step_right( 1, default_step_sleep*2 )
        if limit_switch_bottom_on():
            while limit_switch_bottom_on():
                step_up( 1, default_step_sleep )
        if limit_switch_top_on():
            while limit_switch_top_on():
                step_down( 1, default_step_sleep )
        if limit_switch_left_on():
            while limit_switch_left_on():
                step_right( 1, default_step_sleep )
        if limit_switch_right_on():
            while limit_switch_right_on():
                step_left( 1, default_step_sleep )
        if calibrate_manually_left_reached:
            print( ">     left position reached" )
            break
        # time.sleep( 0.05 )
    print(">   counting steps right")
    while True:
        if calibrate_manually_step_down:
            step_down( 1, default_step_sleep*2 )
        elif calibrate_manually_step_up:
            step_up( 1, default_step_sleep*2 )
        elif calibrate_manually_step_left:
            step_left( 1, default_step_sleep*2 )
            canvas_max_x_steps -= 1
        elif calibrate_manually_step_right:
            step_right( 1, default_step_sleep*2 )
            canvas_max_x_steps += 1
        if limit_switch_bottom_on():
            while limit_switch_bottom_on():
                step_up( 1, default_step_sleep )
        if limit_switch_top_on():
            while limit_switch_top_on():
                step_down( 1, default_step_sleep )
        if limit_switch_left_on():
            while limit_switch_left_on():
                step_right( 1, default_step_sleep )
        if limit_switch_right_on():
            while limit_switch_right_on():
                step_left( 1, default_step_sleep )
        if calibrate_manually_right_reached:
            print( ">     right position reached" )
            break
        # time.sleep( 0.05 )

    print(">   canvas_max_x_steps: " + str( canvas_max_x_steps ))
    # TODO, what do we do if canvas_max_y_steps is negative? (the user totally borked the process)

    print(">   going back to x center")
    step_left( int(round(canvas_max_x_steps/2)), default_step_sleep )

    # setting canvas_max_x and canvas_max_y
    if canvas_max_y_steps>canvas_max_x_steps:
        canvas_max_x = 100.0
        canvas_max_y = 100.0*(float(canvas_max_y_steps)/float(canvas_max_x_steps))
    elif canvas_max_x_steps>canvas_max_y_steps:
        canvas_max_y = 100.0
        canvas_max_x = 100.0*(float(canvas_max_x_steps)/float(canvas_max_y_steps))
    else:
        canvas_max_x = 100.0
        canvas_max_y = 100.0
    print("> calibration finished, canvas area: " + str(canvas_max_x) + "x" + str(canvas_max_y))

    # all righty it's right in the middle now and we know it
    current_x = canvas_max_x/2
    current_y = canvas_max_y/2

    print(">   head currently at " + str(current_x) + "," + str(current_y))

    calibration_ongoing = False
    calibration_done = True
    command_running = False


def calibrate_automatic():
    global canvas_max_x_steps, canvas_max_y_steps, canvas_max_x, canvas_max_y, current_x, current_y, calibration_ongoing, calibration_done, command_running

    command_running = True
    print("> calibrate_automatic")

    calibration_ongoing = True

    if limit_switch_bottom_on(): # we need to move away from the bottom
        print(">   too close to bottom edge, looks like we need to get away from it")
        while True:
            if limit_switch_top_on():
                print("error: calibration issues")
                exit( 1 )
            step_up( 1, default_step_sleep )
            if not limit_switch_bottom_on():
                break ;
        step_up( 100, default_step_sleep )
    if limit_switch_top_on(): # we need to move away from the top
        print(">   too close to top edge, looks like we need to get away from it")
        while True:
            if limit_switch_bottom_on():
                print("error: calibration issues")
                exit( 1 )
            step_down( 1, default_step_sleep )
            if not limit_switch_top_on():
                break ;
        step_down( 100, default_step_sleep )
    if limit_switch_left_on(): # we need to move away from the left
        print(">   too close to left edge, looks like we need to get away from it")
        while True:
            if limit_switch_right_on():
                print("error: calibration issues")
                exit( 1 )
            step_right( 1, default_step_sleep )
            if not limit_switch_left_on():
                break ;
        step_right( 100, default_step_sleep )
    if limit_switch_right_on(): # we need to move away from the right
        print(">   too close to right edge, looks like we need to get away from it")
        while True:
            if limit_switch_left_on():
                print("error: calibration issues")
                exit( 1 )
            step_left( 1, default_step_sleep )
            if not limit_switch_right_on():
                break ;
        step_left( 100, default_step_sleep )

    print(">   down")
    while True:
        step_down( 1, default_step_sleep )
        if limit_switch_bottom_on():
            break ;
    while limit_switch_bottom_on():
        step_up( 1, default_step_sleep )
    # just to be safe
    time.sleep( 0.5 )
    step_up( safety_step_padding, default_step_sleep )
    time.sleep( 0.5 )

    print(">   counting steps up")
    while True:
        step_up( 1, default_step_sleep )
        canvas_max_y_steps = canvas_max_y_steps + 1
        if limit_switch_top_on():
            break ;
    while limit_switch_top_on():
        step_down( 1, default_step_sleep )
    # just to be safe
    time.sleep( 0.5 )
    step_down( safety_step_padding, default_step_sleep )
    time.sleep( 0.5 )

    canvas_max_y_steps = canvas_max_y_steps - safety_step_padding
    print(">   canvas_max_y_steps: " + str( canvas_max_y_steps ))

    # going back to center
    print(">   going back to y center")
    step_down( int(round(canvas_max_y_steps/2)), default_step_sleep )

    time.sleep( 0.5 )
    print(">   left")
    while True:
        step_left( 1, default_step_sleep )
        if limit_switch_left_on():
            break
    while limit_switch_left_on():
        step_right( 1, default_step_sleep )
    # just to be safe
    time.sleep( 0.5 )
    step_right( safety_step_padding, default_step_sleep )
    time.sleep( 0.5 )
    print(">   counting steps right")
    while True:
        step_right( 1, default_step_sleep )
        canvas_max_x_steps = canvas_max_x_steps + 1
        if limit_switch_right_on():
            break ;
    # just to be safe
    time.sleep( 0.5 )
    step_left( safety_step_padding, default_step_sleep )
    time.sleep( 0.5 )

    canvas_max_x_steps = canvas_max_x_steps - safety_step_padding
    print(">   canvas_max_x_steps: " + str( canvas_max_x_steps ))

    print(">   going back to x center")
    step_left( int(round(canvas_max_x_steps/2)), default_step_sleep )

    # setting canvas_max_x and canvas_max_y
    if canvas_max_y_steps>canvas_max_x_steps:
        canvas_max_x = 100.0
        canvas_max_y = 100.0*(float(canvas_max_y_steps)/float(canvas_max_x_steps))
    elif canvas_max_x_steps>canvas_max_y_steps:
        canvas_max_y = 100.0
        canvas_max_x = 100.0*(float(canvas_max_x_steps)/float(canvas_max_y_steps))
    else:
        canvas_max_x = 100.0
        canvas_max_y = 100.0
    print("> calibration finished, canvas area: " + str(canvas_max_x) + "x" + str(canvas_max_y))

    # all righty it's right in the middle now and we know it
    current_x = canvas_max_x/2
    current_y = canvas_max_y/2

    print(">   head currently at " + str(current_x) + "," + str(current_y))

    calibration_ongoing = False
    calibration_done = True
    command_running = False


def cleanup():
    global command_server_thread, command_server_server, command_server_client, clean_up_before_exit, mg_penstrokes_processing_thread_stop

    print( "> cleaning up" )
    clean_up_before_exit = True
    mg_penstrokes_processing_thread_stop = True
    if type(command_server_client)!=type(True):
        command_server_client.close()
    # artificial client to satisfy the socket server wait condition so we can terminate its thread cleanly
    artificial_client = socket.socket( socket.AF_INET, socket.SOCK_STREAM ).connect( (command_server_address, command_server_port))
    command_server_server.close()
    command_server_thread.join()
    GPIO.output( stepper_bottom_step, GPIO.LOW )
    GPIO.output( stepper_bottom_dir, GPIO.LOW )
    GPIO.output( stepper_top_step, GPIO.LOW )
    GPIO.output( stepper_top_dir, GPIO.LOW )
    GPIO.output( stepper_bottom_enable, GPIO.HIGH )
    GPIO.output( stepper_top_enable, GPIO.HIGH )
    # pwm.set_PWM_frequency( servo, 0 )
    # pwm.set_servo_pulsewidth( servo, 0 )


def command_server():
    global command_server_server, command_server_client, command_running
    global calibrate_manually_step_down, calibrate_manually_step_up, calibrate_manually_step_left, calibrate_manually_step_right
    global calibrate_manually_down_reached, calibrate_manually_up_reached, calibrate_manually_left_reached, calibrate_manually_right_reached
    global pause_draw, stop_draw, draw_going
    global mg_penstrokes_processing_thread, mg_penstrokes_processing_thread_stop, mg_link_session_id, mg_link_socket
    global ink_refill_routine_enabled, ink_refill_routine, ink_refill_every_penstroke, ink_refill_every_x
    global pen_down_pulse_width, pen_up_pulse_width, pen_down_action_time, pen_up_action_time, pen_down_sleep_before_move_time, pen_up_sleep_before_move_time, acceleration_steps, deceleration_steps, acceleration_slow_steps_sleep, acceleration_fast_steps_sleep, default_step_sleep
    global ht_live_keyboard_on, ht_penstrokes, ht_penstrokes_mutex, ht_penstrokes_processing_thread_stop, ht_penstrokes_processing_thread
    global acquire_instructions_from_web_on, stop_acquire_instructions_from_web
    global gondola_steps_per_cm, gondola_reserve_margin, gondola_max_travel, gondola_calibration_length

    pen_up()

    command_server_server = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    command_server_server.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )
    command_server_server.bind( (command_server_address, command_server_port) )
    command_server_server.listen()
    print( "# command server started waiting for connections" )
    while True and not clean_up_before_exit:
        command_server_client, address = command_server_server.accept()
        # we only accept 1 command per connection
        buf = command_server_client.recv( 256 )
        buf = buf.strip().decode("utf-8") ;
        if len( buf )>0:
            if buf=="calibrate_automatic":
                print( "# calibrate_automatic" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                    global pen_down_pulse_width, pen_up_pulse_width, pen_down_action_time, pen_up_action_time, pen_down_sleep_before_move_time, pen_up_sleep_before_move_time, acceleration_steps, deceleration_steps
                else:
                    command_server_client.sendall( b"ok" )
                    command_server_client.close()
                    calibrate_automatic()
            elif buf=="pen_up":
                print( "# pen_up" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                else:
                    command_server_client.sendall( b"ok" )
                    command_server_client.close()
                    pen_up()
            elif buf=="pen_down":
                print( "# pen_down" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                else:
                    command_server_client.sendall( b"ok" )
                    command_server_client.close()
                    pen_down()
            elif buf.startswith("go_to("):
                print( "# go_to" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                else:
                    if not calibration_done:
                        print( "#   calibration needed" )
                        command_server_client.sendall( b"calibration_needed" )
                        command_server_client.close()
                    else:
                        x = float( buf.split("(")[1].split(",")[0].strip() )
                        y = float( buf.split("(")[1].split(",")[1].replace(")", "").strip() )
                        if x<0 or x>canvas_max_x or y<0 or y>canvas_max_y:
                            command_running = False
                            print( "#   out of range, canvas_max_x: " + str(canvas_max_x) + ", canvas_max_y: " + str(canvas_max_y) )
                            command_server_client.sendall( b"out_of_range" )
                            command_server_client.close()
                        else:
                            go_to( x, y )
                            command_server_client.sendall( b"ok" )
                            command_server_client.close()
            elif buf=="update_ink_refill_routine":
                print( "# update_ink_refill_routine" )
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                f = open( "/data/ink_refill_routine", "r" )
                ink_refill_routine = f.read()
                print( ink_refill_routine )
                f.close()
                save_variables_to_disk()
            elif buf=="draw":
                print( "# draw" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                else:
                    if not calibration_done:
                        print( "#   calibration needed" )
                        command_server_client.sendall( b"calibration_needed" )
                        command_server_client.close()
                    else:
                        command_server_client.sendall( b"ok" )
                        command_server_client.close()
                        # f = open( "/data/tcode", "r" )
                        # tcode = f.read()
                        # f.close()
                        tcode = "file"
                        draw_thread = threading.Thread( target=draw, args=[tcode] )
                        draw_thread.start()
                        #draw( tcode )
            elif buf=="pause":
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                pause_draw = True
            elif buf=="play":
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                pause_draw = False
            elif buf=="stop":
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                stop_draw = True
            # the following for manual calibration
            elif buf=="calibrate_manually":
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                calibrate_manually_thread = threading.Thread( target=calibrate_manually )
                calibrate_manually_thread.start()
            elif buf=="calibrate_manually_step_down":
                calibrate_manually_step_down = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_step_up":
                calibrate_manually_step_up = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_step_left":
                calibrate_manually_step_left = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_step_right":
                calibrate_manually_step_right = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_step_stop":
                calibrate_manually_step_down = False
                calibrate_manually_step_up = False
                calibrate_manually_step_left = False
                calibrate_manually_step_right = False
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_fixate_down":
                calibrate_manually_down_reached = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_fixate_up":
                calibrate_manually_up_reached = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_fixate_left":
                calibrate_manually_left_reached = True
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="calibrate_manually_fixate_right":
                calibrate_manually_right_reached = True
                while not calibration_done:
                    time.sleep( 0.1 )
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="disconnect_from_mg_session":
                print( "# disconnecting from MG websocket" )
                mg_link_session_id = ""
                mg_penstrokes_processing_thread_stop = True
                try:
                    # TODO clear all penstrokes to be processed
                    mg_link_socket.disconnect()
                except:
                    print( "# well it looks like that didn't go so well..." )
                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf.startswith("ink_refill_routine_enabled("):
                print( buf )
                print( "# ink_refill_routine_enabled" )
                value = buf.split( "(" )[1].replace( ")", "" ).strip() ;
                if value=="true":
                    ink_refill_routine_enabled = True
                else:
                    ink_refill_routine_enabled = False
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("ink_refill_every_penstroke("):
                print( buf )
                print( "# ink_refill_every_penstroke" )
                value = buf.split( "(" )[1].replace( ")", "" ).strip() ;
                if value=="true":
                    ink_refill_every_penstroke = True
                else:
                    ink_refill_every_penstroke = False
                command_server_client.sendall( b"ok" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("ink_refill_every_x("):
                print( buf )
                print( "# ink_refill_every_x" )
                value = buf.split( "(" )[1].replace( ")", "" ).strip() ;
                if type(value)==int:
                    ink_refill_every_x = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("default_step_sleep("):
                print( buf )
                print( "# default_step_sleep" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    default_step_sleep = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("pen_down_pulse_width("):
                print( buf )
                print( "# pen_down_pulse_width" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    pen_down_pulse_width = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("pen_up_pulse_width("):
                print( buf )
                print( "# pen_up_pulse_width" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    pen_up_pulse_width = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("pen_down_action_time("):
                print( buf )
                print( "# pen_down_action_time" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    pen_down_action_time = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("pen_up_action_time("):
                print( buf )
                print( "# pen_up_action_time" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    pen_up_action_time = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("pen_down_sleep_before_move_time("):
                print( buf )
                print( "# pen_down_sleep_before_move_time" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    pen_down_sleep_before_move_time = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("pen_up_sleep_before_move_time("):
                print( buf )
                print( "# pen_up_sleep_before_move_time" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    pen_up_sleep_before_move_time = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("acceleration_steps("):
                print( buf )
                print( "# acceleration_steps" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    acceleration_steps = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("deceleration_steps("):
                print( buf )
                print( "# deceleration_steps" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    deceleration_steps = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("acceleration_slow_steps_sleep("):
                print( buf )
                print( "# acceleration_slow_steps_sleep" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    acceleration_slow_steps_sleep = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("acceleration_fast_steps_sleep("):
                print( buf )
                print( "# acceleration_fast_steps_sleep" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    acceleration_fast_steps_sleep = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("gondola_steps_per_cm("):
                print( buf )
                print( "# gondola_steps_per_cm" )
                value = int( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==int:
                    gondola_steps_per_cm = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("gondola_reserve_margin("):
                print( buf )
                print( "# gondola_reserve_margin" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    gondola_reserve_margin = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("gondola_max_travel("):
                print( buf )
                print( "# gondola_max_travel" )
                value = float( buf.split( "(" )[1].replace( ")", "" ).strip() ) ;
                if type(value)==float:
                    gondola_max_travel = value
                    command_server_client.sendall( b"ok" )
                else:
                    command_server_client.sendall( b"invalid" )
                command_server_client.close()
                save_variables_to_disk()
            elif buf.startswith("gondola_calibrate("):
                gondola_motor_distance = buf.split( "(" )[1].replace(")", "").strip()
                try:
                    f = open( "/var/gondola_motor_distance", "w" )
                    f.write( gondola_motor_distance )
                    f.close()
                except:
                    # oops
                    print( "- file IO error" )
                gondola_calibration_length = float( gondola_motor_distance )
                calibrate_manually()
            elif buf.startswith("connect_to_mg_session("):
                mg_link_session_id = buf.split( "(" )[1].replace(")", "").strip()

                #mg_link_socket = socketio.Client(logger=True, engineio_logger=True)
                mg_link_socket = socketio.Client()

                @mg_link_socket.event
                def plotter_pen_down( cord ):
                    global mg_penstrokes
                    print( "& plotter_pen_down (" + str(cord['x']) + "," + str(cord['y']) + ")" )
                    mg_penstrokes.append( "go_to(" + str(cord['x']) + "," + str(cord['y']) + ")\npen_down()" )

                @mg_link_socket.event
                def plotter_pen_move( cord ):
                    global mg_penstrokes
                    print( "& plotter_pen_move (" + str(cord['x']) + "," + str(cord['y']) + ")" )
                    mg_penstrokes.append( "go_to(" + str(cord['x']) + "," + str(cord['y']) + ")" )

                @mg_link_socket.event
                def plotter_pen_up():
                    global mg_penstrokes
                    print( "& plotter_pen_up" )
                    mg_penstrokes.append( "pen_up()" )

                @mg_link_socket.event
                def plotter_penstroke( cords ):
                    print( "& plotter_penstroke" )
                    add_to_mg_penstrokes( cords )

                @mg_link_socket.event
                def canvas_size( width, height ):
                    global mg_canvas_width, mg_canvas_height
                    print( "& canvas_size " + str(width) + "x" + str(height) )
                    mg_canvas_width = int(width)
                    mg_canvas_height = int(height)

                @mg_link_socket.event
                def connect():
                    global mg_penstrokes, mg_penstrokes_processing_thread_stop, mg_penstrokes_processing_thread
                    print( "& connected to MG websocket" )
                    mg_penstrokes = []
                    mg_penstrokes_processing_thread_stop = False
                    mg_penstrokes_processing_thread = threading.Thread( target=mg_penstrokes_processing )
                    mg_penstrokes_processing_thread.start()

                @mg_link_socket.event
                def connect_error():
                    global mg_link_session_id
                    print( "& ERROR: connection to MG websocket failed" )
                    mg_link_session_id = ""

                @mg_link_socket.event
                def disconnect():
                    global mg_link_session_id, mg_penstrokes_processing_thread_stop
                    print( "& disconnected from MG websocket" )
                    mg_link_session_id = ""
                    mg_penstrokes_processing_thread_stop = True

                import requests
                websocket_server = requests.post('https://plottybot.mandalagaba.com/get_websocket_server.php', data = {'session_id':mg_link_session_id})
                if websocket_server.status_code!=200:
                    print( "& could not get websocket server to connect to" )
                    mg_link_socket = null
                else:
                    actual_server = json.loads( websocket_server.text )
                    if "server" not in actual_server:
                        print( "& invalid server gotten back with: " + websocket_server )
                        mg_link_socket = null
                    else:
                        actual_server = actual_server['server']
                        print( "& websocket server: " + actual_server  )

                        requests.get('https://plottybot.mandalagaba.com/plotter_authenticate.php')
                        mg_link_socket.connect( "https://" + actual_server + "/socket.io/?session_id=" + mg_link_session_id + "&facet=plotter&window_width=" + str(int(canvas_max_x)) + "&window_height=" + str(int(canvas_max_y)) + "&detailed_initial_dump=false&access_type=o&detailed_initial_dump_speed=1" ) # TODO access_type o for this really? # TODO need to get_websocket_server really
                        print( "& socket_id", mg_link_socket.sid )

                command_server_client.sendall( b"ok" )
                command_server_client.close()
            elif buf=="ht_live_keyboard_on":
                print( "# ht_live_keyboard_on" )
                ht_live_keyboard_on = True
                ht_penstrokes = []
                ht_penstrokes_mutex = False
                ht_penstrokes_processing_thread_stop = False
                ht_penstrokes_processing_thread = threading.Thread( target=ht_penstrokes_processing )
                ht_penstrokes_processing_thread.start()
            elif buf=="ht_live_keyboard_off":
                print( "# ht_live_keyboard_off" )
                ht_live_keyboard_on = False
                ht_penstrokes_processing_thread_stop = True
            elif buf=="test_ink_refill_routine":
                print( "# test_ink_refill_routine" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    print( "#   command already running" )
                else:
                    if not calibration_done:
                        print( "#   calibration needed" )
                    else:
                        command_server_client.sendall( b"ok" )
                        run_ink_refill_routine_thread = threading.Thread( target=run_ink_refill_routine )
                        run_ink_refill_routine_thread.start()
                command_server_client.close()
            elif buf=="add_to_ht_penstrokes":
                print( "# add_to_ht_penstrokes" )
                f = open( "/data/add_to_ht_penstrokes", "r" )
                new_pcode = f.read()
                f.close()
                add_to_ht_penstrokes( new_pcode )
            elif buf=="get_status":
                status = {}
                status['calibration_done'] = calibration_done
                status['draw_going'] = draw_going
                status['pause_draw'] = pause_draw
                status['pen_down_pulse_width'] = pen_down_pulse_width
                status['pen_up_pulse_width'] = pen_up_pulse_width
                status['pen_down_action_time'] = pen_down_action_time
                status['pen_up_action_time'] = pen_up_action_time
                status['pen_down_sleep_before_move_time'] = pen_down_sleep_before_move_time
                status['pen_up_sleep_before_move_time'] = pen_up_sleep_before_move_time
                status['acceleration_steps'] = acceleration_steps
                status['deceleration_steps'] = deceleration_steps
                status['acceleration_slow_steps_sleep'] = acceleration_slow_steps_sleep
                status['acceleration_fast_steps_sleep'] = acceleration_fast_steps_sleep
                status['default_step_sleep'] = default_step_sleep
                status['canvas_max_x'] = canvas_max_x
                status['canvas_max_y'] = canvas_max_y
                status['mg_link_session_id'] = mg_link_session_id
                status['ink_refill_routine_enabled'] = ink_refill_routine_enabled
                status['ink_refill_every_penstroke'] = ink_refill_every_penstroke
                status['ink_refill_every_x'] = ink_refill_every_x
                status['ink_refill_routine'] = ink_refill_routine
                status['gondola_steps_per_cm'] = gondola_steps_per_cm
                status['gondola_reserve_margin'] = gondola_reserve_margin
                status['gondola_max_travel'] = gondola_max_travel
                status['ht_live_keyboard_on'] = ht_live_keyboard_on
                status['acquire_instructions_from_web_on'] = acquire_instructions_from_web_on
                status['limit_switch_bottom_on'] = limit_switch_bottom_on()
                status['limit_switch_top_on'] = limit_switch_top_on()
                status['limit_switch_left_on'] = limit_switch_left_on()
                status['limit_switch_right_on'] = limit_switch_right_on()
                status['error'] = get_potential_errors() ;
                command_server_client.sendall( json.dumps(status).encode() )
                command_server_client.close()
            elif buf=="reset_to_defaults":
                pen_down_pulse_width = pen_down_pulse_width_default
                pen_up_pulse_width = pen_up_pulse_width_default
                pen_down_action_time = pen_down_action_time_default
                pen_up_action_time = pen_up_action_time_default
                pen_down_sleep_before_move_time = pen_down_sleep_before_move_time_default
                pen_up_sleep_before_move_time = pen_up_sleep_before_move_time_default
                default_step_sleep = default_step_sleep_default
                acceleration_steps = acceleration_steps_default
                deceleration_steps = deceleration_steps_default
                acceleration_slow_steps_sleep = acceleration_slow_steps_sleep_default
                acceleration_fast_steps_sleep = acceleration_fast_steps_sleep_default
                ink_refill_routine_enabled = ink_refill_routine_enabled_default
                save_variables_to_disk() ;
            elif buf=="test_bottom_stepper":
                print( "# test_bottom_stepper" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                else:
                    command_server_client.sendall( b"ok" )
                    command_server_client.close()
                    test_bottom_stepper_thread = threading.Thread( target=test_bottom_stepper )
                    test_bottom_stepper_thread.start()
            elif buf=="test_top_stepper":
                print( "# test_top_stepper" )
                if command_running:
                    command_server_client.sendall( b"command_already_running" )
                    command_server_client.close()
                    print( "#   command already running" )
                else:
                    command_server_client.sendall( b"ok" )
                    command_server_client.close()
                    test_top_stepper_thread = threading.Thread( target=test_top_stepper )
                    test_top_stepper_thread.start()
            elif buf.startswith("acquire_instructions_from_web("):
                print( "- acquire_instructions_from_web" )
                if not calibration_done:
                    print( "-   calibration needed" )
                    command_server_client.sendall( b"calibration_needed" )
                    command_server_client.close()
                else:
                    command_server_client.sendall( b"ok" )
                    command_server_client.close()
                    stop_acquire_instructions_from_web = False
                    url = buf.split( "(" )[1].replace(")", "").strip()
                    acquire_instructions_from_web_thread = threading.Thread( target=acquire_instructions_from_web, args=[url] )
                    acquire_instructions_from_web_thread.start()
            elif buf.startswith("stop_acquire_instructions_from_web"):
                print( "- stop_acquire_instructions_from_web" )
                stop_acquire_instructions_from_web = True
                acquire_instructions_from_web_on = False
            else:
                print(( "# error: unknown command: " + str(buf.strip()) ))


def add_to_mg_penstrokes( cords ):
    global mg_penstrokes

    parsed_tcode = ""
    first = True
    for cord in cords:
        parsed_tcode += "go_to( " + str(cord['x']) + ", " + str(cord['y']) + " )\n"
        if first:
            parsed_tcode += "pen_down()\n"
            first = False
    parsed_tcode += "pen_up()\n"
    mg_penstrokes.append( parsed_tcode )


def mg_penstrokes_processing():
    global mg_penstrokes, mg_penstrokes_processing_thread_stop, mg_canvas_width, mg_canvas_height

    print( "@ mg_penstrokes_processing thread started" )

    while True:
        if len(mg_penstrokes)>0:
            x_ratio = canvas_max_x / mg_canvas_width
            y_ratio = canvas_max_y / mg_canvas_height
            ratio = min( x_ratio, y_ratio )

            instructions = mg_penstrokes.pop( 0 )
            for line in instructions.split( "\n" ):
                orig_line = line
                #print line
                line = line.split( "(" )
                if len(line)==2:
                    command = line[0]
                    params = line[1].replace( ")", "" ).split( "," )
                    if command=="go_to" and len(params)==2:
                        to_x = float(params[0]) * ratio
                        to_y = canvas_max_y - (float(params[1]) * ratio)
                        if to_x<0.0 or to_x>canvas_max_x or to_y<0.0 or to_y>canvas_max_y:
                            print("@ WARNING: out of bound " + str(to_x) + "," + str(to_y))
                        else:
                            print( "@ go_to( " + str(to_x) + ", " + str(to_y) + " )" )
                            go_to( to_x, to_y )
                    elif command=="pen_up":
                        print( "@ pen_up()" )
                        pen_up()
                    elif command=="pen_down":
                        print( "@ pen_down()" )
                        pen_down()
                    else:
                        print("@ WARNING: can't parse line: " + orig_line)
        else:
            time.sleep( 0.2 )
        if mg_penstrokes_processing_thread_stop:
            break

    print( "@ mg_penstrokes_processing thread finished" )


def add_to_ht_penstrokes( new_pcode ):
    global ht_penstrokes, ht_penstrokes_mutex
    while ht_penstrokes_mutex==True:
        time.sleep( 0.05 )
    ht_penstrokes_mutex = True
    ht_penstrokes.append( new_pcode )
    ht_penstrokes_mutex = False


def ht_penstrokes_processing():
    global ht_penstrokes, ht_penstrokes_processing_thread_stop

    print( "= ht_penstrokes_processing thread started" )

    while True:
        if len(ht_penstrokes)>0:
            while ht_penstrokes_mutex==True:
                time.sleep( 0.05 )
            ht_penstrokes_mutex = True
            instructions = ht_penstrokes.pop( 0 )
            ht_penstrokes_mutex = False
            for line in instructions.split( "\n" ):
                orig_line = line
                #print line
                line = line.split( "(" )
                if len(line)==2:
                    command = line[0]
                    params = line[1].replace( ")", "" ).split( "," )
                    if command=="go_to" and len(params)==2:
                        to_x = float(params[0])
                        to_y = float(params[1])
                        if to_x<0.0 or to_x>canvas_max_x or to_y<0.0 or to_y>canvas_max_y:
                            print("= WARNING: out of bound " + str(to_x) + "," + str(to_y))
                        else:
                            print( "= go_to( " + str(to_x) + ", " + str(to_y) + " )" )
                            go_to( to_x, to_y )
                    elif command=="pen_up":
                        print( "= pen_up()" )
                        pen_up()
                    elif command=="pen_down":
                        print( "= pen_down()" )
                        pen_down()
                    else:
                        print("= WARNING: can't parse line: " + orig_line)
        else:
            time.sleep( 0.2 )
        if ht_penstrokes_processing_thread_stop:
            break

    print( "= ht_penstrokes_processing thread finished" )


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print( "ctrl+c" )
        cleanup()
        exit( 0 )
