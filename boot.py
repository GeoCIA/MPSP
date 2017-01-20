import pyb

pyb.LED(3).on()                 # indicate we are waiting for switch press
pyb.delay(2000)                 # wait for user to maybe press the switch
switch_value = pyb.Switch()()   # sample the switch at end of delay
pyb.LED(3).off()                # indicate that we finished waiting for the switch

pyb.LED(4).on()                 # indicate that we are selecting the mode

if switch_value:
    pyb.usb_mode('CDC+MSC')
    # pyb.main('main.py')
else:
    pyb.usb_mode('CDC+HID')
    pyb.main('mpsp_main.py')

pyb.LED(4).off()                # indicate that we finished selecting the mode
