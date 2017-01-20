import pyb

pyb.LED(2).on()                 # indicate we are waiting for switch press
pyb.delay(200)                 # wait for user to maybe press the switch
pyb.LED(2).off()

pyb.LED(2).on()                 # indicate we are waiting for switch press
pyb.delay(200)                 # wait for user to maybe press the switch
pyb.LED(2).off()

pyb.LED(2).on()                 # indicate we are waiting for switch press
pyb.delay(200)                 # wait for user to maybe press the switch
pyb.LED(2).off()

pyb.LED(2).on()                 # indicate we are waiting for switch press
pyb.delay(200)                 # wait for user to maybe press the switch
pyb.LED(2).off()

# import pyb
#
# for j in range(5):
#     for i in range(1,5):
#         pyb.LED(i).on()
#         if i==4:
#             pyb.LED(1).off()
#         else:
#             pyb.LED(i-1).off()
#         pyb.delay(50)
# ============= EOF =============================================
