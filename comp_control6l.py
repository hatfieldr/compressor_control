import tkinter as tk
from tkinter import *
from PIL import Image, ImageTk
import time
import paho.mqtt.client as mqtt #import the client1
import json
import datetime
import numpy as np
from numpy import mean
import sys
import uuid
import threading
from threading import Event
from threading import Lock
import spidev
#import max6675
import board
import busio
import adafruit_mcp9600
from digitalio import Direction
from adafruit_mcp230xx.mcp23008 import MCP23008
import time
import os
from ADCPi import ADCPi
from email.charset import Charset
from pickletools import float8
from re import T
from string import hexdigits
from sys import byteorder
from tkinter import X
from unittest.util import strclass
import serial



#define main screen and additional frames / pages
root = Tk()
#root.geometry("1366x760")
root.attributes('-fullscreen', True)
root.config(cursor='none')

global T1,T2,T3,T4,T5 #temp1_txt
T1= 0.0
T2=0.0
T3=0.0
T4=0.0
T5=0.0
global P1,P1_zero,P1_cal,P2,P3,P4,P5
P1 = 0.0
P2=0.0
P3=0.0
P4=0.0
P5=0.0

P1_zero = 0.481845
P1_cal = 25.941952

global crc,p
p=0

global oph2,opc1,opc2,opc3,op4,op5,op6,opvent
oph2=False #output plug pin 0
opc1=False  #output plug pin 1
opc2=False  #output plug pin 2
opc3=False  #output plug pin 3
opvent=True  #output plug pin 4
op5=False   #output plug pin 5
op6=False   #output plug pin 6
op7=False   #output plug pin 7

frame1 = Frame(root,)
frame1.configure(bg="black")
frame1.pack(fill=BOTH, expand=1,)

frame2 = Frame(root,)
frame2.configure(bg="dark blue")
#frame2.pack(fill=BOTH, expand=1,)

global car_data_status_txt, car_tank_temp_txt, car_tank_pressure_txt, car_filling_status_txt
car_data_status_txt = StringVar()
car_data_status_txt.set("Not Connected")
car_tank_temp_txt = StringVar()
car_tank_temp_txt.set("Temp: "+str(T5)+u'\u00b0'+'C')
car_tank_pressure_txt = StringVar()
car_tank_pressure_txt.set("Pressure: "+str(P5)+" Bar")
car_filling_status_txt = StringVar()
car_filling_status_txt.set("Stoped")
data_frame_no_txt = StringVar()
data_frame_no_txt.set("0")

global start_car_data,car_data_info,sp,car_update_time,car_data_stat
#start_car_data=time.time()-1
#car_update_time=time.time()-1
#car_data_info=False
# sp=False
car_data_stat = "NULL"


global mp, mt,fc, fn
mp=0.0
mt=0.0
fc=" "
fn=0
#create event flags - can be used in all threads
crc_event = threading.Event()
car_data_info_event = threading.Event()
sp_event = threading.Event()

#set event flags for passing data between threads
crc_event.clear()
car_data_info_event.clear()
sp_event.clear()

#create data locks
mp_lock = threading.Lock()
fc_lock = threading.Lock()
mt_lock = threading.Lock()
fn_lock = threading.Lock()

#set up I2C bus
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

def terminate():
    global root
    sp_event.clear()
    sys.exit()
    root.destroy()

def show_monitor():
    global page_control, page_monitor, page_fill, dummy
    frame1.pack_forget()
    frame2.pack(fill=BOTH, expand=1,)
    page_monitor=True
    page_control=False
    page_fill=False

def show_control():
    global page_control, page_monitor
    frame2.pack_forget()
    frame1.pack(fill=BOTH, expand=1,)
    page_monitor=False
    page_control=True

def air_comp_on():
    global opair
    opair = True
    compressor_on.config(bg='green')
    compressor_off.config(bg='coral')

def air_comp_off():
    global opair
    opair = False
    compressor_off.config(bg='red')
    compressor_on.config(bg='pale green')

def h2_on_cont():
    global oph2
    oph2=True
    h2_on.config(bg='green')
    h2_off.config(bg='coral')

def h2_off_cont():
    global oph2
    oph2=False
    h2_off.config(bg='red')
    h2_on.config(bg='pale green')

def comp1_on_cont():
    global opc1
    opc1=True
    comp1_on.config(bg='green')
    comp1_off.config(bg='coral')


def comp1_off_cont():
    global opc1
    opc1=False
    comp1_on.config(bg='pale green')
    comp1_off.config(bg='red')

def comp2_on_cont():
    global opc2
    opc2=True
    comp2_on.config(bg='green')
    comp2_off.config(bg='coral')

def comp2_off_cont():
    global opc2
    opc2=False
    comp2_on.config(bg='pale green')
    comp2_off.config(bg='red')

def comp3_on_cont():
    global opc3
    opc3=True
    comp3_on.config(bg='green')
    comp3_off.config(bg='coral')

def comp3_off_cont():
    global opc3
    opc3=False
    comp3_on.config(bg='pale green')
    comp3_off.config(bg='red')

def connect_serial():
    global SerialObj,car_data_info,sp,car_data_stat
    #sp is serial port flag
    if not sp_event.is_set():
        try:
            #sp=True
            SerialObj = serial.Serial('/dev/ttyS0') # open the Serial Port
            SerialObj.baudrate = 38400
            SerialObj.bytesize = 8
            SerialObj.parity = 'N'
            SerialObj.stopbits = 1
            SerialObj.timeout = 10
                                            # /dev/ttyUSBx format on Linux
                                            #
                                            # Eg /dev/ttyUSB0
                                            # SerialObj = serial.Serial('/dev/ttyUSB0')
            print('Port Details ->',SerialObj)      # display the properties of Serial Port
    
        except serial.SerialException as var :
            print('An Exception Occured')
            print('Exception Details-> ', var)
            car_data_stat="No connection"

        else:
            print('Serial Port Opened')
            sp_event.set()
            car_data_stat="waiting"
            time.sleep(1)
            #start car data serial thread
            get_car_data()
    else:
       print("Already open!")  #if sp is true, serial port should already be open!
       car_data_stat="waiting"
       pass

def disconnect_serial():
    global SerialObj,sp,car_data_info,car_data_stat

    car_data_info_event.clear()
    car_data_stat="Disconnected"
    if sp_event.is_set():
        SerialObj.close()          # Close the port if it is open/ if flag is true
        sp_event.clear()
        print("Serial port closed")
    else:
        pass

title_text= Label(frame1,text= "Wickham Energy Hydrogen Compression System",font=("Arial Bold",28)).grid(row=1,padx=150)

#get images for main screen and resize them. Make sure you have the images in the "images" directory, a sub directory of your python files

lpta=Image.open("images/mahytec.png")
lptb=lpta.resize((200,250))
lpt=ImageTk.PhotoImage(lptb)

comp1a=Image.open("images/gasbooster2a.png")
comp1b=comp1a.resize((250,200))
comp1=ImageTk.PhotoImage(comp1b)

comp2a=Image.open("images/gasbooster2b.png")
comp2b=comp2a.resize((250,200))
comp2=ImageTk.PhotoImage(comp2b)

comp3a=Image.open("images/gasbooster2c.png")
comp3b=comp3a.resize((250,200))
comp3=ImageTk.PhotoImage(comp3b)

cara=Image.open("images/nexo2.png")
carb=cara.resize((250,200))
car=ImageTk.PhotoImage(carb)



#create buttons and labels
exit = Button(frame1,font=80, height=2,width=6, text="Exit", command=terminate)
exit.place(x=745,y=720)
monitor = Button(frame1,font=80, height=2,width=6, text="Monitor", command=show_monitor)
monitor.place(x=850,y=720)
control = Button(frame2,font=80, height=2,width=6, text="Control", command=show_control)
control.place(x=745,y=700)

compressor_on=Button(frame1,font=80, height=2,width=6,bg="pale green",activebackground='green', text="Air Comp\n ON",command=air_comp_on)
compressor_on.place(x=20,y=100)
compressor_off=Button(frame1,font=80, height=2,width=6,bg= "coral", activebackground='red', text="Air Comp\n OFF",command=air_comp_off)
compressor_off.place(x=160,y=100)

lpt_lab = Label(frame1,image=lpt,width=200,height=250)
lpt_lab.place(x=10,y=280)

h2_on=Button(frame1,font=80,height=2,width=6,bg="pale green", activebackground='green', text="H2 ON",command=h2_on_cont)
h2_on.place(x=110,y=190)

h2_off=Button(frame1,font=80,height=2,width=6,bg= "coral",activebackground='red', text="H2 OFF",command=h2_off_cont)
h2_off.place(x=210,y=190)

comp1_butt=Button(frame1,image=comp1,width=250,height=200)
comp1_butt.place(x=220,y=330)

comp1_on=Button(frame1,font=80, height=2,width=6,bg="pale green",activebackground='green', text="Booster 1\n ON",command=comp1_on_cont)
comp1_on.place(x=245,y=280)
comp1_off=Button(frame1,font=80, height=2,width=6,bg="coral",activebackground='red',text="Booster 1\n OFF",command=comp1_off_cont)
comp1_off.place(x=345,y=280)

comp2_butt=Button(frame1,image=comp2,width=250,height=200)
comp2_butt.place(x=480,y=330)

comp2_on=Button(frame1,font=80, height=2,width=6,bg= "pale green", activebackground='green', text="Booster 2\n ON",command=comp2_on_cont)
comp2_on.place(x=505,y=280)
comp2_off=Button(frame1,font=80, height=2,width=6,bg= "coral", activebackground='red', text="Booster 2\n OFF",command=comp2_off_cont)
comp2_off.place(x=605,y=280)

comp3_butt=Button(frame1,image=comp3,width=250,height=200)
comp3_butt.place(x=740,y=330)

comp3_on=Button(frame1,font=80, height=2,width=6,bg= "pale green", activebackground='green', text="Booster 3\n ON",command=comp3_on_cont)
comp3_on.place(x=765,y=280)
comp3_off=Button(frame1,font=80, height=2,width=6,bg= "coral", activebackground='red', text="Booster 3\n OFF",command=comp3_off_cont)
comp3_off.place(x=865,y=280)

car_butt=Button(frame1,image=car,width=250,height=200)
car_butt.place(x=1000,y=330)

car_data=Label(frame1,text="Car Data Link",font=("Arial Bold",14))
car_data.place(x=1000,y=100)

car_data_on=Button(frame1,font=("Arial Bold",14),text="Connect",command=connect_serial)
car_data_on.place(x=1000,y=150)

car_data_off=Button(frame1,font=("Arial Bold",14),text="Disonnect",command=disconnect_serial)
car_data_off.place(x=1000,y=200)

car_data_status=Label(frame1,font=("Arial Bold",14),textvariable=car_data_status_txt)
car_data_status.place(x=1000,y=250)

car_tank_temp=Label(frame1,font=("Arial Bold",14),textvariable=car_tank_temp_txt)
car_tank_temp.place(x=1100,y=570)

Car_tank_pressure=Label(frame1,font=("Arial Bold",14),textvariable=car_tank_pressure_txt)
Car_tank_pressure.place(x=1100,y=620)

car_filling_status=Label(frame1,font=("Arial Bold",14),textvariable=car_filling_status_txt)
car_filling_status.place(x=1100,y=670)

data_frame_no=Label(frame1,font=("Arial Bold",14),textvariable=data_frame_no_txt)
data_frame_no.place(x=1100,y=720)


#temperature and pressure display lables
temp1_display=Label(frame1,font=("Arial Bold",14),text="Temp: "+str(T1)+u'\u00b0'+'C')
temp1_display.place(x=100,y=570)
P1_display=Label(frame1,font=("Arial Bold",14),text="Pressure: " +str(P1)+" Bar")
P1_display.place(x=100,y=620)

temp2_display=Label(frame1,font=("Arial Bold",14),text="Temp: "+str(T2)+u'\u00b0'+'C')
temp2_display.place(x=360,y=570)
P2_display=Label(frame1,font=("Arial Bold",14),text="Pressure: " +str(P2)+" Bar")
P2_display.place(x=360,y=620)

temp3_display=Label(frame1,font=("Arial Bold",14),text="Temp: "+str(T3)+u'\u00b0'+'C')
temp3_display.place(x=620,y=570)
P3_display=Label(frame1,font=("Arial Bold",14),text="Pressure: " +str(P3)+" Bar")
P3_display.place(x=620,y=620)

temp4_display=Label(frame1,font=("Arial Bold",14),text="Temp: "+str(T4)+u'\u00b0'+'C')
temp4_display.place(x=880,y=570)
P4_display=Label(frame1,font=("Arial Bold",14),text="Pressure: " +str(P4)+" Bar")
P4_display.place(x=880,y=620)


#CRC check function
def crc16(data : bytes, offset , length):
    global crc
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0
    crc = 0
    for i in range(0, length):
        crc ^= data[offset + i]
        for j in range(0,8):
            if (crc & 1) > 0:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc = crc >> 1
    return crc



#start the read serial to get car data in separate thread
def get_car_data():
    #print("get car data ")
    def car_data():
        global mp, mt,fc,SerialObj,p,start_car_data,sp,car_data_info,car_data_stat,fn
        #print("car data")
        with fn_lock:
            fn=0
        #fn_lock.release()
        car_update_time = time.time()
        #while True:
        #while SerialObj.in_waiting >=6:
        #while SerialObj.is_open:
        while sp_event.is_set():
            try:
                d=SerialObj.read(6) #read 6 bytes
                if d==b'\xFF\xFF\xFF\xFF\xFF\xC0':    #look for 5 x XBOF + BOF at start of frame
                    #print("Frame recieved")
                    ReceivedString = SerialObj.read_until(b'\xC1',87)    #read bytes until EOF
                    #test for crc
                    ReceivedCRC=ReceivedString[-3:-1]   #The received CRC check bytes are the 2 bytes before the last byte
                    ReceivedStringCRC = ReceivedString[:-3] #remove last 3 bytes from the string for CRC check
                    crcheck=crc16(ReceivedStringCRC, 0, int(len(ReceivedStringCRC))) #Calculate CRC check

                    # check to see if calculated CRC is the same as received CRC (both Hex strings)
                    if hex(crcheck)[2:]==(ReceivedCRC).hex():
                        with fn_lock:
                            fn= fn+1
                        #print("CRC OK")
                        crc_event.set()

                        try:
                            i=ReceivedString.find(b'|MP=')      #look for '|MP='
                            #mp=0.0                              #set value of mp local variable if not found
                            if ReceivedString[i+9:i+10]==b'|':# check that the MP data frame ends in '|'
                                with mp_lock:
                                    mp=10*float(str(ReceivedString[i+4:i+9],'utf-8'))      #make float of MP value
                                #print("MP OK")
                                car_update_time =time.time()
                                car_data_info_event.set()
                    
                            #mt=0.0
                            x=ReceivedString.find(b'|MT=')           #look for '|MT'     etc
                            if ReceivedString[x+9:x+10]==b'|':
                                with mt_lock:
                                    mt=float(str(ReceivedString[x+4:x+9],'utf-8'))
                                    mt=round(mt-273.2,1)
                                #print("MT OK")
                                car_update_time=time.time()
                    
                            #fc='Null'
                            f=ReceivedString.find(b'|FC=')
                            if ReceivedString[f+8:f+9]==b'|':
                                with fc_lock:
                                    fc=(str(ReceivedString[f+4:f+8],'utf-8'))
                                #print("FC OK")
                            elif ReceivedString[f+9:f+10]==b'|':        #Abort is 1 char longer than the other FC commands
                                fc=(str(ReceivedString[f+4:f+9],'utf-8'))
                                #print("FC ABORT")
                                with fc_lock:
                                    fc="ABORT"

                            else:
                                pass

                            tv=0.0
                            v=ReceivedString.find(b'|TV=')
                            if ReceivedString[v+10:v+11]==b'|':
                                tv=float(str(ReceivedString[v+4:v+10],'utf-8'))
                            else:
                                pass

                        except:
                    #    pass
                            print("Bad data")
                            #car_data_stat="Bad Data"
                    
                    else:
                        print("CRC error")
                        #car_data_stat="CRC Error"
                        crc_event.clear()
                        break
                #elif not car_data_info_event.is_set():
                   # break      
                #if no data for 3 seconds set car_data_info_event flag to false
                if time.time()- car_update_time >3:
                    car_data_info_event.clear()
                    crc_event.clear()

            except:
                pass
                # break

    if not sp_event.is_set():
        return      #stops separate thread if serial port flag is closed 
        
    
    thread = threading.Thread(target=car_data)
    thread.start()


#read ADC & calculate pressures
def get_pressures():
    global P1,P1_zero,P1_cal
    adc = ADCPi(0x68, 0x69,12)

    v1=adc.read_voltage(1)
   # print("V0lts = "+str(v1))
    P1 = (v1-P1_zero)*P1_cal
    P1 = round(P1,1)
    #if pressure < -5, sensor or loop is faulty
    if P1 < -5:
        P1_display.config(text="P Sensor FAULTY")
    else:
        P1_display.config(text="Pressure: " +str(P1)+" Bar") 
    root.after(1000,get_pressures)

#get temperatures from i2c bus
def get_temps():
    global T1,T2,T3,T4,T5
    #i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    mcp = adafruit_mcp9600.MCP9600(i2c,address=0x67)
    T1 = (mcp.temperature)
    T1=round(T1,1)
    temp1_display.config(text="Temp: "+str(T1)+u'\u00b0'+'C')
    mcp= adafruit_mcp9600.MCP9600(i2c,address=0x66)
    T2 = (mcp.temperature)
    T2=round(T2,1)
    temp2_display.config(text="Temp: "+str(T2)+u'\u00b0'+'C')
    mcp= adafruit_mcp9600.MCP9600(i2c,address=0x65)
    T3 = (mcp.temperature)
    T3=round(T3,1)
    temp3_display.config(text="Temp: "+str(T3)+u'\u00b0'+'C')
    mcp= adafruit_mcp9600.MCP9600(i2c,address=0x64)
    T4 = (mcp.temperature)
    T4=round(T4,1)
    temp4_display.config(text="Temp: "+str(T4)+u'\u00b0'+'C')
    root.after(1000,get_temps)  


def set_outputs():
    global opair,oph2,opc1,opc2,opc3,opvent,op5,op6,op7
    mcp = MCP23008(i2c, address=0x26)
    pin0 = mcp.get_pin(0)
    pin0.direction = Direction.OUTPUT
    pin1 = mcp.get_pin(1)
    pin1.direction = Direction.OUTPUT
    pin2 = mcp.get_pin(2)
    pin2.direction = Direction.OUTPUT
    pin3 = mcp.get_pin(3)
    pin3.direction = Direction.OUTPUT
    pin4 = mcp.get_pin(4)
    pin4.direction = Direction.OUTPUT
    pin5 = mcp.get_pin(5)
    pin5.direction = Direction.OUTPUT
    pin6 = mcp.get_pin(6)
    pin6.direction = Direction.OUTPUT
    pin7 = mcp.get_pin(7)
    pin7.direction = Direction.OUTPUT

    pin0.value = oph2
    pin1.value = opc1
    pin2.value = opc2
    pin3.value = opc3
    pin4.value = opvent
    pin5.value = op5
    pin6.value = op6
    pin7.value = op7

    #print("C1 = "+str(opc1))
    root.after(1000,set_outputs)

#update car data lables
def update_car_data():
    global mp,mt,fc,fn,start_car_data,car_data_info,car_tank_temp_txt,car_data_stat,car_tank_pressure_txt,car_filling_status_txt,car_update_time,car_data_stat
    
    #test if get car data is running
    #if time.time()-car_update_time >3:
    if not car_data_info_event.is_set():
        car_data_info=False
        car_tank_temp_txt.set("Temp: "+"? "+u'\u00b0'+'C')
        car_tank_pressure_txt.set("Pressure: "+"? "+" Bar")
        car_filling_status_txt.set("Stopped")
        if car_data_stat != "waiting" and sp_event.is_set():
            car_data_stat="waiting"
        elif not sp_event.is_set(): #if serial port is closed, set status to disconnected
            car_data_stat="Disconnected"
        car_data_status_txt.set(car_data_stat)

    else:
        car_data_info=True
        with mt_lock:    #use locks to prevent data being changed while label text is being set 
            car_tank_temp_txt.set("Temp: "+str(mt)+u'\u00b0'+'C')
        
        with mp_lock:
            car_tank_pressure_txt.set("Pressure: "+str(mp)+" Bar")
        
        with fc_lock:
            car_filling_status_txt.set(fc)
        
        if crc_event.is_set():
            car_data_stat = ("CRC OK")
        car_data_status_txt.set(car_data_stat)
        with fn_lock:    #fn is the number of OK frames received
            data_frame_no_txt.set(str(fn))
        
    root.after(1000,update_car_data)


#set all buttons to off on start up
h2_off_cont()
comp1_off_cont()
comp2_off_cont()
comp3_off_cont()

air_comp_off()
root.after(0,get_temps)
root.after(100,get_pressures)
root.after(200,update_car_data)
root.after(300,set_outputs)

root.mainloop() #start the GUI