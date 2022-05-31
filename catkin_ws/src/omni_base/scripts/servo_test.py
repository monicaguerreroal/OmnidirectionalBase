#!/usr/bin/env python
import rospy
from dynamixel_sdk import*
from std_msgs.msg import Float32, Float32MultiArray
import math
MAX_SPEED = 470 #MAX SPEED INT RPM

#Angular speed int rad/s

def callback_servo3(msg):

    #Angular speed int rad/s
    global portHandler, packetHandler
    speed =int(470/(MAX_SPEED*2*math.pi/60)*msg.data)
    print ("received:"+str(msg.data))
    dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,3, 32, speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


def callback_servo2(msg):

    #Angular speed int rad/s
    global portHandler, packetHandler
    speed =int(470/(MAX_SPEED*2*math.pi/60)*msg.data)
    print ("received:"+str(msg.data))
    dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,2, 32, speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        

def callback_servo1(msg):

    #Angular speed int rad/s
    global portHandler, packetHandler
    speed =int(470/(MAX_SPEED*2*math.pi/60)*msg.data)
    print ("received:"+str(msg.data))
    dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,1, 32, speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def callback_speeds(msg):
    speeds = []
    for s in msg.data:
        if s<0:
            speeds.append(int(470/(MAX_SPEED*2*math.pi/60)*(-s))+1024)
        else:
            speeds.append(int(470/(MAX_SPEED*2*math.pi/60)*(s)))
            
    for i in range (1,4):
        dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,i, 32, speeds[i-1])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    
        
def main():

    global portHandler, packetHandler
    rospy.init_node('omni_base', anonymous=True)
    rospy.Subscriber("/servo1_speed", Float32, callback_servo1)
    rospy.Subscriber("/servo2_speed", Float32, callback_servo2)
    rospy.Subscriber("/servo3_speed", Float32, callback_servo3)
    rospy.Subscriber("/servo_speeds", Float32MultiArray, callback_speeds)
    
    DXL_ID                  = 1                 # Dynamixel ID : 1
    BAUDRATE                = 1000000           # Dynamixel default baudrate : 57600
    DEVICENAME              = '/dev/ttyUSB0'    # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(1.0)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
    

    # Try to ping the Dynamixel
    # Get Dynamixel model number
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
    if dxl_comm_result != COMM_SUCCESS:
        print("result: %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("error: %s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number))
        
    """
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler,1,8)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, dxl_present_position))
    """ 
    
    # Close port
    
    rospy.spin()
    dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,1, 32, 0)
    dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,2, 32, 0)
    dxl_comm_result,dxl_error=packetHandler.write2ByteTxRx(portHandler,3, 32, 0)
    portHandler.closePort()

if __name__ == "__main__":
    main()
