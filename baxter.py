#!/usr/bin/python
import sys
import time 
import rospy
from Phidget22.Devices.VoltageInput import *
from Phidget22.PhidgetException import *
from Phidget22.ErrorCode import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from std_msgs.msg import String

def DisplayError(e):
    sys.stderr.write("Desc: " + e.details + "\n")

def OpenPhidgetChannel(ph):
    print("Opening Channel...")
    try:
        ph.open()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Opening Phidget Channel: \n\t")
        DisplayError(e)
        raise

    return

def OpenPhidgetChannel_waitForAttach(ph, timeout_in_ms):
    print("Opening Channel...")
    try:
        ph.openWaitForAttachment(timeout_in_ms)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Opening Phidget Channel: \n\t")
        DisplayError(e)
        if(e.code == ErrorCode.EPHIDGET_TIMEOUT):
            sys.stderr.write("\nThis error commonly occurs if your device is not connected as specified, "
                             "or if another program is using the device, such as the Phidget Control Panel.\n")
            if(     ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGEINPUT
                and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGERATIOINPUT
                and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALINPUT
                and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALOUTPUT
            ):
                sys.stderr.write("\nIf you are trying to connect to an analog sensor, you will need to use the "
                                 "corresponding VoltageInput or VoltageRatioInput API with the appropriate SensorType.\n")
        raise

    return

def ClosePhidgetChannel(ph):
    print("Closing Channel...")
    try:
        ph.close()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Closing Phidget Channel: \n\t")
        DisplayError(e)
        raise

    return

"""
* Configures the device's DataInterval and ChangeTrigger.
* Displays info about the attached phidget channel.
* Fired when a Phidget channel with onAttachHandler registered attaches
*
* @param self The Phidget channel that fired the attach event
"""
def onAttachHandler(self):
    
    ph = self

    """
    * Set the DataInterval inside of the attach handler to initialize the device with this value.
    * DataInterval defines the minimum time between VoltageChange events.
    * DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
    """
    print("\tSetting DataInterval to 1000ms")
    try:
        ph.setDataInterval(100)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Setting DataInterval: \n\t")
        DisplayError(e)
        return

    """
    * Set the VoltageChangeTrigger inside of the attach handler to initialize the device with this value.
    * VoltageChangeTrigger will affect the frequency of VoltageChange events, by limiting them to only occur when
    * the voltage changes by at least the value set.
    """
    print("\tSetting Voltage ChangeTrigger to 0.0")
    try:
        ph.setVoltageChangeTrigger(0.0)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Set VoltageChangeTrigger: \n\t")
        DisplayError(e)
        return

    """
    * Set the SensorType inside of the attach handler to initialize the device with this value.
    * SensorType will apply the appropriate calculations to the voltage reported by the device
    * to convert it to the sensor's units.
    * SensorType can only be set for Sensor Port voltage inputs (VINT Ports and Analog Input Ports)
    """
    if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT):
        print("\tSetting Voltage SensorType")
        try:
            ph.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Set SensorType: \n\t")
            DisplayError(e)
            return
        
    try:
        serialNumber = ph.getDeviceSerialNumber()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Get DeviceSerialNumber: \n\t")
        DisplayError(e)
        return

    try:
        channel = ph.getChannel()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Get Channel: \n\t")
        DisplayError(e)
        return

    # Check if this is a VINT device
    try:
        hub = ph.getHub()
    except PhidgetException as e:
        if(e.code == ErrorCode.EPHIDGET_WRONGDEVICE):
            print("\nAttach Event:\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel " + str(channel) + "\n")
        return
    try:
        hubPort = ph.getHubPort()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Get HubPort: \n\t")
        DisplayError(e)
        return

    print("\nAttach Event:\n\t-> Serial Number: " + str(serialNumber) +
          "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel " + str(channel) + "\n")
    return

"""
* Displays info about the detached phidget channel.
* Fired when a Phidget channel with onDetachHandler registered detaches
*
* @param self The Phidget channel that fired the attach event
"""
def onDetachHandler(self):

    ph = self

    try:
        serialNumber = ph.getDeviceSerialNumber()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Get DeviceSerialNumber: \n\t")
        DisplayError(e)
        return

    try:
        channel = ph.getChannel()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Get Channel: \n\t")
        DisplayError(e)
        return

     # Check if this is a VINT device
    try:
        hub = ph.getHub()
    except PhidgetException as e:
        if(e.code == ErrorCode.EPHIDGET_WRONGDEVICE):
            print("\nDetach Event:\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel " + str(channel) + "\n")
        return

    try:
        hubPort = ph.getHubPort()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Get HubPort: \n\t")
        DisplayError(e)
        return

    print("\nDetach Event:\n\t-> Serial Number: " + str(serialNumber) +
          "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel " + str(channel) + "\n")
    return

"""
* Writes phidget error info to stderr.
* Fired when a Phidget channel with onErrorHandler registered encounters an error in the library
*
* @param self The Phidget channel that fired the attach event
* @param errorCode the code associated with the error of enum type ph.ErrorEventCode
* @param errorString string containing the description of the error fired
"""
def onErrorHandler(self, errorCode, errorString):

    sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")

"""
* Sets the event handlers for Phidget Attach, Phidget Detach, Phidget Error events
*
* @param ph The Phidget channel to add event handlers to
* @return if the operation succeeds
* @raise PhidgetException if it fails
"""
def SetAttachDetachError_Handlers(ph):
    print("\n--------------------------------------")
    print("\nSetting OnAttachHandler...")
    try:
        ph.setOnAttachHandler(onAttachHandler)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Set Attach Handler: \n\t")
        DisplayError(e)
        raise

    print("Setting OnDetachHandler...")
    try:
        ph.setOnDetachHandler(onDetachHandler)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Set Detach Handler: \n\t")
        DisplayError(e)
        raise

    print("Setting OnErrorHandler...")
    try:
        ph.setOnErrorHandler(onErrorHandler)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Set Error Handler: \n\t")
        DisplayError(e)
        raise

    return


def talker(voltage, publisher):
    global volts
    volts =  str(voltage)
    hello_str = volts
#    rospy.loginfo(hello_str)
    publisher.publish(hello_str)

"""
* Outputs the VoltageInput's most recently reported voltage.
* Fired when a VoltageInput channel with onVoltageChangeHandler registered meets DataInterval and ChangeTrigger criteria
*
* @param self The VoltageInput channel that fired the VoltageChange event
* @param voltage The reported voltage from the VoltageInput channel
"""
def onVoltageChangeHandler(self, voltage):

    global volts
    volts = str(voltage)
    print("[Voltage Event] -> Voltage: " + volts)
    
"""
* Outputs the VoltageInput's most recently reported sensor value.
* Fired when a VoltageInput channel with onSensorChangeHandler registered meets DataInterval and ChangeTrigger criteria
*
* @param self The VoltageInput channel that fired the SensorChange event
* @param sensorValue The reported sensor value from the VoltageInput channel
"""
def onSensorChangeHandler(self, sensorValue, sensorUnit):

    print("[Sensor Event] -> Sensor Value: " + str(sensorValue) + sensorUnit.symbol)
    
"""
* Creates a new instance of a VoltageInput channel.
*
* @param pvih Pointer to the PhidgetVoltageInputHandle channel to create
* @return if the operation succeeds
* @raise PhidgetException if it fails
"""
def CreateVoltageInput(pvih):

    print("Creating VoltageInput Channel...")
    try:
        pvih.create()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Creating VoltageInput: \n\t")
        DisplayError(e)
        raise

    return

"""
* Sets the event handler for VoltageInput's VoltageChange event
*
* @param pvih The PhidgetVoltageInputHandle channel to add the event to
* @param fptr The callback function to be called when a VoltageChange event is fired
* @return if the operation succeeds
* @raise PhidgetException if it fails
"""
def SetVoltageHandler(pvih, fptr):

    if (not (fptr is None)):
        print("\n--------------------\n"
            "\n  | Voltage change events contain the most recent voltage received from the device.\n"
            "  | The linked VoltageChange function will run as an event at every DataInterval.\n"
            "  | These events will not occur until a change in voltage >= to the set ChangeTrigger has occurred.\n"
            "  | DataInterval and ChangeTrigger should initially be set in the device AttachHandler function.")
    
    print("")
    if (fptr is None):
        print("Clearing OnVoltageChangeHandler...")
    else:
        print("Setting OnVoltageChangeHandler...")

    print("\n--------------------")
    try:
        pvih.setOnVoltageChangeHandler(fptr)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Setting VoltageChangeHandler: \n\t")
        DisplayError(e)
        raise
    
    return
    
"""
* Sets the event handler for VoltageInput's SensorChange event
*
* @param pvih The PhidgetVoltageInputHandle channel to add the event to
* @param fptr The callback function to be called when a SensorChange event is fired
* @return if the operation succeeds
* @raise PhidgetException if it fails
"""
def SetSensorHandler(pvih, fptr):

    if (not (fptr is None)):
        print("\n--------------------\n"
            "\n  | Sensor change events contain the most recent sensor value received from the device.\n"
            "  | Sensor change events will occur instead of Voltage change events if the SensorType is changed from the default."
            "  | The linked SensorChange function will run as an event at every DataInterval.\n"
            "  | These events will not occur until a change in sensor value >= to the set ChangeTrigger has occurred.\n"
            "  | DataInterval, ChangeTrigger and SensorType should initially be set in the device AttachHandler function.")
    
    print("")
    if (fptr is None):
        print("Clearing OnSensorChangeHandler...")
    else:
        print("Setting OnSensorChangeHandler...")

    print("\n--------------------")
    try:
        pvih.setOnSensorChangeHandler(fptr)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Setting SensorChangeHandler: \n\t")
        DisplayError(e)
        raise
    return

"""
* Creates, configures, and opens a VoltageInput channel.
* Displays Voltage events for 10 seconds
* Closes out VoltageInput channel
*
* @return 0 if the program exits successfully, 1 if it exits with errors.
"""
def main():
    try:
        """
        * Allocate a new Phidget Channel object
        """
        try:
            ch = VoltageInput()
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Creating VoltageInput: \n\t")
            DisplayError(e)
            raise
        except RuntimeError as e:
            sys.stderr.write("Runtime Error -> Creating VoltageInput: \n\t" + e)
            raise

        """
        * Set matching parameters to specify which channel to open
        """
        ch.setDeviceSerialNumber(261542)
        ch.setChannel(0)
        """
        * Add event handlers before calling open so that no events are missed.
        """
        SetAttachDetachError_Handlers(ch)
        
        SetVoltageHandler(ch, onVoltageChangeHandler)
        
        SetSensorHandler(ch, onSensorChangeHandler)
        
        """
        * Open the channel with a timeout
        """
        OpenPhidgetChannel_waitForAttach(ch, 5000)
        
        print("Sampling data for 10 seconds...")
        pub = rospy.Publisher('/baxter/phidget/voltage', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            talker(volts, publisher=pub)
            rate.sleep()

        """
         * Perform clean up and exit
         """

        SetVoltageHandler(ch, None)
        
        print("\nDone Sampling...")

        print("Cleaning up...")
        ClosePhidgetChannel(ch)
        print("\nExiting...")
        print("Press ENTER to end program.")
        readin = sys.stdin.readline(1)
        return 0

    except PhidgetException as e:
        sys.stderr.write("\nExiting with error(s)...")
        DisplayError(e)
        print("Press ENTER to end program.")
        readin = sys.stdin.readline(1)
        return 1

main()
