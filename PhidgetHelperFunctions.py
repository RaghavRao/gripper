import sys
from Phidget22.PhidgetException import *
from Phidget22.ErrorCode import *
from Phidget22.Phidget import *
from Phidget22.Net import *

class InputError(Exception):
    """Exception raised for errors in the input.

    Attributes:
        msg  -- explanation of the error
    """

    def __init__(self, msg):
        self.msg = msg

# Returns None if an error occurred, True for 'Y' and False for 'N'  
def ProcessYesNo_Input(default): #N
    return False

def DisplayError(e):
    sys.stderr.write("Desc: " + e.details + "\n")

def SetSerialNumber(ph):
    try:
        ph.setDeviceSerialNumber(261542)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Setting DeviceSerialNumber: \n\t")
        DisplayError(e)
        raise

    return


def SetIsHubPortDevice(ph):
    isHubPortDevice = -1

    while (True):
        print("\nIs this a \"HubPortDevice\"? [y/n] ")
        try:
            isHubPortDevice = ProcessYesNo_Input(-1)
            break
        except InputError as e:
            pass

    try:
        ph.setIsHubPortDevice(isHubPortDevice)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Setting isHubPortDevice: \n\t")
        DisplayError(e)
        raise

    return

def SetVINTProperties(ph):
    canBeHubPortDevice = 0
    pcc = -1
    hubPort = -1
    isVINT = 0

    print("\n--------------------------------------")

    while (True):
        print("\nDo you want to specify the hub port that your device is plugged into?\n"
            "Choose No if your device is not plugged into a VINT Hub. (y/n) ")
        try:
            isVINT = ProcessYesNo_Input(-1)
            break
        except InputError as e:
            pass

    # Don't ask about the HubPort and the HubPortDevice if it's not a VINT device
    if (not isVINT):
        return

    print("\n--------------------------------------")
    print("\n  | VINT Hubs have numbered ports that can be uniquely addressed.\n"
        "  | The HubPort# is identified by the number above the port it is plugged into.\n"
        "  | Specify the hub port to ensure you are only opening channels from that specific port.\n"
        "  | Otherwise, use -1 to open a channel on any port.")
    while (True):
        print("\nWhat HubPort is the device plugged into? [-1] ")
        strvar = sys.stdin.readline(100)
        if not strvar:
            continue

        strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (strvar[0] == '\n'):
            hubPort = -1
            break

        try:
            hubPort = int(strvar)
        except ValueError as e:
            continue

        if (hubPort >= -1 and hubPort <= 5):
            break

    try:
        ph.setHubPort(hubPort)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Setting HubPort: \n\t")
        DisplayError(e)
        raise

    try:
        pcc = ph.getChannelClass()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Getting ChannelClass: \n\t")
        DisplayError(e)
        raise

    if (pcc == ChannelClass.PHIDCHCLASS_VOLTAGEINPUT):
        print("\n--------------------------------------")
        print("\n  | A VoltageInput HubPortDevice uses the VINT Hub's internal channel to measure the voltage on the white wire.\n"
          "  | If the device you are trying to interface returns an analog voltage between 0V-5V, open it as a HubPortDevice.")
        canBeHubPortDevice = 1
    elif (pcc == ChannelClass.PHIDCHCLASS_VOLTAGERATIOINPUT):
        print("\n--------------------------------------")
        print("\n  | A VoltageRatioInput HubPortDevice uses the VINT Hub's internal channel to measure the voltage ratio on the white wire.\n"
          "  | If the device you are trying to interface returns an ratiometric voltage between 0V-5V, open it as a HubPortDevice.")
        canBeHubPortDevice = 1
    elif (pcc == ChannelClass.PHIDCHCLASS_DIGITALINPUT):
        print("\n--------------------------------------")
        print("\n  | A DigitalInput HubPortDevice uses the VINT Hub's internal channel to detect digital changes on the white wire.\n"
          "  | If the device you are trying to interface outputs a 5V digital signal, open it as a HubPortDevice.")
        canBeHubPortDevice = 1
    elif (pcc == ChannelClass.PHIDCHCLASS_DIGITALOUTPUT):
        print("\n--------------------------------------")
        print("\n  | A DigitalOutput HubPortDevice uses the VINT Hub's internal channel to output a 3.3V digital signal on the white wire.\n"
          "  | If the device you are trying to interface accepts a 3.3V digital signal, open it as a HubPortDevice.")
        canBeHubPortDevice = 1

    if (canBeHubPortDevice):
        SetIsHubPortDevice(ph)

    return

def SetChannel(ph):
    isHubPortDevice = 0
    Channel = 0

    try:
        isHubPortDevice = ph.getIsHubPortDevice()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Getting isHubPortDevice: \n\t")
        DisplayError(e)
        raise

    # Hub port devices only have a single channel, so don't ask for the channel
    if (isHubPortDevice):
        return

    print("using Channel 0")
    ph.setChannel(0)
    return

def EnableServerDiscovery():
    print("Enabling Server Discovery...")
    try:
        Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Enable Server Discovery: \n\t")
        DisplayError(e)
        raise
    return

def SetupNetwork(ph):
    hostname = ""
    password = ""
    discovery = 0
    isRemote = 0
    port = 0

    print("\n--------------------------------------")
    print("\n  | Devices can either be opened directly, or over the network.\n"
      "  | In order to open over the network, the target system must be running a Phidget Server.")
    while (True):
        print("\nIs this device being opened over the network? [y/N] ")
        try:
            isRemote = ProcessYesNo_Input(0)
            break
        except InputError as e:
            pass

    try:
        ph.setIsRemote(isRemote)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Set IsRemote: \n\t")
        DisplayError(e)
        raise

    # if it's not remote, don't need to ask about the network
    if (not isRemote):
        return

    print("\n--------------------------------------")
    print("\n  | Server discovery enables the dynamic discovery of phidget servers that publish their identity to the network.\n"
          "  | This allows you to open devices over the network without specifying the hostname and port of the server.")
    while (True):
        print("\nDo you want to enable server discovery? [Y/n] ");
        try:
            discovery = ProcessYesNo_Input(1)
            break
        except InputError as e:
            pass

    if (discovery):
        return EnableServerDiscovery()

    print("\n--------------------------------------")
    print("\nPlease provide the following information in order to open the device")

    while (True):
        print("\nWhat is the Hostname (or IP Address) of the server? [localhost] ")
        hostname = sys.stdin.readline(100)
        if not hostname:
            continue

        hostname = hostname.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (hostname[0] == '\n'):
            hostname = "localhost"
            break

        # Remove trailing newline
        hostname = hostname.split('\n')[0]
        break

    print("\n--------------------------------------")
    while (True):
        print("\nWhat port is the server on? [5661] ")
        strvar = sys.stdin.readline(100)
        if not strvar:
            continue

        strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (strvar[0] == '\n'):
            port = 5661
            break

        try:
            port = int(strvar)
        except ValueError as e:
            continue

        if (port <= 65535 and port > 0):
            break
    
    print("\n--------------------------------------")
    while (True):
        print("\nWhat is the password of the server? [] ")
        password = sys.stdin.readline(100)
        if not password:
            continue
        # Remove trailing newline
        password = password.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older

        password = password.split('\n')[0]
        break
    
    print("\n--------------------------------------")

    print("Adding Server...")
    try:
        Net.addServer("Server", hostname, port, password, 0)
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> AddServer: \n\t")
        DisplayError(e)
        raise

    return

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
