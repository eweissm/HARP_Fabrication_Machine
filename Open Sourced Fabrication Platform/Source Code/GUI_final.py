
"""
================================================================================
HARP Machine Controller GUI
--------------------------------------------------------------------------------
Author: Eric Weissman
Created: 6/16/2025
Description:
    This script provides a graphical user interface (GUI) for controlling a
    PTCA (Percutaneous Transluminal Coronary Angioplasty) machine using Python
    and serial communication with an Arduino-based hardware system.

    Users can input parameters for tube forming (twist angle, tube length,
    diameters, FR pitch, etc.) and send commands such as:
        - Home
        - Cold Draw
        - Fiber Reinforce
        - Twist
        - Coil
        - Jog motions
        - Spin
        - Stop

    The GUI supports live COM port selection and communicates with the machine
    via formatted serial messages.

Dependencies:
    - Python 3.x
    - tkinter (standard GUI library)
    - pyserial (install via pip: `pip install pyserial`)

Usage:
    1. Run this script.
    2. Select the COM port and connect.
    3. Enter your desired tube and muscle properties.
    4. Click a control button to send a command to the hardware.

Communication Protocol:
    - Messages sent over serial are formatted as:
        <Command>,<Twist>,<Length>,<Tube Ø>,<Draw Ratio>,<FR Pitch>,<Coil Pitch>,<Mandrel Ø>Z
    - The Arduino listens for these messages and executes the corresponding action.

================================================================================
"""


import tkinter as tk       # GUI library
import serial              # Serial communication library
import time                # Timing utility

# Global variable declarations
global ser
global ConnectionStateMessage
global arduinoQueue
global localQueue
global ConnectionState
global SerialReq

################################################################################################
# Utility Functions
################################################################################################

def ReadInputs():
    """Reads values from the GUI entry widgets and returns them as a tuple."""
    twistAngle = TwistAngle_entry.get()
    tubeLength = TubeLength_entry.get()
    tubeDiameter = TubeDiameter_entry.get()
    coldDrawRatio = coldDrawRatio_entry.get()
    FRAngle = FRAngle_entry.get()
    coilAngle = CoilAngle_entry.get()
    MandrelDiameter = MandrelDiameter_entry.get()
    return (twistAngle, tubeLength, tubeDiameter, coldDrawRatio, FRAngle, coilAngle, MandrelDiameter)

def packAndSendMsg(Command):
    """Constructs a command string from the GUI inputs and sends it over the serial port."""
    global ser
    print(Command)
    Parameters = ReadInputs()
    msg = Command  # start with command character

    for i in Parameters:
        msg = msg + ',' + i  # append each parameter

    msg = msg + 'Z'  # terminate message with 'Z'

    ser.write(bytes(str(msg), 'UTF-8'))  # send message

# Placeholder for serial listener using queues (currently commented out)
# This could be useful for receiving data from Arduino asynchronously
# import queue
# arduinoQueue = queue.Queue()
# localQueue = queue.Queue()

ConnectionState = False  # Initialize connection flag

def ConnectSerial(PortString):
    """Attempts to connect to the Arduino on the specified COM port."""
    global ser
    global ConnectionStateMessage
    global ConnectionState

    ConnectionStateMessage.set("...")  # show connecting message
    try:
        ser = serial.Serial(port=PortString, baudrate=9600, timeout=10)
        time.sleep(0.1)  # slight delay to stabilize connection
        ConnectionStateMessage.set("Connected")
        ConnectionState = True
    except:
        ConnectionStateMessage.set("Connection Failed")
        ConnectionState = False

def runStartUp():
    """Launches a startup window to prompt the user for a COM port."""
    global ConnectionState
    global ConnectionStateMessage

    # Create main window
    tkTop = tk.Tk()
    tkTop.geometry('500x300')
    tkTop.title("🔌 HARP Machine Startup")

    tkTop.configure(bg="#f0f0f0")  # Light background

    # Connection status text
    ConnectionStateMessage = tk.StringVar()
    ConnectionStateMessage.set("🔴 Not Connected")

    # Title label
    tk.Label(
        tkTop, text="Select a COM Port to Connect", font=("Courier", 16, 'bold'),
        bg="#f0f0f0", fg="#333"
    ).pack(pady=(20, 10))

    # Frame for input
    input_frame = tk.Frame(tkTop, bg="#f0f0f0")
    input_frame.pack(pady=10)

    tk.Label(
        input_frame, text="COM Port:", font=("Courier", 12),
        bg="#f0f0f0"
    ).grid(row=0, column=0, sticky='e', padx=(0, 10))

    ComPortInput_entry = tk.Entry(input_frame, font=("Courier", 12), width=15)
    ComPortInput_entry.insert(0, 'com5')
    ComPortInput_entry.grid(row=0, column=1)

    tk.Label(
        input_frame, text="e.g. COM3, COM4", font=("Courier", 10),
        bg="#f0f0f0", fg="gray"
    ).grid(row=1, column=0, columnspan=2, pady=(5, 0))

    # Frame for buttons and status
    button_frame = tk.Frame(tkTop, bg="#f0f0f0")
    button_frame.pack(pady=20)

    ConnectButton = tk.Button(
        button_frame, text="🔗 Connect",
        command=lambda: ConnectSerial(ComPortInput_entry.get()),
        height=2, width=12, font=("Courier", 11, 'bold'),
        bg="#d4ffd4", activebackground="#b2ffb2", bd=3
    )
    ConnectButton.grid(row=0, column=0, padx=10)

    ExitStartup_Button = tk.Button(
        button_frame, text="✓ Done",
        command=tkTop.destroy,
        height=2, width=12, font=("Courier", 11, 'bold'),
        bg="#d4e0ff", activebackground="#b2cfff", bd=3
    )
    ExitStartup_Button.grid(row=0, column=1, padx=10)

    # Connection status indicator
    tk.Label(
        tkTop, textvariable=ConnectionStateMessage,
        font=("Courier", 12, 'bold'), fg="#444", bg="#f0f0f0"
    ).pack(pady=(0, 10))

    # Serial disable checkbox
    SerialReq = tk.IntVar()
    tk.Checkbutton(
        tkTop, text="Disable Serial Requirement",
        variable=SerialReq, onvalue=True, offvalue=False,
        font=("Courier", 10), bg="#f0f0f0"
    ).pack()

    tk.mainloop()
    return ConnectionState, SerialReq.get()


################################################################################################
# Establish Serial Connection
################################################################################################

ComsState = False
SerialReq = 0

# Loop until either connection succeeds or user disables requirement
while not ComsState and SerialReq == 0:
    ComsState, SerialReq = runStartUp()


tkTop = tk.Tk()
tkTop.geometry('800x700')
tkTop.title("🧠 HARP-Machine Controller")
tkTop.configure(bg="#f9f9f9")

# Title
tk.Label(
    tkTop, text='HARP Machine Controller',
    font=("Courier", 16, 'bold'),
    bg="#f9f9f9", fg="#222"
).pack(pady=10)

##################################################################################
# Tube Properties Section
##################################################################################
TubePropertiesFrame = tk.LabelFrame(
    master=tkTop, text="Tube Properties",
    font=("Courier", 12, 'bold'), bg="#f0f0f0", padx=10, pady=10
)
TubePropertiesFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True, padx=15, pady=10)

tube_params = [
    ("Tube Length (mm)", "Initial tube length before cold draw", '220', 'TubeLength_entry'),
    ("Tube Diameter (mm)", "Outer diameter of the drawn tube", '2.5', 'TubeDiameter_entry'),
    ("Mandrel Diameter (mm)", "Diameter of the coiling mandrel", '3.0', 'MandrelDiameter_entry')
]

for label, note, default, var in tube_params:
    frame = tk.Frame(master=TubePropertiesFrame, bg="#f0f0f0")
    frame.pack(fill=tk.X, pady=3)
    tk.Label(frame, text=label + ":", font=("Courier", 12), width=20, anchor='w', bg="#f0f0f0").pack(side='left')
    globals()[var] = tk.Entry(frame, font=("Courier", 12), width=10)
    globals()[var].insert(0, default)
    globals()[var].pack(side='left', padx=10)
    tk.Label(frame, text=f"({note})", font=("Courier", 9), fg="gray", bg="#f0f0f0").pack(side='left')

##################################################################################
# Muscle Properties Section
##################################################################################
MusclePropertiesFrame = tk.LabelFrame(
    master=tkTop, text="Muscle Properties",
    font=("Courier", 12, 'bold'), bg="#f0f0f0", padx=10, pady=10
)
MusclePropertiesFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True, padx=15, pady=10)

muscle_params = [
    ("Twist Angle (Deg)", "Helical twist applied to the drawn tube ", '5', 'TwistAngle_entry'),
    ("FR Pitch (mm)", "Axial distance between fiber wraps", '12', 'FRAngle_entry'),
    ("Cold Draw Ratio", "Length ratio of drawn to raw tube", '2.5', 'coldDrawRatio_entry'),
    ("Coil Pitch (mm)", "Spacing of muscle coils on mandrel", '6', 'CoilAngle_entry')
]

for label, note, default, var in muscle_params:
    frame = tk.Frame(master=MusclePropertiesFrame, bg="#f0f0f0")
    frame.pack(fill=tk.X, pady=3)
    tk.Label(frame, text=label + ":", font=("Courier", 12), width=20, anchor='w', bg="#f0f0f0").pack(side='left')
    globals()[var] = tk.Entry(frame, font=("Courier", 12), width=10)
    globals()[var].insert(0, default)
    globals()[var].pack(side='left', padx=10)
    tk.Label(frame, text=f"({note})", font=("Courier", 9), fg="gray", bg="#f0f0f0").pack(side='left')

##################################################################################
# Control Buttons Section
##################################################################################
ControlsFrame = tk.LabelFrame(
    master=tkTop, text="Controls",
    font=("Courier", 12, 'bold'), bg="#f9f9f9", padx=10, pady=10
)
ControlsFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True, padx=15, pady=10)

# Define command buttons with notes
control_buttons = [
    ("Home", "", 'A'),
    ("Cold Draw", "", 'B'),
    ("Fiber Reinforce", "", 'C'),
    ("Twist", "", 'D'),
    ("Coil", "", 'E')
]

jog_buttons = [
    ("Follower Up", "", 'G'),
    ("Follower Down", "", 'H'),
    ("Runner Up", "", 'I'),
    ("Runner Down", "", 'J'),
    ("Spin", "", 'K')
]

def make_control_button(master, label, note, cmd, color='green'):
    btn = tk.Button(
        master, text=label, command=lambda: packAndSendMsg(cmd),
        height=2, width=14, font=("Courier", 10),
        bg=f"#ccffcc" if color == 'green' else "#ffcccc",
        activebackground=f"#aaffaa" if color == 'green' else "#ff9999",
        bd=3
    )
    btn.pack(side='left', padx=5, pady=5)
    tk.Label(master, text=note, font=("Courier", 8), fg="gray", bg="#f9f9f9").pack(side='left', padx=5)

# Top row: main process buttons
TopRow = tk.Frame(master=ControlsFrame, bg="#f9f9f9"); TopRow.pack(pady=5)
for label, note, cmd in control_buttons:
    make_control_button(TopRow, label, note, cmd)

# Middle row: jogging buttons
MiddleRow = tk.Frame(master=ControlsFrame, bg="#f9f9f9"); MiddleRow.pack(pady=5)
for label, note, cmd in jog_buttons:
    make_control_button(MiddleRow, label, note, cmd)

# Bottom row: stop button
BottomRow = tk.Frame(master=ControlsFrame, bg="#f9f9f9"); BottomRow.pack(pady=10)
tk.Button(
    BottomRow, text="STOP", command=lambda: packAndSendMsg('F'),
    height=2, width=16, font=("Courier", 12, 'bold'),
    bg="#ff9999", activebackground="#ff6666", bd=4
).pack()
tk.Label(BottomRow, text="(Immediately halts all motors)",
         font=("Courier", 9), fg="gray", bg="#f9f9f9").pack()

tk.mainloop()
