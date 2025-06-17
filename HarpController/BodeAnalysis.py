import os
from serial_communicator import SerialCommunicator
import time

output_file= "Data_final/BodeAnalysis_Muscle1_short_30psi_120INPUT_v1_30g.csv"

if os.path.exists(output_file):
    raise FileExistsError(f"Error: The file '{output_file}' already exists. Choose a different name.")


communicator = SerialCommunicator(port="COM4",
                                  baudrate=115200,
                                  n_floats_to_arduino=6,
                                  n_floats_from_arduino=6,
                                  verbose=False,
                                  buffer_size=1000,
                                  direction = 'oneWayFromArduino',
                                  logFreq=True,
                                  CSVPath = output_file)
communicator.start()

while True:
    buffer = communicator.get_buffer()
    # try:
    if buffer:
        # if buffer[-1]> 350 or buffer[-1]<50:
        #     print("Error:check laser limits")
        print(buffer[-1][2])
    # except Exception:
    #     pass

    time.sleep(1)