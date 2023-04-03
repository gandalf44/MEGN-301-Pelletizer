import serial as serial

# Variable Bank:
debug = True  # Value to activate debug commands.
log = False  # Variable to activate user log.
myBaud = 9600  # Sets the universal baud rate for the program.


def debug_print(message):
    """
    :param message: message to be printed
    This method prints a message only if the debug setting is turned on
    """
    if debug:
        print(message)
    if log:
        with open("package_OS_log.txt", "a") as logfile:
            logfile.write(message)


ser = serial.Serial('COM8', myBaud, timeout=1)
com_port = 'COM8'
ser.reset_input_buffer()  # Resets the input buffer
debug_print(com_port)
