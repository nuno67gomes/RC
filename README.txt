INSTRUCTIONS FOR SERIAL PORT PROTOCOL
=====================================

This folder contains the base code of the serial port protocol.

Project Structure
-----------------

- bin/: Compiled binaries.
- src/: Source code for the implementation of the link-layer and application layer protocols. Students should edit these files to implement the project.
- cable/: Virtual cable program to help test the serial port. This file must not be changed.
- Makefile: Makefile to build the project and run the application.
- penguin.gif: Example file to be sent through the serial port.

Instructions to Run the Project
-------------------------------

1. Edit the source code in the src/ directory.
2. Compile the application and the virtual cable program using the provided Makefile.
3. Run the virtual cable program (either by running the executable manually or using the Makefile target).
   Note that the virtual cable program requires the installation of "socat".
    (Option 1) $ sudo ./bin/cable_app
    (Option 2) $ sudo make run_cable

4. Test the protocol without cable disconnections and noise
    4.1 Run the receiver (either by running the executable manually or using the Makefile target):
        (Option 1) $ ./bin/main /dev/ttyS11 9600 rx penguin-received.gif
        (Option 2) $ make run_rx

    4.2 Run the transmitter (either by running the executable manually or using the Makefile target):
        (Option 1) $ ./bin/main /dev/ttyS10 9600 tx penguin.gif
        (Option 2) $ make run_tx

    4.3 Check if the file received matches the file sent, using the diff Linux command or using the Makefile target:
        (Option 1) $ diff -s penguin.gif penguin-received.gif
        (Option 2) $ make check_files

5. Test the protocol with cable disconnections and noise
    5.1. Run receiver and transmitter again
    5.2. Quickly move to the cable program console and press 0 for unplugging the cable, 2 to add noise, and 1 to normal
    5.3. Check if the file received matches the file sent, even with cable disconnections or with noise


  no idea
  1. why is it not closing on reciver end 


DONE:
  apllication_layer
  1. change to little endian of the fileSze info size indicator 
  2. apllication layer when reciving END ceck match byteSTART == byteEND == byteReceived
  3. remove seq number from apllication layer
  4. not saving START file name correctly

  link_layer
  2. verify when reciving response ignore if not RR or REJ
  3. change Receiver response to A_TX
  4. when RX send DISC its a command so change adress to A_RX
  5. verify closing sequence TX disc - RX disc - TX UA

  statistics (n de tramas , sucessos , duplicadas, erros, transmission duration)
  transmissions  3