### REAL TIME CADMIUM INSTALL ###

Clone this repo into an empty folder, ensure you download the MBed submodules:

> git submodule update --init --recursive

Run this to install dependencies

> RT-Cadmium-Blinky/install.sh

### SIMULATE MODELS ###

> cd RT-Cadmium-Blinky/top_model/

> make clean; make all

> ./BLINKY_TOP

This will run the standard Cadmium simulator. Cadmium logs will be generated in Blinky_ECadmiu/top_model/blinky_test_output.txt
The pin's inputs are stored in Blinky_ECadmiu/top_model/inputs. The value of the output pins will be in Blinky_ECadmiu/top_model/inputs.
SVEC (Simulation Visualization for Embedded Cadmium) is a python GUI that parses these files and steps through the simulation to help debug the models.


### RUN MODELS ON TARGET PLATFORM ###

If your target platform *is not* the Nucleo-STM32F401, you will need to change the COMPILE_TARGET / FLASH_TARGET in the make file.

> cd RT-Cadmium-Blinky/top_model/

> make clean; make embedded; make flash;

