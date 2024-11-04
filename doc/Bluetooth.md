This file will house the program flow for the bluetooth library. Made from research with CUBBIDE and example code. Here are the steps

# 1: Initialising
## 1.1: Retrieve trimming
Retrieve the flash trimming from the flash if not exist predefine them. The datasheet describes that is automatically provided by the POR ( power on reset ).

* IBias
* IPtat
* VDG
* RXADC delay I (I channel)
* RXADC delay Q (Q channel)
* RCADC delay flag
