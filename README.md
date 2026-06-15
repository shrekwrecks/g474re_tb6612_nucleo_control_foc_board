money money is in core/src/tasks.c

hallo.pdf is the uhh hardware. or at least the schematic. actually i had to stack two of them on top of eachother since tb6612 ties together the grounds for both full bridges together on die
so had to have two separate tb6612 to actually drive. one for each stepper coil. also my actual hardware i messed up the op amps. oh yeah ended up doing inverted op amp with offset set by dac such that 0v on the shunt -> 1.65 volts read by the adc. 
