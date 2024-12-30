This repo sets up the DiscoveryF4 used in the drum repo (initially for the levelwind development) with the 360 pulse per rev shaft encoder connected to the drive motor shaft. 

The F4 pin usage is the same as the drum/levelwind, so that the levelwind DiscoveryF4 and related hardware can be used for these early tests. Later, when the levelwind mechanism is installed, this code will be merged into the levelwind, as the levelwind algorithm also uses the encoder.

This code primarily sets up the odometer function which tracks the amount of line out, and line speed. However, for initial testing this routine determines drum speed, acceleration. 
