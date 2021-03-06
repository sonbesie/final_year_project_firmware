# Final Year Project: Firmware

## Description
The Nucleo-F767ZI firmware for my final year project. The main project files written by me are included in the Core directory. The source files are organised as follows:

1. camera.c and camera.h : The custom driver for the SC20MPB single board camera.
2. image.c, image.h and image_hard.h : The custom image analysis library. This library performs the thorax detection on the athlete as well as the motion detection.
3. jfif.c, jfif.h and jfif_hard.h : The custom JPEG decoding library.
4. mem.c and mem.h : The custom memory driver.
5. mis.c and mis.h : Miscellaneous functions that controls the screen output and starting gun trigger.
6. network.c and network.h : The custom layer two network protocol is implemented in this library.
7. main.c and main.h : The main program. The main() function is located here.

I used the hardware abstraction layer libraries provided by STMicroelectronics to communicate with the peripherals. These libraries are in the Drivers directory and are the property of STMicroelectronics.

## Open
This project can be compiled using the STM32Cube IDE. Download the project, open the IDE and import the project from your file system. 

## Image

A render of the final camera module with the two lenses of the camera pointing downwards to the athletics track is given below:

![alt text](https://github.com/sonbesie/final_year_project_firmware/blob/main/assembly_s.JPG?raw=true)
