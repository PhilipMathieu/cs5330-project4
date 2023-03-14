# CS5330 Template Repository

This repository is a template for CS5330 at Northeastern University, Spring 2023, taught by Prof. Bruce Maxwell. __It is designed for use in Ubuntu 20.04 running in WSL2 on a Windows 11 computer and may not be directly transferable to other environments.__

## Installation
See INSTALL.md for notes on my environment setup.

## Directory Structure

```
./              Makefile and configure scripts
    ./bin       Executables build directory
    ./data      Any data created by the project. May also include data used 
                by the project, but avoid adding large directories
    ./include   Header files (.h)
    ./src       General sources (.cpp)
```

This is a fairly conventional structure. My explanation draws inspiration from [this StackOverflow response (CC BY-SA 2.5)](https://stackoverflow.com/a/1398594/2213454)

In order to track these directories with git, I have added empty `.gitkeep` files in each empty directory. This may not be standard practice, but it is an efficient way of making sure the directory structure is enforced.

## Makefile
The included Makefile has not been tested extensively and may need modification to match your system.

## Credits
Created by Philip Englund Mathieu and licensed under the [MIT License](LICENSE.md).