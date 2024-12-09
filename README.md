# HD-Robots

## Project Overview
This repository contains the code and project structure for **HD-Robots**, a robotics project developed using the PROS kernel. The project includes functionality for operator control and autonomous operations for our IME 100 Robot Competition.

## Project Structure

```
HD-Robots/
├── project.pros        # Metadata for the PROS CLI (kernel version, etc.)
├── Makefile            # Instructions for compiling the project
├── common.mk           # Helper file used by the Makefile
│
├── static/             # Contains PATH.JERRYIO scripts
│   ├── example.txt     # Example path script
│
├── src/                # Source files go here
│   ├── main.cpp        # Source for competition task functions
│                       # (e.g., operator control and autonomous)
│
├── include/            # Header files go here
│   ├── api.h           # Includes PROS API functions
│   ├── main.h          # Includes `api.h` and project-wide headers
│   ├── lemlib/         # LemLib library folder
│   ├── pros/           # Header files for the PROS API functions
│   ├── okapi/          # Header files for OkapiLib
│   ├── display/        # Header files for LVGL (graphics library for the V5 screen)
│
├── firmware/           # Precompiled libraries and firmware files
│   ├── libpros.a       # Precompiled PROS library
│   ├── okapilib.a      # Precompiled OkapiLib library
│   ├── v5.ld           # Linker script for constructing binaries for the V5
```

## Key Files and Directories

### `static/`
This directory contains scripts for **PATH.JERRYIO**, including:
- **`example.txt`**: A sample path script.

### `src/`
All source files for the project go here.  
- **`main.cpp`**: Contains the main competition task functions:
  - Operator control
  - Autonomous routines

### `include/`
All header files for the project should go here. This includes:
- **`api.h`**: PROS API functions.
- **`main.h`**: Project-wide includes and configurations.
- Subdirectories for specific libraries and utilities:
  - **`lemlib/`**: LemLib library.
  - **`pros/`**: PROS API headers.
  - **`okapi/`**: OkapiLib headers.
  - **`display/`**: LVGL headers for V5 screen graphics.

## Getting Started

### Requirements
- **[PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/cli/)**: Required for building and uploading the project.
- **LemLib**: Included in the `include/` directory.

### Compilation
1. Open a terminal in the project root.
2. Run the following command:
   
   ```bash
   pros make
   ```

### Uploading
To upload the compiled code to the robot, use:
```bash
pros upload
```
