# High Throughput Genome Releaser for Rapid and Efficient PCR Screening

## Overview

This project is a novel controller system designed for the **High Throughput Genome Releaser (HTGR)**, a device that enables rapid and efficient PCR screening. The controller integrates a **Matlab-based GUI** for user interaction and an **Arduino microcontroller (MCU)** for precise control over the hardware components.

The system is compatible with **96-well commercial plates**, making it versatile and suitable for high-throughput laboratory workflows.

## Features

- **User-Friendly GUI**: The Matlab-based GUI provides an intuitive interface for controlling the HTGR system.
- **Arduino-Controlled Hardware**: The Arduino MCU manages hardware functions, allowing precise control of the stepper motor, linear actuator, and load cell sensors.
- **High Compatibility**: Designed to work with standard 96-well plates for seamless integration into existing lab protocols.

## System Components

- **Matlab App**: The GUI for configuring, controlling, and monitoring the HTGR system.
- **Arduino Code**: Firmware running on the Arduino MCU that interfaces with the hardware components.
- **Hardware Configuration**:
  - **Stepper Motor with Linear Actuator**: Drives the movement required for the genome release process.
  - **Load Cell Sensors**: Monitors and ensures controlled force application for consistent performance.

## Requirements

- **Matlab R2022b** (ensure required toolboxes are installed for full compatibility)
- **Arduino IDE 1.8.19** (used for uploading firmware to the Arduino MCU)

## Setup Instructions

### Hardware Assembly

1. Connect the stepper motor to the linear actuator as specified in the hardware configuration guide.
2. Integrate the load cell sensors with the Arduino for force feedback.
3. Mount the Arduino, stepper motor, and sensors according to the provided schematics.

### Software Installation

1. **Matlab GUI**:
   - Open the Matlab App (provided in the `matlab/` folder).
   - Follow instructions in the GUI setup guide to configure Matlab for the HTGR system.

2. **Arduino Firmware**:
   - Open the Arduino IDE version 1.8.19.
   - Load the Arduino code from the `arduino/` directory.
   - Upload the code to the Arduino MCU to enable communication with the Matlab GUI and control over the hardware.

### Operation

1. Load a 96-well plate into the HTGR system.
2. Launch the Matlab GUI and follow the prompts to configure the settings.
3. Use the GUI controls to initiate and monitor the genome release process.

## Project Structure

```bash
HTGR-Controller/
├── arduino/                     # Arduino source files
│   └── HTGR_mcu.ino             # Main Arduino code file
├── matlab/                      # Matlab app development files
│   └── matlab_gui.mlapp         # Matlab GUI app file
├── docs/                        # Hardware setup and configuration details
│   ├── schematics/              # HTGR schematics
│   ├── parts_summary.md         # Summary of controller parts used
│   └── setup_instructions.md    # Hardware assembly and configuration instructions
├── README.md                    # Project overview and instructions
└── LICENSE                      # Licensing information
```

## License

This project is licensed under the **BSD 3-Clause License**. See the `LICENSE` file for details.

## Citation

If you use this project in your research, please cite it as follows:

@article{Author_Year,
    title = {An Innovative High Throughput Genome Releaser for Rapid and Efficient PCR Screening},
    author = {TBD},
    journal = {TBD},
    year = {TBD},
    url = {TBD}
}

## Funding

TBD
