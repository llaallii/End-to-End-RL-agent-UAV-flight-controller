# Firmware

Embedded C/C++ flight controller firmware targeting ARM Cortex-M.

## Structure

```
firmware/
├── src/             # Application source code
├── include/         # Header files
├── drivers/         # Hardware abstraction layer (HAL) drivers
├── rtos/            # RTOS configuration and wrappers
├── tests/           # Unit tests (Unity/Ceedling)
├── CMakeLists.txt   # Build configuration
└── linker/          # Linker scripts
```

## Toolchain

- ARM GCC (`arm-none-eabi-gcc`)
- CMake build system
- Unity + Ceedling for unit testing
- QEMU for emulated testing

> **Note:** MCU target (STM32F4 vs STM32H7) will be selected in Phase 4/5.
> CI/CD uses a generic Cortex-M4 target for now.
