# C Language Coding Rules and Guidelines

This document defines the coding rules and guidelines for this project to maximize **maintainability** and **portability**. It aims to enable multiple developers to write consistent, robust, and clean code efficiently in an open-source development environment. These rules are based on the IPA "Embedded System Coding Reading Guide (ESCR)" and adapted for practical and flexible operation.

---

## 1. Development Environment and Prerequisites

* **Target Hardware:** 32-bit microcontrollers such as Raspberry Pi Pico (Pico SDK) and ESP32 (ESP-IDF).
* **Development Environment:** Visual Studio Code (VS Code).
* **Core Philosophy:** * Prioritize code readability and consistency for open-source contributors.
* Maintain a flexible workflow that allows exceptions for debugging and extreme performance optimization, rather than overly restricting developers.



---

## 2. Basic Coding Style and Naming Conventions

### 2.1 Indentation and Formatting

* **Indentation:** Use **4 half-width spaces** (the use of tab characters is strictly prohibited).
* **VS Code Integration:** It is highly recommended to use `Clang-Format` for automatic code formatting (see Appendix for the configuration file).

### 2.2 Naming Conventions (Snake Case)

Identifiers must follow **snake_case** rules in principle. Use uppercase and lowercase letters distinctively depending on the type of identifier.

| Target | Rule | Example | Remarks |
| --- | --- | --- | --- |
| **Variables** | Lowercase snake | `motor_speed`, `target_temp` |  |
| **Functions** | Lowercase snake | `read_sensor_data()`, `init_peripherals()` |  |
| **Constants / Macros** | Uppercase snake | `MAX_BUFFER_SIZE`, `SYS_CLOCK_HZ` |  |
| **Type Definitions (typedef)** | Lowercase snake + `_t` | `config_t`, `state_type_t` | Must end with `_t` |

---

## 3. Data Types and Portability

To ensure high portability across different microcontroller architectures, the direct use of standard basic types (such as `int` or `long`) is restricted.

### 3.1 Mandatory Use of Fixed-Width Integer Types

For integer types whose sizes may vary depending on the architecture, use the fixed-width integer types defined in `<stdint.h>`.

* **Recommended Types:** `uint8_t`, `int8_t`, `uint16_t`, `int16_t`, `uint32_t`, `int32_t`
* **Exceptions for Basic Types:** * Standard `int` can be used exclusively for loop counters (e.g., `for (int i = 0; ...)`), where the specific data size does not affect the system behavior.
* Use `char` when handling characters or string literals.



---

## 4. Syntax and Control Flow Restrictions

### 4.1 Prohibition of Ternary Operators

To prevent poor readability and nested bugs, **the use of ternary operators (`? :`) is strictly prohibited in standard source code.** All conditional branches must be written using `if-else` statements instead.

* **Exception:** Ternary operators are allowed only within macro definitions (`#define`) to keep constant expressions or inline expressions concise.

### 4.2 Prohibition of Expressions with Side Effects (Evaluation Order Safety)

To prevent compiler-dependent bugs caused by ambiguous evaluation orders, **assignment operations (`=`) and increment/decrement operations (`++`, `--`) must not be mixed with other operations or conditional expressions on the same line.** They must always be executed on a standalone line.

```c
// BAD: Evaluation order is ambiguous, which can introduce bugs
array[i++] = ++j;
if (x = get_status()) { ... }

// GOOD: Safe, clear, and unambiguous
j++;
array[i] = j;
i++;

x = get_status();
if (x != 0) { ... }

```

### 4.3 Control Flow Rules (`goto` and `switch`)

* **goto Statement:** Active use is discouraged. However, using `goto` is permitted only when it provides clear benefits to keep the code clean, such as jumping to a single "centralized error handling/cleanup (resource deallocation)" block at the end of a function. The risks must be thoroughly considered before implementation.
* **switch / if-else:** To prevent unhandled states, it is highly recommended to include a `default` case in every `switch` statement and a final `else` clause in every `if - else if` chain. If no action is needed, leave an explicit comment like `/* Do nothing */`.

---

## 5. Function Design and Constants

### 5.1 Function Scale and Structure (Early Return Recommended)

* **Function Length:** A single function should generally be short enough to be read entirely on a screen without excessive scrolling. Avoid creating bloated functions.
* **Encouraged Use of Early Returns (Guard Clauses):** To avoid deeply nested code, implement guard clauses at the beginning of functions to return immediately upon detecting error conditions or invalid arguments.

```c
// GOOD: Uses early returns to avoid nesting, improving readability
int32_t process_data(uint8_t *data) {
    if (data == NULL) {
        return ERROR_INVALID_PARAM; // Guard clause
    }
    if (!is_system_ready()) {
        return ERROR_NOT_READY;     // Guard clause
    }

    // Main process goes here
    return SUCCESS;
}

```

### 5.2 Strict Prohibition of Magic Numbers

Do not use raw numerical literals with ambiguous meanings (e.g., `if (status == 3)` or `delay_ms(50);`) directly in the code. Always define meaningful names for numbers.

* The method of defining constants (`#define`, `const` variables, or `enum`) should be chosen based on the context of the module; no uniform restriction is imposed.

---

## 6. Pointer and Memory Management Safety

### 6.1 Pointer Safety (Null Pointer Defense)

To prevent critical illegal memory access bugs (which cause system crashes or memory corruption), enforce the following defensive programming rules:

* **NULL Checks on Arguments:** Functions that accept pointers as arguments must perform a `NULL` check at the very beginning of the function.
* **NULL Initialization:** When declaring a pointer variable, always initialize it to `NULL`, unless a valid address is assigned to it immediately upon declaration.

### 6.2 Principle Prohibition of Dynamic Memory Allocation (`malloc` / `free`)

To prevent memory fragmentation and memory leaks in long-running embedded systems, **dynamic memory allocation inside loops or recurring routines is strictly prohibited.**

* In principle, rely on static allocation (such as `static` arrays or global buffers).
* **Exception Rule:** Dynamic memory allocation is permitted exclusively during the initial startup phase (e.g., at the beginning of `setup()` or `app_main()`). If dynamic memory must be used anywhere else, **the specific reason for its necessity must be explicitly documented in the comments.**

---

## 7. Layered Architecture and Dependencies

To ensure system maintainability and smooth portability between different MCUs (e.g., Raspberry Pi Pico and ESP32), the codebase must be structured into the following three distinct layers:

```
[Upper]  Application Layer (Main business logic, application flow)
   │
[Middle] Middleware / Control Layer (Protocols, data processing, hardware abstraction)
   │
[Lower]  Driver / HAL Layer (SDK-specific logic, peripheral control)

```

### 7.1 Cross-Layer Call Rules

* **Principle (Recommended):** Dependencies must flow in a **single direction, from top to bottom**. If a lower layer needs to notify an upper layer of an event, it must not call the upper-layer function directly; use **callbacks (function pointers)** instead.
* **Flexible Exceptions:** For debugging purposes or modules explicitly declared as "outside the layered architecture," flexible cross-layer calls and mutual references are permitted.

### 7.2 Isolation of Hardware-Dependent Code

* **SDK Function Utilization:** To simplify future MCU migrations, hardware control for peripherals like SPI or I2C must be encapsulated with abstraction functions in the control layer.
* **Flexible Exceptions:** Simple GPIO operations (e.g., toggling a status LED) and standard logging outputs (`printf` or SDK-specific log functions) may be called directly from the application layer.
* **Strict Prohibition:** **Directly writing register addresses (pointer manipulation to hardcoded memory addresses) in the source code is strictly prohibited.** You must always use the APIs or macros provided by the vendor SDK.

### 7.3 Header File (`.h`) Include Rules

* **Self-Contained Headers Recommended:** If a header file depends on other type definitions or macros, do not hide the dependency. **Include the necessary files directly inside the header file.** This ensures that any file including this header automatically gains access to all required definitions (self-containment).
* **Prevention of Double Inclusion:** To avoid build errors from duplicate inclusions, **include guards are mandatory for all header files**. For simplicity, use `#pragma once`.

```c
#pragma once

#include <stdint.h>
#include "project_types.h" // Actively include required headers

// Struct and function declarations...

```

---

## 8. Comments and Documentation (Doxygen Format)

To enhance code readability for open-source contributors and to improve the developer experience in VS Code (such as enabling description tooltips on hover), **all public functions (declared in header files) should include Doxygen-style comments.**

### 8.1 Documentation Example

```c
/**
 * @brief Sets the target speed of the motor.
 * @param[in] target_speed The desired speed (Unit: rpm).
 * @return int32_t The actual speed applied to control, or a negative value on error.
 */
int32_t set_motor_speed(int32_t target_speed);

```
