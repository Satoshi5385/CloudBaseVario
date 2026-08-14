# C Language Coding Rules

This document defines the C rules used to preserve CloudBaseVario maintainability, portability, and safety. Rules distinguish mandatory requirements from recommendations. An exception to a mandatory rule is allowed only when every exception condition stated here is met.

## 1. Scope

- Mandatory rules apply to project-owned `SRC/**/*.c` and `SRC/**/*.h`.
- `tests/`, `tools/`, and the upstream-derived `components/esp_tinyusb/` are outside the automated scope of this document. They follow their language or upstream rules.
- The target is ESP32-S3 firmware using ESP-IDF 6.0.2.
- A coding-rules change must not alter an existing external interface, wire format, or safety decision by itself.

## 2. Formatting and Naming

### 2.1 Mandatory

- Use UTF-8, normal line endings, four spaces for indentation, and no tab characters.
- Use lower snake case for variables and functions, and upper snake case for constants and macros.
- Project-owned `typedef` names end in `_t`.
- Names that store time, voltage, frequency, distance, or similar quantities include units such as `_ms`, `_us`, `_v`, `_hz`, or `_m` where practical.
- File-scope state has a preceding comment when its owner, meaning, or unit is not clear from its name.

### 2.2 Recommended

- Use the root `.clang-format` and target a 100-column line length.
- No boilerplate file header is required. Add a leading responsibility comment only when the file name is insufficient.

## 3. Types and Portability

### 3.1 Mandatory

- Use fixed-width types such as `uint32_t` and `int32_t` for integers stored, calculated, persisted, or transmitted by the application.
- Use unsigned types for bitwise and shift operations and make required casts explicit.
- At SDK, standard-library, and callback ABI boundaries, use the exact required type, including `int` or `long`.
- `size_t` may be used for sizes, indexes, and counts when it matches the API and meaning. `bool`, enumerations, and `char` for text are also allowed.
- A narrowing conversion at an API boundary is allowed only after range validation or when a defined upper bound proves it safe.

## 4. Syntax and Control Flow

### 4.1 Ternary Operator (Mandatory)

- Do not use the ternary operator `? :` in ordinary C code. Use `if` / `else` and evaluate the condition once.
- The replacement list of a `#define` may use it for a concise constant or inline expression.
- A `?` inside a comment, string literal, or character constant is not an operator and is excluded.

### 4.2 Side Effects (Mandatory)

- Do not embed `++` or `--` in an array index, function argument, assigned value, condition, or another operation.
- Put the update in its own statement. The update clause of an ordinary `for` loop is allowed.
- Do not assign inside a condition. Separate assignment from testing.

### 4.3 Control Flow

- Guard clauses for invalid input and errors are recommended.
- Use `goto` only when it safely centralizes ownership cleanup at the end of a function.
- A `default` case and the final `else` in an `if` / `else if` chain are recommended. Comment an intentional no-op.

## 5. Constants and Function Design

### 5.1 Magic Numbers (Mandatory)

- Use named constants for specification values such as time, voltage, frequency, size, count, threshold, GPIO, register, and protocol values.
- Initial or Boolean `0` / `1`, array indexes, string terminators, obvious small counts, and mathematical constants whose meaning is clear from the expression are allowed.
- Give data-sheet values a named constant or a nearby explanatory comment.

### 5.2 Functions

- Keep one responsibility per function and avoid excessive nesting.
- Initialize local variables to safe values at declaration. A valid returned address or value may instead initialize the variable in that declaration.

## 6. Pointers and Memory Management

### 6.1 Pointers (Mandatory)

- Check mandatory pointer arguments of public functions for `NULL` before dereferencing them.
- Check optional pointer arguments before every use.
- SDK callbacks and internal helpers whose callers already validated pointers may be exceptions. Document their non-null precondition immediately before the function.
- Initialize a pointer variable to `NULL` unless its declaration initializes it with a valid address.

### 6.2 Dynamic Memory (Mandatory)

- Do not allocate dynamic memory in periodic tasks, recurring processing loops, or interrupt handlers.
- Startup initialization or a low-frequency serialized transaction may allocate only when it has a size bound and explicit ownership and cleanup paths.
- An explicit or implicit allocation outside startup has a directly preceding comment containing `CODING_RULES_DYNAMIC_MEMORY:` and explaining necessity, maximum size or bound, serialization, and cleanup responsibility.
- Library APIs that allocate internally, including cJSON parsing, follow the same rule.

## 7. Layers and Dependencies

- `domain/` owns SDK-independent values, calculations, and policies. It must not depend on `app/`, `platform/`, or ESP-IDF headers.
- `platform/` owns ESP-IDF, device, persistence, and communication adapters. It may use `domain/` values and policies, but must not depend on `app/`.
- `app/` owns startup, tasks, resources, and layer integration. It may use `domain/` and `platform/`.
- Use callbacks, queues, or event bits for notifications toward the application layer.
- Never access a hard-coded register address directly. Use an SDK or device-driver API.

## 8. Headers, Includes, and Comments

### 8.1 Mandatory

- Every header has `#pragma once` or a correct include guard. New headers use `#pragma once`.
- Headers are self-contained and directly include definitions required by their public declarations.
- A `.c` file includes its own header first, followed by standard C, SDK, and project headers.
- Every public function declared in a header has at least a Doxygen-style responsibility comment. Document parameters, units, returns, and nullability when they are not obvious.

## 9. Automated Checks and Review

- CI checks `SRC/` for tabs, non-macro ternary operators, embedded side effects, unannotated dynamic allocation, header guards, and forbidden dependency directions.
- Review also checks semantic types, pointer preconditions, magic numbers, units, and public comments to avoid unreliable automated guesses.
- After a compliance change, run Python tests, host C tests, `git diff --check`, and a full ESP-IDF build.
- Builds and automated tests do not prove device behavior. Validate GPIO, LEDs, audio, BLE, USB, configuration persistence, and OTA separately when required.

## Appendix: `.clang-format`

The root configuration is authoritative.

```yaml
BasedOnStyle: LLVM
Language: Cpp
IndentWidth: 4
UseTab: Never
ColumnLimit: 100
AllowShortFunctionsOnASingleLine: None
BreakBeforeBraces: Attach
SpaceAfterCStyleCast: true
```
