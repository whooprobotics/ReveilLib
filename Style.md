# Code styling & standards for ReveilLib

This project follows a modified version of the Chromium style guide. This style is defined in `.clang-tidy` and `.clang-format`

## Capitalization

Capitalization of identifiers is completely different from Chromium. It is based on the Rust Style Guide.

| Item                     | Convention             |
| ------------------------ | ---------------------- |
| Struct/Class names       | `UpperCamelCase`       |
| Functions & Methods      | `snake_case`           |
| Local & Member Variables | `snake_case`           |
| Constants                | `SCREAMING_SNAKE_CASE` |