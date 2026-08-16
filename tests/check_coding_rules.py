"""Check mechanically enforceable C rules for project-owned SRC files."""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path


DYNAMIC_MEMORY_MARKER = "CODING_RULES_DYNAMIC_MEMORY:"
DYNAMIC_ALLOCATION_PATTERN = re.compile(
    r"\b(?:malloc|calloc|realloc|strdup|asprintf|vasprintf|"
    r"heap_caps_malloc|heap_caps_calloc|heap_caps_realloc|"
    r"cJSON_Parse|cJSON_ParseWithLengthOpts)\s*\("
)
INCREMENT_PATTERN = re.compile(r"\+\+|--")
STANDALONE_INCREMENT_PATTERN = re.compile(
    r"^\s*(?:\(\s*\*\s*)?[A-Za-z_]\w*"
    r"(?:(?:->|\.)\w+|\[[^\]]+\])*(?:\s*\))?"
    r"\s*(?:\+\+|--)\s*;\s*$"
)
CONDITIONAL_ASSIGNMENT_PATTERN = re.compile(
    r"\b(?:if|while)\s*\([^\n)]*(?<![!<>=])=(?!=)"
)
INCLUDE_PATTERN = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
PUBLIC_FUNCTION_PATTERN = re.compile(
    r"^[ \t]*(?!typedef\b)(?:[A-Za-z_]\w*[ \t*]+)+"
    r"[A-Za-z_]\w*\s*\([^;{}]*\)\s*;",
    re.MULTILINE,
)


@dataclass(frozen=True)
class Violation:
    path: Path
    line: int
    message: str

    def format(self, root: Path) -> str:
        return f"{self.path.relative_to(root)}:{self.line}: {self.message}"


def sanitize_c_source(source: str) -> str:
    """Replace comments and literals with spaces while preserving line layout."""
    result: list[str] = []
    index = 0
    state = "code"

    while index < len(source):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""

        if state == "code":
            if char == "/" and next_char == "/":
                result.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if char == "/" and next_char == "*":
                result.extend((" ", " "))
                index += 2
                state = "block_comment"
                continue
            if char == '"':
                result.append(" ")
                index += 1
                state = "string"
                continue
            if char == "'":
                result.append(" ")
                index += 1
                state = "character"
                continue
            result.append(char)
            index += 1
            continue

        if state == "line_comment":
            if char == "\n":
                result.append("\n")
                state = "code"
            else:
                result.append(" ")
            index += 1
            continue

        if state == "block_comment":
            if char == "*" and next_char == "/":
                result.extend((" ", " "))
                index += 2
                state = "code"
                continue
            result.append("\n" if char == "\n" else " ")
            index += 1
            continue

        if char == "\\" and next_char:
            result.append(" ")
            result.append("\n" if next_char == "\n" else " ")
            index += 2
            continue
        if (state == "string" and char == '"') or (
            state == "character" and char == "'"
        ):
            result.append(" ")
            index += 1
            state = "code"
            continue
        result.append("\n" if char == "\n" else " ")
        index += 1

    return "".join(result)


def _for_update_lines(sanitized: str) -> set[int]:
    allowed_lines: set[int] = set()
    for for_match in re.finditer(r"\bfor\s*\(", sanitized):
        open_parenthesis = sanitized.find("(", for_match.start())
        depth = 0
        end = open_parenthesis
        while end < len(sanitized):
            if sanitized[end] == "(":
                depth += 1
            elif sanitized[end] == ")":
                depth -= 1
                if depth == 0:
                    break
            end += 1
        if depth != 0:
            continue
        segment = sanitized[open_parenthesis : end + 1]
        for update in INCREMENT_PATTERN.finditer(segment):
            absolute = open_parenthesis + update.start()
            allowed_lines.add(sanitized.count("\n", 0, absolute) + 1)
    return allowed_lines


def _audit_file(path: Path) -> list[Violation]:
    source = path.read_text(encoding="utf-8")
    sanitized = sanitize_c_source(source)
    source_lines = source.splitlines()
    sanitized_lines = sanitized.splitlines()
    violations: list[Violation] = []

    for line_number, line in enumerate(source_lines, 1):
        if "\t" in line:
            violations.append(Violation(path, line_number, "tab character"))

    macro_continuation = False
    for line_number, line in enumerate(sanitized_lines, 1):
        stripped = line.lstrip()
        is_define = stripped.startswith("#define") or macro_continuation
        if "?" in line and not is_define:
            violations.append(
                Violation(path, line_number, "ternary operator outside #define")
            )
        macro_continuation = is_define and line.rstrip().endswith("\\")

    allowed_for_lines = _for_update_lines(sanitized)
    for line_number, line in enumerate(sanitized_lines, 1):
        if not INCREMENT_PATTERN.search(line) or line_number in allowed_for_lines:
            continue
        if not STANDALONE_INCREMENT_PATTERN.fullmatch(line):
            violations.append(
                Violation(path, line_number, "embedded increment or decrement")
            )

    for match in CONDITIONAL_ASSIGNMENT_PATTERN.finditer(sanitized):
        line_number = sanitized.count("\n", 0, match.start()) + 1
        violations.append(Violation(path, line_number, "assignment in condition"))

    for line_number, line in enumerate(sanitized_lines, 1):
        if not DYNAMIC_ALLOCATION_PATTERN.search(line):
            continue
        first_context_line = max(0, line_number - 7)
        context = "\n".join(source_lines[first_context_line : line_number - 1])
        if DYNAMIC_MEMORY_MARKER not in context:
            violations.append(
                Violation(path, line_number, "dynamic allocation lacks policy marker")
            )

    if path.suffix == ".h" and "#pragma once" not in sanitized:
        if not re.search(r"^\s*#\s*ifndef\s+\w+", sanitized, re.MULTILINE):
            violations.append(Violation(path, 1, "header lacks include guard"))

    if path.suffix == ".h":
        for declaration in PUBLIC_FUNCTION_PATTERN.finditer(sanitized):
            prefix = source[: declaration.start()].rstrip()
            has_doxygen = prefix.endswith("*/")
            if has_doxygen:
                comment_start = prefix.rfind("/**")
                prior_comment_end = prefix.rfind("*/", 0, len(prefix) - 2)
                has_doxygen = comment_start > prior_comment_end
            if not has_doxygen:
                line_number = source.count("\n", 0, declaration.start()) + 1
                violations.append(
                    Violation(path, line_number, "public function lacks Doxygen comment")
                )

    relative_parts = path.parts
    in_domain = "domain" in relative_parts
    in_platform = "platform" in relative_parts
    for line_number, line in enumerate(source_lines, 1):
        include_match = INCLUDE_PATTERN.match(line)
        if include_match is None:
            continue
        include = include_match.group(1)
        if in_domain and (
            include.startswith(("app/", "platform/"))
            or include.startswith(
                ("driver/", "esp_", "freertos/", "nvs", "soc/", "hal/")
            )
        ):
            violations.append(
                Violation(path, line_number, f"domain depends on {include}")
            )
        if in_platform and include.startswith("app/"):
            violations.append(
                Violation(path, line_number, f"platform depends on {include}")
            )

    return violations


def audit_source_tree(source_root: Path) -> list[Violation]:
    violations: list[Violation] = []
    for path in sorted((*source_root.rglob("*.c"), *source_root.rglob("*.h"))):
        violations.extend(_audit_file(path))
    return violations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "source_root",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "SRC",
    )
    args = parser.parse_args()
    source_root = args.source_root.resolve()
    violations = audit_source_tree(source_root)
    for violation in violations:
        print(violation.format(source_root.parent))
    return 1 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())
