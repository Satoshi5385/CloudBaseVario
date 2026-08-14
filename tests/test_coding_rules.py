import tempfile
import unittest
from pathlib import Path

from tools.check_coding_rules import audit_source_tree, sanitize_c_source


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class CodingRulesTests(unittest.TestCase):
    def test_project_owned_c_source_complies(self):
        violations = audit_source_tree(PROJECT_ROOT / "SRC")
        messages = [violation.format(PROJECT_ROOT) for violation in violations]
        self.assertEqual([], messages, "\n" + "\n".join(messages))

    def test_lexer_ignores_comments_strings_and_character_literals(self):
        source = "// ? ++\nconst char *text = \"? --\";\nchar value = '?';\n"
        sanitized = sanitize_c_source(source)
        self.assertNotIn("?", sanitized)
        self.assertNotIn("++", sanitized)
        self.assertNotIn("--", sanitized)

    def test_checker_accepts_macro_ternary_and_rejects_source_ternary(self):
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory) / "SRC"
            source_root.mkdir()
            path = source_root / "sample.c"
            path.write_text(
                "#define MINIMUM(a, b) ((a) < (b) ? (a) : (b))\n"
                "int32_t choose(bool condition) {\n"
                "    return condition ? 1 : 0;\n"
                "}\n",
                encoding="utf-8",
            )
            violations = audit_source_tree(source_root)
        self.assertEqual(1, len(violations))
        self.assertIn("ternary operator", violations[0].message)

    def test_checker_reports_each_mechanical_rule(self):
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory) / "SRC"
            domain_root = source_root / "domain"
            domain_root.mkdir(parents=True)
            (source_root / "sample.h").write_text(
                "void sample(void);\n", encoding="utf-8"
            )
            (domain_root / "sample.c").write_text(
                '#include "platform/board.h"\n'
                "void sample(void) {\n"
                "\titems[index++] = malloc(8U);\n"
                "}\n",
                encoding="utf-8",
            )
            violations = audit_source_tree(source_root)
        messages = {violation.message for violation in violations}
        self.assertIn("tab character", messages)
        self.assertIn("embedded increment or decrement", messages)
        self.assertIn("dynamic allocation lacks policy marker", messages)
        self.assertIn("header lacks include guard", messages)
        self.assertIn("domain depends on platform/board.h", messages)

    def test_checker_accepts_documented_allocation_and_standalone_updates(self):
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory) / "SRC"
            source_root.mkdir()
            (source_root / "sample.c").write_text(
                "void sample(void) {\n"
                "    /* CODING_RULES_DYNAMIC_MEMORY: bounded, serialized, freed. */\n"
                "    buffer = malloc(8U);\n"
                "    counter++;\n"
                "    for (size_t index = 0U; index < 2U; index++) {\n"
                "    }\n"
                "}\n",
                encoding="utf-8",
            )
            violations = audit_source_tree(source_root)
        self.assertEqual([], violations)


if __name__ == "__main__":
    unittest.main()
