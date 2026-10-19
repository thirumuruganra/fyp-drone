import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from env_data import load_dotenv, parse_dotenv  # noqa: E402


class ParseTests(unittest.TestCase):
    def test_formats(self):
        text = ("# comment\n\nOPENTOPO_API_KEY=abc123\nexport A=1\nB = \"two words\"\nC='x#y'\n"
                "D=plain # trailing comment\nnot a line\n1BAD=x\n")
        self.assertEqual(parse_dotenv(text), {"OPENTOPO_API_KEY": "abc123", "A": "1", "B": "two words",
                                              "C": "x#y", "D": "plain"})


class LoadTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, ".env")
        with open(self.path, "w", encoding="utf-8") as fh:
            fh.write("FYP_TEST_KEY=from-file\nFYP_TEST_OTHER=2\n")

    def tearDown(self):
        self.tmp.cleanup()

    def load(self, **kw):
        with redirect_stdout(io.StringIO()) as out:
            names = load_dotenv([self.path], **kw)
        return names, out.getvalue()

    def test_sets_missing_variables_without_printing_values(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("FYP_TEST_KEY", None)
            os.environ.pop("FYP_TEST_OTHER", None)
            names, log = self.load()
            self.assertEqual(os.environ["FYP_TEST_KEY"], "from-file")
            self.assertEqual(sorted(names), ["FYP_TEST_KEY", "FYP_TEST_OTHER"])
            self.assertNotIn("from-file", log)

    def test_real_environment_wins_unless_override(self):
        with mock.patch.dict(os.environ, {"FYP_TEST_KEY": "from-shell"}):
            self.load()
            self.assertEqual(os.environ["FYP_TEST_KEY"], "from-shell")
            self.load(override=True)
            self.assertEqual(os.environ["FYP_TEST_KEY"], "from-file")

    def test_missing_file_is_ignored(self):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(load_dotenv([os.path.join(self.tmp.name, "nope.env")]), [])


if __name__ == "__main__":
    unittest.main()
