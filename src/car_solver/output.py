"""Output directory management."""

from pathlib import Path

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "output"


def output_path(filename: str) -> str:
    """Return full path to a file in the output directory, creating it if needed."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    return str(OUTPUT_DIR / filename)
