#!/usr/bin/env python3
"""Assemble the doc/ tree into a single book and render it to PDF and/or HTML.

Chapter and section ordering is *derived* from the existing README tables, so
the booklet stays in sync with the library automatically:

  1. Category order comes from the Documentation table in the top-level README.md
     (each `[Name](doc/<domain>/README.md)` link, in order).
  2. Algorithm order within a category comes from that category's
     `doc/<domain>/README.md` table (each `[Title](File.md)` link, in order).
  3. Any doc/**/*.md not reached by the tables (excluding README.md, TEMPLATE.md
     and .backup/) is grouped by top-level domain and appended as its own chapter,
     with a warning — so a new doc is never silently dropped.

Each algorithm's "References & Further Reading" section is extracted, deduplicated
and emitted once as a consolidated back-matter chapter.

Rendering uses Pandoc: XeLaTeX for the PDF, standalone HTML (MathJax) for Pages.

Usage:
    python scripts/build-booklet.py [--format pdf|html|all] [--assemble-only]

Exit codes:
    0 — success (or, with --assemble-only, book.md written)
    1 — doc/ missing, no docs found, or a Pandoc render failed
    2 — Pandoc / required engine not installed
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys
from datetime import date

ROOT = pathlib.Path(__file__).resolve().parent.parent
DOC_ROOT = ROOT / "doc"
README = ROOT / "README.md"
BOOKLET_ASSETS = ROOT / "scripts" / "booklet"
BUILD_DIR = ROOT / "build" / "booklet"

SKIP_NAMES = {"README.md", "TEMPLATE.md"}
SKIP_DIRS = {".backup"}

REFERENCES_HEADING = "References & Further Reading"
OUTPUT_STEM = "NumericalToolbox"

LINK_RE = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")
H1_RE = re.compile(r"^#\s+(.+?)\s*$", re.MULTILINE)
HEADING_RE = re.compile(r"^(#{1,6})\s")
IMAGE_RE = re.compile(r"(!\[[^\]]*\]\()([^)]+)(\))")


def category_order() -> list[pathlib.Path]:
    """Return category doc dirs in the order the top-level README lists them."""
    text = README.read_text(encoding="utf-8")
    dirs: list[pathlib.Path] = []
    seen: set[pathlib.Path] = set()
    for _, target in LINK_RE.findall(text):
        target = target.split("#", 1)[0].strip()
        if not target.startswith("doc/") or not target.endswith("/README.md"):
            continue
        cat = (ROOT / target).resolve().parent
        if cat not in seen and cat.is_dir():
            seen.add(cat)
            dirs.append(cat)
    return dirs


def algorithm_docs(category: pathlib.Path) -> list[pathlib.Path]:
    """Ordered algorithm doc paths reached from a category's README table.

    Links to README.md, out-of-category paths (../) and URLs are cross-references,
    not algorithms, and are skipped.
    """
    readme = category / "README.md"
    if not readme.is_file():
        return []
    docs: list[pathlib.Path] = []
    seen: set[pathlib.Path] = set()
    for _, target in LINK_RE.findall(readme.read_text(encoding="utf-8")):
        target = target.split("#", 1)[0].strip()
        if not target.endswith(".md"):
            continue
        if target.startswith(("http://", "https://", "../")) or "README.md" in target:
            continue
        path = (category / target).resolve()
        if path in seen or not path.is_file():
            continue
        seen.add(path)
        docs.append(path)
    return docs


def all_docs() -> list[pathlib.Path]:
    return sorted(
        p
        for p in DOC_ROOT.rglob("*.md")
        if p.name not in SKIP_NAMES
        and not any(part in SKIP_DIRS for part in p.parts)
    )


def title_of(path: pathlib.Path, fallback: str) -> str:
    match = H1_RE.search(path.read_text(encoding="utf-8"))
    return match.group(1).strip() if match else fallback


def split_references(text: str) -> tuple[str, list[str]]:
    """Strip the References section from a doc body; return (body, ref_lines)."""
    lines = text.splitlines()
    out: list[str] = []
    refs: list[str] = []
    current: list[str] = []
    in_refs = False

    def flush() -> None:
        if current:
            refs.append(" ".join(current))
            current.clear()

    for line in lines:
        heading = HEADING_RE.match(line)
        if heading and REFERENCES_HEADING.lower() in line.lower():
            in_refs = True
            continue
        if in_refs and heading:
            flush()
            in_refs = False
        if in_refs:
            stripped = line.strip()
            if stripped.startswith(("-", "*")):
                flush()
                current.append(stripped.lstrip("-* ").strip())
            elif stripped:
                current.append(stripped)  # continuation of a wrapped reference
            else:
                flush()
            continue
        out.append(line)
    flush()
    return "\n".join(out), refs


def demote_and_rewrite(text: str, doc: pathlib.Path) -> str:
    """Demote every heading one level and make image paths absolute.

    Skips heading/image edits inside fenced code blocks.
    """
    lines = text.splitlines()
    out: list[str] = []
    in_fence = False
    for line in lines:
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
            out.append(line)
            continue
        if in_fence:
            out.append(line)
            continue
        if HEADING_RE.match(line):
            line = "#" + line
        line = IMAGE_RE.sub(lambda m: _abs_image(m, doc), line)
        out.append(line)
    return "\n".join(out)


def _abs_image(match: re.Match, doc: pathlib.Path) -> str:
    target = match.group(2).strip()
    if target.startswith(("http://", "https://", "/")):
        return match.group(0)
    resolved = (doc.parent / target).resolve()
    return f"{match.group(1)}{resolved}{match.group(3)}"


def normalize_ref(line: str) -> str:
    body = line.lstrip("-* ").strip()
    return re.sub(r"\s+", " ", body).lower()


def assemble(book_md: pathlib.Path) -> int:
    if not DOC_ROOT.is_dir():
        print(f"ERROR: doc directory not found at {DOC_ROOT}", file=sys.stderr)
        return 1

    remaining = set(all_docs())
    if not remaining:
        print("ERROR: no algorithm documentation files found.", file=sys.stderr)
        return 1

    # One chapter per top-level domain, keyed by its doc dir; README order first.
    chapters: dict[pathlib.Path, list[pathlib.Path]] = {}
    for category in category_order():
        docs = [d for d in algorithm_docs(category) if d in remaining]
        remaining.difference_update(docs)
        if docs:
            chapters[category] = docs

    # Fallback: any doc not reached by the README tables is appended to its domain's
    # chapter (or a new chapter if that domain has none), never silently dropped.
    leftovers: dict[pathlib.Path, list[pathlib.Path]] = {}
    for doc in sorted(remaining):
        top = DOC_ROOT / doc.relative_to(DOC_ROOT).parts[0]
        leftovers.setdefault(top, []).append(doc)
    for top, docs in sorted(leftovers.items()):
        rel = [str(d.relative_to(DOC_ROOT)) for d in docs]
        print(f"WARNING: {len(docs)} doc(s) not listed in README tables: {rel}")
        chapters.setdefault(top, []).extend(docs)

    parts: list[str] = []
    references: list[str] = []
    seen_refs: set[str] = set()

    for category, docs in chapters.items():
        parts.append(f"# {title_of(category / 'README.md', category.name)}\n")
        for doc in docs:
            body, refs = split_references(doc.read_text(encoding="utf-8"))
            parts.append(demote_and_rewrite(body, doc).strip() + "\n")
            for ref in refs:
                key = normalize_ref(ref)
                if key and key not in seen_refs:
                    seen_refs.add(key)
                    references.append(ref.lstrip("-* ").strip())

    if references:
        parts.append("# References\n")
        parts.extend(f"- {ref}" for ref in sorted(references))
        parts.append("")

    book_md.parent.mkdir(parents=True, exist_ok=True)
    book_md.write_text("\n".join(parts), encoding="utf-8")
    total = sum(len(d) for d in chapters.values())
    print(
        f"Assembled {total} algorithm doc(s) into {len(chapters)} chapter(s) "
        f"and {len(references)} unique reference(s) -> {book_md}"
    )
    return 0


def pandoc_common(book_md: pathlib.Path) -> list[str]:
    return [
        "pandoc",
        str(book_md),
        "--metadata-file",
        str(BOOKLET_ASSETS / "metadata.yaml"),
        "-M",
        f"date={date.today():%B %d, %Y}",
        "-M",
        f"version={_git_version()}",
        "--toc",
        "--toc-depth=2",
        "--number-sections",
    ]


def _git_version() -> str:
    try:
        out = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "dev"


def render_pdf(book_md: pathlib.Path) -> int:
    out = BUILD_DIR / f"{OUTPUT_STEM}.pdf"
    cmd = pandoc_common(book_md) + [
        "--pdf-engine=xelatex",
        "-H",
        str(BOOKLET_ASSETS / "preamble.tex"),
        "--include-after-body",
        str(BOOKLET_ASSETS / "back-cover.tex"),
        "-o",
        str(out),
    ]
    return _run(cmd, out)


def render_html(book_md: pathlib.Path) -> int:
    out = BUILD_DIR / "index.html"
    shutil.copyfile(BOOKLET_ASSETS / "book.css", BUILD_DIR / "book.css")
    cmd = pandoc_common(book_md) + [
        "--standalone",
        "--mathjax",
        "--css=book.css",
        "-o",
        str(out),
    ]
    return _run(cmd, out)


def _run(cmd: list[str], out: pathlib.Path) -> int:
    result = subprocess.run(cmd, cwd=ROOT)
    if result.returncode != 0:
        print(f"ERROR: pandoc failed for {out.name}", file=sys.stderr)
        return 1
    print(f"Wrote {out}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the documentation booklet.")
    parser.add_argument(
        "--format", choices=["pdf", "html", "all"], default="all"
    )
    parser.add_argument(
        "--assemble-only",
        action="store_true",
        help="Write build/booklet/book.md and stop (no Pandoc).",
    )
    args = parser.parse_args()

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    book_md = BUILD_DIR / "book.md"
    status = assemble(book_md)
    if status or args.assemble_only:
        return status

    if shutil.which("pandoc") is None:
        print("ERROR: pandoc not found on PATH.", file=sys.stderr)
        return 2

    rc = 0
    if args.format in ("pdf", "all"):
        rc |= render_pdf(book_md)
    if args.format in ("html", "all"):
        rc |= render_html(book_md)
    return 1 if rc else 0


if __name__ == "__main__":
    raise SystemExit(main())
