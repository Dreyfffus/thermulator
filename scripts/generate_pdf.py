#!/usr/bin/env python3

from pathlib import Path
import textwrap


PAGE_WIDTH = 612
PAGE_HEIGHT = 792
LEFT = 54
TOP = 54
BOTTOM = 54
FONT_SIZE = 11
LINE_HEIGHT = 14
MAX_CHARS = 88


def escape_pdf(text: str) -> str:
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def wrap_markdown(text: str) -> list[str]:
    lines: list[str] = []
    in_code = False
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("```"):
            in_code = not in_code
            lines.append("")
            continue
        if not line:
            lines.append("")
            continue
        if in_code:
            lines.append(line)
            continue
        if line.startswith("#"):
            lines.append(line.lstrip("#").strip().upper())
            lines.append("")
            continue
        if line.startswith("- "):
            wrapped = textwrap.wrap(
                line[2:],
                width=MAX_CHARS - 2,
                subsequent_indent="  ",
                break_long_words=False,
                break_on_hyphens=False,
            )
            if wrapped:
                lines.append("* " + wrapped[0])
                lines.extend(wrapped[1:])
            else:
                lines.append("*")
            continue
        wrapped = textwrap.wrap(
            line,
            width=MAX_CHARS,
            break_long_words=False,
            break_on_hyphens=False,
        )
        lines.extend(wrapped or [""])
    return lines


def paginate(lines: list[str]) -> list[list[str]]:
    usable = PAGE_HEIGHT - TOP - BOTTOM
    lines_per_page = usable // LINE_HEIGHT
    pages: list[list[str]] = []
    current: list[str] = []
    for line in lines:
        if len(current) >= lines_per_page:
            pages.append(current)
            current = []
        current.append(line)
    if current or not pages:
        pages.append(current)
    return pages


def build_pdf(pages: list[list[str]]) -> bytes:
    objects: list[bytes] = []

    def add_object(body: str | bytes) -> int:
        data = body.encode("latin-1") if isinstance(body, str) else body
        objects.append(data)
        return len(objects)

    font_id = add_object("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")
    page_ids: list[int] = []
    content_ids: list[int] = []

    for page_lines in pages:
        y = PAGE_HEIGHT - TOP
        commands = ["BT", f"/F1 {FONT_SIZE} Tf"]
        for line in page_lines:
            commands.append(f"1 0 0 1 {LEFT} {y} Tm ({escape_pdf(line)}) Tj")
            y -= LINE_HEIGHT
        commands.append("ET")
        stream = "\n".join(commands).encode("latin-1")
        content_id = add_object(
            b"<< /Length " + str(len(stream)).encode("ascii") + b" >>\nstream\n" +
            stream + b"\nendstream"
        )
        content_ids.append(content_id)
        page_ids.append(0)

    kids = []
    placeholder_pages_id = len(objects) + 1
    for idx, content_id in enumerate(content_ids):
        page_obj = (
            f"<< /Type /Page /Parent {placeholder_pages_id} 0 R "
            f"/MediaBox [0 0 {PAGE_WIDTH} {PAGE_HEIGHT}] "
            f"/Resources << /Font << /F1 {font_id} 0 R >> >> "
            f"/Contents {content_id} 0 R >>"
        )
        page_ids[idx] = add_object(page_obj)
        kids.append(f"{page_ids[idx]} 0 R")

    pages_id = add_object(
        f"<< /Type /Pages /Kids [{' '.join(kids)}] /Count {len(page_ids)} >>"
    )
    catalog_id = add_object(f"<< /Type /Catalog /Pages {pages_id} 0 R >>")

    output = bytearray(b"%PDF-1.4\n")
    offsets = [0]
    for index, obj in enumerate(objects, start=1):
        offsets.append(len(output))
        output.extend(f"{index} 0 obj\n".encode("ascii"))
        output.extend(obj)
        output.extend(b"\nendobj\n")

    xref_start = len(output)
    output.extend(f"xref\n0 {len(objects) + 1}\n".encode("ascii"))
    output.extend(b"0000000000 65535 f \n")
    for offset in offsets[1:]:
        output.extend(f"{offset:010d} 00000 n \n".encode("ascii"))
    output.extend(
        (
            f"trailer\n<< /Size {len(objects) + 1} /Root {catalog_id} 0 R >>\n"
            f"startxref\n{xref_start}\n%%EOF\n"
        ).encode("ascii")
    )
    return bytes(output)


def main() -> None:
    source = Path("docs/digital_twin_test_report.md")
    target = Path("docs/digital_twin_test_report.pdf")
    lines = wrap_markdown(source.read_text(encoding="utf-8"))
    pages = paginate(lines)
    target.write_bytes(build_pdf(pages))


if __name__ == "__main__":
    main()
