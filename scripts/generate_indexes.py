#!/usr/bin/env python3
"""Generate the root landing page, docs pages, and dist indexes."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
from pathlib import Path
import re
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
DIST = ROOT / "dist"
DOCS = ROOT / "docs"


@dataclass(frozen=True)
class Package:
    name: str
    folder: Path
    wheels: tuple[Path, ...]

    @property
    def latest(self) -> Path:
        return self.wheels[0]

    @property
    def version(self) -> str:
        stem = self.latest.name.removesuffix(".whl")
        parts = stem.split("-")
        if len(parts) >= 2:
            return parts[1]
        return ""


@dataclass(frozen=True)
class Document:
    stem: str
    path: Path
    title: str
    summary: str
    source: str


def iter_packages() -> list[Package]:
    packages: list[Package] = []
    for folder in sorted(p for p in DIST.iterdir() if p.is_dir()):
        wheels = tuple(sorted(folder.glob("*.whl"), key=lambda path: path.name, reverse=True))
        if not wheels:
            continue
        packages.append(Package(name=folder.name, folder=folder, wheels=wheels))
    return packages


def iter_documents() -> list[Document]:
    if not DOCS.exists():
        return []

    documents: list[Document] = []
    for path in sorted(DOCS.glob("*.md")):
        source = path.read_text(encoding="utf-8")
        title = extract_title(source, path.stem)
        summary = extract_summary(source, title)
        documents.append(
            Document(
                stem=path.stem,
                path=path,
                title=title,
                summary=summary,
                source=source,
            )
        )
    return documents


def write_text(path: Path, content: str) -> None:
    path.write_text(content.rstrip() + "\n", encoding="utf-8")


def render_root_index(packages: Iterable[Package], documents: Iterable[Document]) -> str:
    package_list = list(packages)
    document_list = list(documents)
    docs_by_stem = {document.stem: document for document in document_list}
    cards = "\n".join(render_card(package, docs_by_stem.get(package.name)) for package in package_list)
    count = len(package_list)
    matched_doc_count = sum(1 for package in package_list if package.name in docs_by_stem)
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>pip.wgists.me 轮子仓库</title>
  <meta
    name="description"
    content="基于 GitHub Pages 托管的静态 wheel 仓库。"
  >
  <style>
    :root {{
      --bg: #f5efe3;
      --bg-strong: #efe4d1;
      --panel: rgba(255, 251, 245, 0.88);
      --panel-strong: #fffaf2;
      --text: #201a13;
      --muted: #6d6256;
      --line: rgba(56, 41, 20, 0.14);
      --teal: #0f766e;
      --shadow: 0 24px 70px rgba(51, 33, 14, 0.14);
      --radius: 26px;
      --mono: "SFMono-Regular", "Cascadia Code", "JetBrains Mono", "Fira Code", monospace;
      --sans: "Avenir Next", "Segoe UI Variable", "PingFang SC", "Noto Sans", sans-serif;
    }}

    * {{
      box-sizing: border-box;
    }}

    html {{
      min-height: 100%;
      background:
        radial-gradient(circle at top left, rgba(200, 100, 40, 0.18), transparent 34%),
        radial-gradient(circle at right 12% top 18%, rgba(15, 118, 110, 0.16), transparent 28%),
        linear-gradient(180deg, var(--bg), var(--bg-strong));
    }}

    body {{
      margin: 0;
      min-height: 100vh;
      color: var(--text);
      font-family: var(--sans);
      line-height: 1.6;
      background-image:
        linear-gradient(rgba(56, 41, 20, 0.04) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56, 41, 20, 0.04) 1px, transparent 1px);
      background-size: 28px 28px;
    }}

    a {{
      color: inherit;
    }}

    .page {{
      width: min(1120px, calc(100% - 32px));
      margin: 0 auto;
      padding: 40px 0 56px;
    }}

    .hero,
    .section {{
      border: 1px solid var(--line);
      border-radius: calc(var(--radius) + 4px);
      background: var(--panel);
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }}

    .hero {{
      position: relative;
      overflow: hidden;
      padding: 32px;
    }}

    .hero::after {{
      content: "";
      position: absolute;
      inset: auto -8% -28% auto;
      width: 320px;
      aspect-ratio: 1;
      border-radius: 999px;
      background: radial-gradient(circle, rgba(200, 100, 40, 0.18), transparent 68%);
    }}

    .eyebrow {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
      margin: 0 0 14px;
      padding: 6px 12px;
      border-radius: 999px;
      background: rgba(15, 118, 110, 0.08);
      color: var(--teal);
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }}

    h1,
    h2,
    h3 {{
      margin: 0;
      line-height: 1.02;
      letter-spacing: -0.03em;
    }}

    h1 {{
      max-width: 10ch;
      font-size: clamp(40px, 8vw, 72px);
    }}

    .lead {{
      max-width: 700px;
      margin: 16px 0 0;
      color: var(--muted);
      font-size: clamp(16px, 2.2vw, 20px);
    }}

    .hero-actions,
    .meta-row,
    .card-actions {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
    }}

    .hero-actions {{
      margin-top: 24px;
    }}

    .button,
    .link-button {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-height: 44px;
      padding: 11px 16px;
      border: 1px solid var(--line);
      border-radius: 14px;
      text-decoration: none;
      transition: transform 120ms ease;
      white-space: nowrap;
    }}

    .link-button.small {{
      min-height: 36px;
      padding: 8px 12px;
      font-size: 14px;
    }}

    .button:hover,
    .link-button:hover,
    .button:focus-visible,
    .link-button:focus-visible {{
      transform: translateY(-1px);
    }}

    .button.primary {{
      background: #1b2423;
      border-color: #1b2423;
      color: #f8fbfa;
    }}

    .button.secondary,
    .link-button {{
      background: rgba(255, 255, 255, 0.58);
    }}

    .meta-row {{
      margin-top: 24px;
    }}

    .chip {{
      padding: 8px 12px;
      border: 1px solid var(--line);
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.52);
      font-size: 14px;
      color: var(--muted);
    }}

    .section {{
      margin-top: 22px;
      padding: 24px;
    }}

    .section-head {{
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      gap: 12px 20px;
      align-items: end;
      margin-bottom: 18px;
    }}

    .section-copy {{
      max-width: 640px;
      color: var(--muted);
    }}

    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 18px;
    }}

    .card {{
      display: flex;
      flex-direction: column;
      gap: 16px;
      min-width: 0;
      padding: 22px;
      border: 1px solid var(--line);
      border-radius: 22px;
      background: var(--panel-strong);
    }}

    .card-top {{
      display: flex;
      justify-content: space-between;
      align-items: start;
      gap: 14px;
    }}

    .card-head {{
      min-width: 0;
    }}

    .card h3 {{
      margin-top: 10px;
      font-size: clamp(28px, 4vw, 34px);
      word-break: break-word;
    }}

    .badge {{
      display: inline-flex;
      align-items: center;
      min-height: 32px;
      max-width: 100%;
      padding: 0 12px;
      border-radius: 999px;
      background: rgba(15, 118, 110, 0.08);
      color: var(--teal);
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.05em;
    }}

    .version {{
      color: var(--muted);
      font-size: 15px;
      white-space: nowrap;
    }}

    .card-copy {{
      margin: 0;
      color: var(--muted);
    }}

    .path {{
      display: block;
      max-width: 100%;
      padding: 12px 14px;
      overflow-x: auto;
      border: 1px solid rgba(15, 118, 110, 0.16);
      border-radius: 16px;
      background: #172120;
      color: #eafaf7;
      font-size: 14px;
      font-family: var(--mono);
      white-space: nowrap;
    }}

    .maintain-grid {{
      display: grid;
      grid-template-columns: 1.05fr 0.95fr;
      gap: 18px;
    }}

    .steps,
    .sample {{
      padding: 20px;
      border: 1px solid var(--line);
      border-radius: 22px;
      background: var(--panel-strong);
    }}

    ol {{
      margin: 0;
      padding-left: 20px;
      color: var(--muted);
    }}

    li + li {{
      margin-top: 10px;
    }}

    pre {{
      margin: 0;
      padding: 16px;
      overflow-x: auto;
      border: 1px solid rgba(15, 118, 110, 0.16);
      border-radius: 16px;
      background: #172120;
      color: #eafaf7;
      font-size: 14px;
      line-height: 1.55;
      font-family: var(--mono);
    }}

    .footnote {{
      margin-top: 16px;
      color: var(--muted);
      font-size: 14px;
    }}

    @media (max-width: 860px) {{
      .page {{
        width: min(100% - 20px, 760px);
        padding-top: 20px;
        padding-bottom: 28px;
      }}

      .hero,
      .section {{
        padding: 20px;
      }}

      .maintain-grid {{
        grid-template-columns: 1fr;
      }}

      .card-top {{
        flex-direction: column;
        align-items: start;
      }}
    }}
  </style>
</head>
<body>
  <main class="page">
    <section class="hero">
      <p class="eyebrow">静态 Wheel 仓库</p>
      <h1>pip.wgists.me</h1>
      <p class="lead">
        用于托管和分发 Python wheel 文件，并提供对应的使用文档入口。
      </p>
      <div class="hero-actions">
        <a class="button primary" href="./dist/">打开 dist/</a>
        <a class="button secondary" href="./dist/index.html">简洁索引页</a>
      </div>
      <div class="meta-row">
        <span class="chip">{count} 个包</span>
        <span class="chip">{matched_doc_count} 个包带文档</span>
        <span class="chip">GitHub Pages</span>
        <span class="chip">自动扫描生成</span>
      </div>
    </section>

    <section class="section">
      <div class="section-head">
        <div>
          <h2>包列表</h2>
          <p class="section-copy">
            点击可进入包目录、直接下载 wheel，或打开对应的使用文档。
          </p>
        </div>
      </div>

      <div class="grid">
{cards}
      </div>
    </section>

    <section class="section">
      <div class="section-head">
        <div>
          <h2>如何新增 Wheel</h2>
          <p class="section-copy">
            只要按目录约定放文件，再运行一次脚本，首页和索引页都会自动更新。
          </p>
        </div>
      </div>

      <div class="maintain-grid">
        <div class="steps">
          <ol>
            <li>创建目录：<code>dist/new_pkg/</code></li>
            <li>把 wheel 文件放进这个目录里。</li>
            <li>如果要显示文档按钮，文档文件命名为：<code>docs/new_pkg.md</code></li>
            <li>运行：<code>python scripts/generate_indexes.py</code></li>
            <li>提交生成后的 <code>index.html</code>、<code>docs/*.html</code> 和 <code>dist/*/index.html</code></li>
          </ol>
          <p class="footnote">
            不要手工复制卡片。以后只维护 <code>dist/</code> 和 <code>docs/</code> 目录内容即可。
          </p>
        </div>

        <div class="sample">
          <pre><code>dist/
  new_pkg/
    new_pkg-0.1.0-py3-none-any.whl

docs/
  new_pkg.md

python scripts/generate_indexes.py</code></pre>
        </div>
      </div>
    </section>
  </main>
</body>
</html>"""


def render_card(package: Package, document: Document | None) -> str:
    name = escape(package.name)
    wheel_path = f"dist/{package.name}/{package.latest.name}"
    description = (
        f"目录中共有 {len(package.wheels)} 个 wheel 文件，当前默认展示最新文件。"
        if len(package.wheels) > 1
        else "目录中当前有 1 个 wheel 文件。"
    )
    version = f"v{escape(package.version)}" if package.version else "wheel"
    doc_link = ""
    if document is not None:
        doc_link = (
            f'\n            <a class="link-button small" href="./docs/{escape(document.stem)}.html">'
            "使用文档</a>"
        )
    return f"""        <article class="card">
          <div class="card-top">
            <div class="card-head">
              <span class="badge">{name}</span>
              <h3>{name}</h3>
            </div>
            <span class="version">{version}</span>
          </div>
          <p class="card-copy">{escape(description)}</p>
          <code class="path">{escape(wheel_path)}</code>
          <div class="card-actions">
            <a class="link-button" href="./dist/{name}/">打开包目录</a>
            <a class="link-button" href="./{escape(wheel_path)}">下载最新 wheel</a>{doc_link}
          </div>
        </article>"""


def render_dist_index(packages: Iterable[Package]) -> str:
    lines: list[str] = []
    for package in packages:
        for wheel in package.wheels:
            rel = f"./{package.name}/{wheel.name}"
            lines.append(f'<a href="{escape(rel)}">{escape(wheel.name)}</a>')
    return "\n".join(lines)


def render_package_index(package: Package) -> str:
    links = "\n".join(
        f'<a href="./{escape(wheel.name)}">{escape(wheel.name)}</a><br>'
        for wheel in package.wheels
    )
    return links


def extract_title(source: str, default: str) -> str:
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith("# "):
            return stripped[2:].strip()
    return default


def extract_summary(source: str, title: str) -> str:
    in_code = False
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith("```"):
            in_code = not in_code
            continue
        if in_code or not stripped:
            continue
        if stripped.startswith("#"):
            continue
        if stripped.startswith("- ") or stripped.startswith("+ ") or re.match(r"^\d+\.\s+", stripped):
            continue
        if stripped == title:
            continue
        return stripped
    return "查看安装方式、运行示例和参数说明。"


def render_docs_index(documents: Iterable[Document]) -> str:
    links = "\n".join(
        f'<a href="./{escape(document.stem)}.html">{escape(document.title)}</a><br>'
        for document in documents
    )
    return links


def render_doc_page(document: Document) -> str:
    title = escape(document.title)
    body = render_markdown(document.source)
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} | pip.wgists.me</title>
  <style>
    :root {{
      --bg: #f5efe3;
      --panel: rgba(255, 251, 245, 0.9);
      --panel-strong: #fffaf2;
      --text: #201a13;
      --muted: #6d6256;
      --line: rgba(56, 41, 20, 0.14);
      --teal: #0f766e;
      --shadow: 0 24px 70px rgba(51, 33, 14, 0.14);
      --radius: 24px;
      --mono: "SFMono-Regular", "Cascadia Code", "JetBrains Mono", "Fira Code", monospace;
      --sans: "Avenir Next", "Segoe UI Variable", "PingFang SC", "Noto Sans", sans-serif;
    }}

    * {{
      box-sizing: border-box;
    }}

    body {{
      margin: 0;
      min-height: 100vh;
      color: var(--text);
      font-family: var(--sans);
      line-height: 1.7;
      background:
        radial-gradient(circle at top left, rgba(200, 100, 40, 0.18), transparent 34%),
        radial-gradient(circle at right 12% top 18%, rgba(15, 118, 110, 0.16), transparent 28%),
        linear-gradient(180deg, #f5efe3, #efe4d1);
    }}

    a {{
      color: #0c645d;
    }}

    .page {{
      width: min(920px, calc(100% - 24px));
      margin: 0 auto;
      padding: 24px 0 40px;
    }}

    .topbar,
    .content {{
      border: 1px solid var(--line);
      border-radius: var(--radius);
      background: var(--panel);
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }}

    .topbar {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
      padding: 18px 20px;
      margin-bottom: 18px;
    }}

    .crumbs {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      color: var(--muted);
    }}

    .button {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-height: 42px;
      padding: 10px 14px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: var(--panel-strong);
      color: inherit;
      text-decoration: none;
      white-space: nowrap;
    }}

    .content {{
      padding: 28px 24px;
    }}

    .content h1,
    .content h2,
    .content h3 {{
      margin: 0 0 14px;
      line-height: 1.1;
      letter-spacing: -0.03em;
    }}

    .content h1 {{
      font-size: clamp(34px, 6vw, 54px);
    }}

    .content h2 {{
      margin-top: 28px;
      font-size: 28px;
    }}

    .content h3 {{
      margin-top: 24px;
      font-size: 22px;
    }}

    .content p,
    .content li {{
      color: var(--text);
    }}

    .content ul,
    .content ol {{
      padding-left: 22px;
    }}

    .content code {{
      padding: 2px 6px;
      border-radius: 8px;
      background: rgba(15, 118, 110, 0.08);
      font-family: var(--mono);
      font-size: 0.95em;
    }}

    .content pre {{
      overflow-x: auto;
      padding: 16px;
      border: 1px solid rgba(15, 118, 110, 0.16);
      border-radius: 18px;
      background: #172120;
      color: #eafaf7;
    }}

    .content pre code {{
      padding: 0;
      background: transparent;
      color: inherit;
    }}

    @media (max-width: 720px) {{
      .content {{
        padding: 20px 16px;
      }}

      .topbar {{
        padding: 14px 16px;
      }}
    }}
  </style>
</head>
<body>
  <main class="page">
    <div class="topbar">
      <div class="crumbs">
        <a href="../index.html">首页</a>
        <span>/</span>
        <a href="./index.html">使用文档</a>
        <span>/</span>
        <span>{title}</span>
      </div>
      <a class="button" href="./{escape(document.path.name)}">查看 Markdown 源文档</a>
    </div>
    <article class="content">
{body}
    </article>
  </main>
</body>
</html>"""


def render_markdown(source: str) -> str:
    blocks: list[str] = []
    paragraph: list[str] = []
    unordered: list[str] = []
    ordered: list[str] = []
    in_code = False
    code_lang = ""
    code_lines: list[str] = []

    def flush_paragraph() -> None:
        nonlocal paragraph
        if paragraph:
            blocks.append(f"      <p>{render_inline(' '.join(paragraph))}</p>")
            paragraph = []

    def flush_unordered() -> None:
        nonlocal unordered
        if unordered:
            items = "\n".join(f"        <li>{render_inline(item)}</li>" for item in unordered)
            blocks.append(f"      <ul>\n{items}\n      </ul>")
            unordered = []

    def flush_ordered() -> None:
        nonlocal ordered
        if ordered:
            items = "\n".join(f"        <li>{render_inline(item)}</li>" for item in ordered)
            blocks.append(f"      <ol>\n{items}\n      </ol>")
            ordered = []

    def flush_code() -> None:
        nonlocal code_lines, code_lang
        if code_lines:
            class_attr = f' class="language-{escape(code_lang)}"' if code_lang else ""
            code = escape("\n".join(code_lines))
            blocks.append(f"      <pre><code{class_attr}>{code}</code></pre>")
            code_lines = []
            code_lang = ""

    for raw_line in source.splitlines():
        stripped = raw_line.strip()

        if stripped.startswith("```"):
            if in_code:
                flush_code()
                in_code = False
            else:
                flush_paragraph()
                flush_unordered()
                flush_ordered()
                in_code = True
                code_lang = stripped[3:].strip()
            continue

        if in_code:
            code_lines.append(raw_line)
            continue

        if not stripped:
            flush_paragraph()
            flush_unordered()
            flush_ordered()
            continue

        heading = re.match(r"^(#{1,3})\s+(.*)$", stripped)
        if heading:
            flush_paragraph()
            flush_unordered()
            flush_ordered()
            level = len(heading.group(1))
            text = render_inline(heading.group(2).strip())
            blocks.append(f"      <h{level}>{text}</h{level}>")
            continue

        unordered_item = re.match(r"^[-+*]\s+(.*)$", stripped)
        if unordered_item:
            flush_paragraph()
            flush_ordered()
            unordered.append(unordered_item.group(1).strip())
            continue

        ordered_item = re.match(r"^\d+\.\s+(.*)$", stripped)
        if ordered_item:
            flush_paragraph()
            flush_unordered()
            ordered.append(ordered_item.group(1).strip())
            continue

        paragraph.append(stripped)

    flush_paragraph()
    flush_unordered()
    flush_ordered()
    flush_code()
    return "\n".join(blocks)


def render_inline(text: str) -> str:
    placeholders: dict[str, str] = {}
    counter = 0

    def stash(html: str) -> str:
        nonlocal counter
        token = f"ZZPH{counter}ZZ"
        counter += 1
        placeholders[token] = html
        return token

    def replace_link(match: re.Match[str]) -> str:
        label = escape(match.group(1))
        url = match.group(2).strip()
        if url.startswith("/home/"):
            return stash(f"<code>{label}</code>")
        safe_url = escape(url, quote=True)
        return stash(f'<a href="{safe_url}">{label}</a>')

    def replace_code(match: re.Match[str]) -> str:
        return stash(f"<code>{escape(match.group(1))}</code>")

    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", replace_link, text)
    text = re.sub(r"`([^`]+)`", replace_code, text)
    text = escape(text)
    for token, html in placeholders.items():
        text = text.replace(token, html)
    return text


def main() -> None:
    packages = iter_packages()
    documents = iter_documents()
    write_text(ROOT / "index.html", render_root_index(packages, documents))
    write_text(DIST / "index.html", render_dist_index(packages))
    for package in packages:
        write_text(package.folder / "index.html", render_package_index(package))
    write_text(DOCS / "index.html", render_docs_index(documents))
    for document in documents:
        write_text(DOCS / f"{document.stem}.html", render_doc_page(document))


if __name__ == "__main__":
    main()
