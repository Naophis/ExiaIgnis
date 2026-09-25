#!/usr/bin/env python3
"""ExiaIgnis 技術資料の静的 HTML を生成する。

chapters/NN-slug.html (本文フラグメント) と style.css から、
  - 各章のページ (index.html, 01-hardware.html, ...)
  - 全章を 1 ページにまとめた all.html
を同じディレクトリに書き出す。--artifact <path> を付けると、
claude.ai Artifact 向けの本文のみ版 (doctype/html/head/body 無し) も出す。

使い方:
    python3 docs/techdoc/build.py
    python3 docs/techdoc/build.py --artifact /tmp/exiaignis_techdoc_artifact.html
"""
import argparse
import datetime
import html
import pathlib
import re
import subprocess

HERE = pathlib.Path(__file__).resolve().parent
CHAPTERS = [
    # (fragment, output, number label, title, short label)
    ("00-index.html", "index.html", "", "ExiaIgnis 技術資料", "概要"),
    ("01-hardware.html", "01-hardware.html", "1", "機体構成", "機体構成"),
    ("02-control.html", "02-control.html", "2", "制御", "制御"),
    ("03-software.html", "03-software.html", "3", "ソフトウェア構成", "ソフトウェア"),
    ("04-search.html", "04-search.html", "4", "探索・経路生成", "探索・経路"),
    ("05-tools.html", "05-tools.html", "5", "開発環境・ツール", "ツール"),
    ("06-tuning.html", "06-tuning.html", "6", "調整手順", "調整手順"),
    ("07-devlog.html", "07-devlog.html", "7", "開発記録・知見(2026 シーズン)", "開発記録"),
    ("08-performance.html", "08-performance.html", "8", "性能と今後の課題", "性能・課題"),
    ("09-appendix.html", "09-appendix.html", "付録", "付録", "付録"),
]

FONTS = ("https://fonts.googleapis.com/css2?family=Zen+Kaku+Gothic+New:wght@500;700;900"
         "&family=BIZ+UDPGothic:wght@400;700&family=IBM+Plex+Mono:wght@400;500&display=swap")


def git_stamp():
    try:
        rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=HERE,
                                      stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        rev = "unknown"
    return rev


def load_fragment(name):
    return (HERE / "chapters" / name).read_text(encoding="utf-8")


def toc_of(fragment):
    """h2 の id/テキストを拾ってページ内目次にする"""
    items = []
    for m in re.finditer(r'<h2[^>]*\bid="([^"]+)"[^>]*>(.*?)</h2>', fragment, re.S):
        text = re.sub(r"<[^>]+>", "", m.group(2)).strip()
        items.append((m.group(1), text))
    return items


def nav_html(current_out, single=False):
    lis = []
    for frag, out, num, title, short in CHAPTERS:
        href = f"#ch-{out.rsplit('.', 1)[0]}" if single else out
        cls = ' class="is-current"' if (out == current_out and not single) else ""
        label = f'<span class="nav-num">{html.escape(num)}</span>' if num else ""
        lis.append(f'<li{cls}><a href="{href}">{label}<span>{html.escape(short)}</span></a></li>')
    return "\n".join(lis)


def page_shell(title, body, nav, toc, stamp, prev_next, single=False, inline_css=None):
    css = (f"<style>\n{inline_css}\n</style>" if inline_css is not None
           else '<link rel="stylesheet" href="style.css">')
    toc_html = ""
    if toc:
        toc_html = '<nav class="toc" aria-label="このページの目次"><p class="toc-title">このページ</p><ol>' + "".join(
            f'<li><a href="#{html.escape(i)}">{html.escape(t)}</a></li>' for i, t in toc) + "</ol></nav>"
    return f"""<title>{html.escape(title)}</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="{FONTS}">
{css}
<div class="site">
<header class="site-head">
  <a class="brand" href="{'#top' if single else 'index.html'}"><span class="brand-name">ExiaIgnis</span><span class="brand-sub">技術資料</span></a>
  <nav class="chapters" aria-label="章"><ul>{nav}</ul></nav>
</header>
<div class="layout">
  <aside class="side">{toc_html}</aside>
  <main class="content" id="top">
{body}
  <footer class="page-foot">
    {prev_next}
    <p class="stamp">生成: {stamp}</p>
  </footer>
  </main>
</div>
</div>
"""


def full_doc(inner, lang="ja"):
    return ("<!doctype html>\n<html lang=\"%s\">\n<head>\n<meta charset=\"utf-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n"
            "</head>\n<body>\n%s\n</body>\n</html>\n") % (lang, inner)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--artifact", help="Artifact 用の本文のみ HTML の出力先")
    args = ap.parse_args()

    css = (HERE / "style.css").read_text(encoding="utf-8")
    stamp = f"{datetime.date.today().isoformat()} ({git_stamp()})"

    all_bodies = []
    all_tocs = []
    for i, (frag, out, num, title, short) in enumerate(CHAPTERS):
        body = load_fragment(frag)
        toc = toc_of(body)
        prev_link = next_link = ""
        if i > 0:
            p = CHAPTERS[i - 1]
            prev_link = f'<a class="pn prev" href="{p[1]}"><small>前の章</small>{html.escape((p[2] + ". " if p[2] else "") + p[3])}</a>'
        if i < len(CHAPTERS) - 1:
            n = CHAPTERS[i + 1]
            next_link = f'<a class="pn next" href="{n[1]}"><small>次の章</small>{html.escape((n[2] + ". " if n[2] else "") + n[3])}</a>'
        prev_next = f'<nav class="prevnext" aria-label="前後の章">{prev_link}{next_link}</nav>'
        page_title = title if not num else f"{num}. {title}"
        inner = page_shell(f"{page_title} | ExiaIgnis 技術資料" if num else title,
                           body, nav_html(out), toc, stamp, prev_next)
        (HERE / out).write_text(full_doc(inner), encoding="utf-8")
        anchor = f"ch-{out.rsplit('.', 1)[0]}"
        all_bodies.append(f'<section class="chapter-block" id="{anchor}">\n{body}\n</section>')
        all_tocs.append((anchor, (num + ". " if num else "") + title))

    # 全章 1 ページ版 (リポジトリ内で完結する版)
    single_inner = page_shell("ExiaIgnis 技術資料", "\n<hr class=\"chapter-sep\">\n".join(all_bodies),
                              nav_html("", single=True), all_tocs, stamp, "", single=True)
    (HERE / "all.html").write_text(full_doc(single_inner), encoding="utf-8")

    if args.artifact:
        art = page_shell("ExiaIgnis 技術資料", "\n<hr class=\"chapter-sep\">\n".join(all_bodies),
                         nav_html("", single=True), all_tocs, stamp, "", single=True, inline_css=css)
        pathlib.Path(args.artifact).write_text(art, encoding="utf-8")
    print(f"built {len(CHAPTERS)} pages + all.html ({stamp})")


if __name__ == "__main__":
    main()
