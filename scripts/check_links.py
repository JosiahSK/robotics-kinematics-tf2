#!/usr/bin/env python3
"""
Verify all local Markdown links, images, and file references in repository.
"""

import os
import glob
import re

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
md_files = glob.glob(os.path.join(repo_root, "**", "*.md"), recursive=True)
md_files = [f for f in md_files if ".pytest_cache" not in f and ".git" not in f]

link_regex = re.compile(r'!?\[.*?\]\(([^)]+)\)')
broken_links = []
total_links = 0

for md_file in md_files:
    dir_name = os.path.dirname(md_file)
    with open(md_file, "r", encoding="utf-8") as f:
        content = f.read()

    matches = link_regex.findall(content)
    for target in matches:
        if target.startswith("http://") or target.startswith("https://") or target.startswith("#") or target.startswith("mailto:"):
            continue

        clean_target = target.split("#")[0]
        if not clean_target:
            continue

        full_path = os.path.normpath(os.path.join(dir_name, clean_target))
        total_links += 1
        if not os.path.exists(full_path):
            broken_links.append((os.path.relpath(md_file, repo_root), target, os.path.relpath(full_path, repo_root)))

print(f"Scanned {len(md_files)} markdown files with {total_links} local links.")
if broken_links:
    print(f"FOUND {len(broken_links)} BROKEN LINKS:")
    for src, tgt, resolved in broken_links:
        print(f"  In {src}: '{tgt}' -> {resolved}")
    exit(1)
else:
    print("SUCCESS: All local markdown links and image paths are valid!")
