// Generates a VitePress sidebar from a GitBook-style SUMMARY.md file.
// Adapted from the PX4 user guide (px4_user_guide/.vitepress/get_sidebar.js).
// Each locale (zh, en) has its own <lang>/SUMMARY.md whose nested markdown
// list is parsed into the { text, link, items, collapsed } tree VitePress wants.

import path from "path";
import fs from "fs";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function getEntryArray(parent) {
  if (Array.isArray(parent)) return parent;
  parent.items = parent?.items ? parent.items : [];
  return parent.items;
}

function parseSummary(sidebarContent, lang) {
  const lines = sidebarContent.split("\n");
  const parents = [];
  const topLevelParent = { title: "DUMMY", path: "DUMMY", items: [] };
  parents.push(topLevelParent); // last item is always the sidebar root

  let current_parent;
  let lastlevel = 0;
  let indent_divider = 0;

  lines.forEach((line) => {
    if (line.startsWith("#") || line.trim() === "") return;

    const regex = /(\s*?)[\*-]\s\[(.*?)\]\((.*?)\)/g;
    let indent_level, link_title, link_url;
    try {
      const match = regex.exec(line);
      indent_level = match[1].length;
      link_title = match[2];
      link_url = match[3].trim();
    } catch (err) {
      return; // skip lines that don't match
    }

    // .md -> clean URL, prefix locale for internal links
    if (link_url.endsWith(".md")) link_url = link_url.replace(".md", "");
    if (!link_url.startsWith("http")) {
      link_url = lang ? `/${lang}/${link_url}` : `/${link_url}`;
    }
    // index -> directory root
    link_url = link_url.replace(/\/index$/, "/");

    // Un-escape GitBook title escaping
    link_title = link_title
      .replace("\\(", "(")
      .replace("\\)", ")")
      .replace("\\_", "_");

    // normalise indentation into integer levels
    if (indent_divider === 0 && indent_level > 0) indent_divider = indent_level;
    if (indent_divider > 0) indent_level = indent_level / indent_divider;

    const entry = { text: link_title, link: link_url };

    current_parent = parents.pop();

    if (indent_level === lastlevel) {
      getEntryArray(current_parent).push(entry);
      parents.push(current_parent);
    } else if (indent_level > lastlevel) {
      const parentArray = getEntryArray(current_parent);
      const lastElement = parentArray.pop();
      lastElement.collapsed = true;
      getEntryArray(lastElement).push(entry);
      parentArray.push(lastElement);
      parents.push(current_parent);
      parents.push(lastElement);
    } else {
      while (indent_level < lastlevel--) {
        current_parent = parents.pop();
      }
      getEntryArray(current_parent).push(entry);
      parents.push(current_parent);
    }

    lastlevel = indent_level;
  });

  return topLevelParent.items;
}

export function sidebar(lang) {
  const summaryPath = path.resolve(__dirname, "..", lang, "SUMMARY.md");
  let data = "";
  try {
    data = fs.readFileSync(summaryPath, "UTF-8");
  } catch (err) {
    console.log(`DEBUG: ${lang} - SUMMARY.md NOT FOUND at ${summaryPath}`);
  }
  return parseSummary(data, lang);
}

export default { sidebar };
