#!/usr/bin/env node
/**
 * 修复 @viteplus/versions 库生成的版本 locale 下 sidebar key 与
 * VitePress page.relativePath 不匹配的问题。
 *
 * 根因：
 *   - 库把 sidebar key 设置为裸版本号 "1.2.0"
 *   - VitePress 使用 ensureStartingSlash(relativePath).startsWith(ensureStartingSlash(key))
 *   - relativePath = "versions/1.2.0/api/hand.md" -> "/versions/1.2.0/api/hand.md"
 *   - key "1.2.0" -> "/1.2.0"  匹配失败，侧边栏为空
 *
 * 修复：遍历构建产物中的所有 HTML，为版本 locale sidebar 注入
 *       "/versions/<v>" 和 "/versions/<v>/" 两个 key。
 *
 * 用法：
 *   node docs/.vitepress/scripts/fix-version-sidebar.js <dist-dir>
 *   （默认 <dist-dir> = ./dist）
 */
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '../../../');
const docsDir = path.resolve(projectRoot, 'docs');

const distDir = process.argv[2]
  ? path.resolve(process.argv[2])
  : path.resolve(projectRoot, 'dist');

// 收集版本号
function collectVersions() {
  const versionsRoot = path.resolve(docsDir, 'versions');
  if (!fs.existsSync(versionsRoot)) return [];
  return fs
    .readdirSync(versionsRoot, { withFileTypes: true })
    .filter((e) => e.isDirectory())
    .map((e) => e.name)
    .filter((n) => !n.startsWith('.'));
}

function patchHtml(filePath, versions) {
  const html = fs.readFileSync(filePath, 'utf-8');
  if (!html.includes('__VP_SITE_DATA__')) return 0;

  const pattern = /window\.__VP_SITE_DATA__=JSON\.parse\("(.*?)"\)/;
  const m = html.match(pattern);
  if (!m) return 0;

  const innerEscaped = m[1];
  const jsonStr = JSON.parse('"' + innerEscaped + '"');
  const siteData = JSON.parse(jsonStr);
  const locales = siteData.locales;
  if (!locales) return 0;

  let modified = false;
  for (const v of versions) {
    const locale = locales[v];
    const sb = locale?.themeConfig?.sidebar;
    if (!sb || !(v in sb)) continue;
    const value = sb[v];
    for (const k of [`/versions/${v}`, `/versions/${v}/`]) {
      if (!(k in sb)) {
        sb[k] = value;
        modified = true;
      }
    }
  }
  if (!modified) return 0;

  const newJsonStr = JSON.stringify(siteData);
  // 把 JSON 字符串再转义为 JS 字符串字面量（JSON.stringify(string) 恰好做了这件事）
  const newInnerEscaped = JSON.stringify(newJsonStr).slice(1, -1);
  const newHtml = html.replace(
    pattern,
    'window.__VP_SITE_DATA__=JSON.parse("' + newInnerEscaped + '")'
  );
  fs.writeFileSync(filePath, newHtml, 'utf-8');
  return 1;
}

function walk(dir, list = []) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) walk(full, list);
    else if (entry.isFile() && entry.name.endsWith('.html')) list.push(full);
  }
  return list;
}

// === main ===
const versions = collectVersions();
if (versions.length === 0) {
  console.log('[fix-version-sidebar] no versions found, skip.');
  process.exit(0);
}
if (!fs.existsSync(distDir)) {
  console.error('[fix-version-sidebar] dist dir not found:', distDir);
  process.exit(1);
}

const htmls = walk(distDir);
let patched = 0;
for (const f of htmls) {
  try {
    patched += patchHtml(f, versions);
  } catch (e) {
    console.warn(`[fix-version-sidebar] fail on ${f}:`, e.message);
  }
}
console.log(`[fix-version-sidebar] patched ${patched}/${htmls.length} html files (versions=${versions.join(',')}).`);
