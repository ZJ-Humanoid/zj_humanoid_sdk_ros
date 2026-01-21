#!/usr/bin/env node

/**
 * 从 Git 获取版本列表（用于动态配置）
 * 
 * 输出格式: JSON 数组，例如: ["v1.0", "v2.0", "main"]
 */

import { execSync } from 'child_process';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const docsRoot = path.resolve(__dirname, '../..');
const versionsDir = path.join(docsRoot, 'versions');

function getGitTags() {
  try {
    const tags = execSync('git tag -l', { encoding: 'utf-8', cwd: docsRoot })
      .trim()
      .split('\n')
      .filter(tag => tag.trim() !== '')
      .filter(tag => /^v?\d+\.\d+/.test(tag))
      .map(tag => tag.startsWith('v') ? tag : `v${tag}`)
      .sort((a, b) => {
        // 语义化版本排序
        const aParts = a.replace(/^v/, '').split('.').map(Number);
        const bParts = b.replace(/^v/, '').split('.').map(Number);
        for (let i = 0; i < Math.max(aParts.length, bParts.length); i++) {
          const aPart = aParts[i] || 0;
          const bPart = bParts[i] || 0;
          if (aPart !== bPart) {
            return bPart - aPart; // 降序排列（最新版本在前）
          }
        }
        return 0;
      });
    return tags;
  } catch (error) {
    return [];
  }
}

function getLocalVersions() {
  if (!fs.existsSync(versionsDir)) {
    return [];
  }
  
  return fs.readdirSync(versionsDir, { withFileTypes: true })
    .filter(entry => entry.isDirectory())
    .map(entry => entry.name)
    .filter(name => /^\d+\.\d+/.test(name))
    .map(name => name.startsWith('v') ? name : `v${name}`)
    .sort((a, b) => {
      const aParts = a.replace(/^v/, '').split('.').map(Number);
      const bParts = b.replace(/^v/, '').split('.').map(Number);
      for (let i = 0; i < Math.max(aParts.length, bParts.length); i++) {
        const aPart = aParts[i] || 0;
        const bPart = bParts[i] || 0;
        if (aPart !== bPart) {
          return bPart - aPart;
        }
      }
      return 0;
    });
}

// 合并 Git 标签和本地版本，去重
const gitTags = getGitTags();
const localVersions = getLocalVersions();
const allVersions = [...new Set([...gitTags, ...localVersions])]
  .sort((a, b) => {
    const aParts = a.replace(/^v/, '').split('.').map(Number);
    const bParts = b.replace(/^v/, '').split('.').map(Number);
    for (let i = 0; i < Math.max(aParts.length, bParts.length); i++) {
      const aPart = aParts[i] || 0;
      const bPart = bParts[i] || 0;
      if (aPart !== bPart) {
        return bPart - aPart;
      }
    }
    return 0;
  });

// 输出 JSON
console.log(JSON.stringify(allVersions, null, 2));
