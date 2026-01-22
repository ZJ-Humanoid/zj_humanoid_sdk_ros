#!/usr/bin/env node

/**
 * 从 Git 标签同步版本文档
 * 
 * 用法:
 *   node sync-versions-from-git.js [--tag <tag-name>] [--branch <branch-name>]
 * 
 * 功能:
 *   1. 读取所有 Git 标签
 *   2. 为每个标签创建对应的版本目录
 *   3. 从标签对应的提交中提取文档内容
 */

import { execSync } from 'child_process';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { dirname } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const docsDir = path.resolve(__dirname, '../..'); // docs 目录
const projectRoot = path.resolve(docsDir, '..'); // 项目根目录
const versionsDir = path.join(docsDir, 'versions');
const srcDir = path.join(docsDir, 'src');

// 确保 versions 目录存在
if (!fs.existsSync(versionsDir)) {
  fs.mkdirSync(versionsDir, { recursive: true });
}

/**
 * 获取所有 Git 标签（按版本号排序）
 */
function getGitTags() {
  try {
    const tags = execSync('git tag -l', { encoding: 'utf-8', cwd: projectRoot })
      .trim()
      .split('\n')
      .filter(tag => tag.trim() !== '')
      .filter(tag => /^v?\d+\.\d+/.test(tag)) // 只处理版本号格式的标签
      .sort((a, b) => {
        // 简单的版本号排序（可以改进为语义化版本排序）
        return a.localeCompare(b, undefined, { numeric: true, sensitivity: 'base' });
      });
    return tags;
  } catch (error) {
    console.warn('无法获取 Git 标签:', error.message);
    return [];
  }
}

/**
 * 从指定标签或分支检出文档
 */
function checkoutDocsFromRef(ref, versionDir) {
  try {
    // 检查标签/分支是否存在
    execSync(`git rev-parse --verify ${ref}`, { 
      encoding: 'utf-8', 
      cwd: projectRoot,
      stdio: 'pipe'
    });
    
    // 临时检出该版本的 docs/src 目录
    const tempDir = path.join(versionsDir, `.temp-${ref}`);
    
    try {
      // 使用 git show 提取特定目录的内容
      const docsPath = 'docs/src';
      const files = execSync(`git ls-tree -r --name-only ${ref} -- ${docsPath}`, {
        encoding: 'utf-8',
        cwd: projectRoot
      }).trim().split('\n').filter(f => f);
      
      if (files.length === 0) {
        console.warn(`标签 ${ref} 中没有找到 docs/src 目录`);
        return false;
      }
      
      // 创建版本目录
      if (fs.existsSync(versionDir)) {
        fs.rmSync(versionDir, { recursive: true, force: true });
      }
      fs.mkdirSync(versionDir, { recursive: true });
      
      // 提取每个文件
      for (const file of files) {
        try {
          const content = execSync(`git show ${ref}:${file}`, {
            encoding: 'utf-8',
            cwd: projectRoot,
            stdio: 'pipe'
          });
          
          const relativePath = path.relative(docsPath, file);
          const targetPath = path.join(versionDir, relativePath);
          const targetDir = path.dirname(targetPath);
          
          if (!fs.existsSync(targetDir)) {
            fs.mkdirSync(targetDir, { recursive: true });
          }
          
          fs.writeFileSync(targetPath, content, 'utf-8');
        } catch (fileError) {
          console.warn(`无法提取文件 ${file}:`, fileError.message);
        }
      }
      
      console.log(`✓ 成功从 ${ref} 创建版本目录: ${versionDir}`);
      return true;
    } catch (error) {
      console.error(`从 ${ref} 提取文档失败:`, error.message);
      return false;
    }
  } catch (error) {
    console.warn(`标签/分支 ${ref} 不存在，跳过`);
    return false;
  }
}

/**
 * 同步所有标签版本
 * @param {boolean} force - 是否强制同步（即使目录已存在）
 */
function syncAllVersions(force = false) {
  const tags = getGitTags();
  console.log(`找到 ${tags.length} 个 Git 标签:`, tags);
  
  for (const tag of tags) {
    // 规范化版本名称（移除 'v' 前缀如果存在）
    const versionName = tag.startsWith('v') ? tag.substring(1) : tag;
    const versionDir = path.join(versionsDir, versionName);
    
    // 如果版本目录已存在且不强制同步，跳过
    if (fs.existsSync(versionDir) && !force) {
      console.log(`版本 ${versionName} 已存在，跳过（使用 --force 强制同步）`);
      continue;
    }
    
    checkoutDocsFromRef(tag, versionDir);
  }
}

/**
 * 从指定分支创建版本（用于开发分支）
 * @param {string} branchName - 分支名称（可以是 'develop' 或 'origin/develop'）
 */
function syncFromBranch(branchName) {
  const versionName = branchName.replace(/^v/, '').replace(/\//g, '-').replace(/^origin-/, '');
  const versionDir = path.join(versionsDir, versionName);
  
  console.log(`从分支 ${branchName} 创建版本: ${versionName}`);
  
  // 尝试从远程分支提取文档（如果分支名不包含 origin/，尝试添加）
  let refName = branchName;
  if (!branchName.includes('/') && !branchName.startsWith('origin/')) {
    // 尝试 origin/branchName
    try {
      execSync(`git rev-parse --verify origin/${branchName}`, { 
        encoding: 'utf-8', 
        cwd: projectRoot,
        stdio: 'pipe'
      });
      refName = `origin/${branchName}`;
      console.log(`使用远程分支: ${refName}`);
    } catch (e) {
      // 如果远程分支不存在，尝试本地分支
      try {
        execSync(`git rev-parse --verify ${branchName}`, { 
          encoding: 'utf-8', 
          cwd: projectRoot,
          stdio: 'pipe'
        });
        refName = branchName;
        console.log(`使用本地分支: ${refName}`);
      } catch (e2) {
        console.error(`分支 ${branchName} 不存在（本地和远程）`);
        return;
      }
    }
  }
  
  // 使用 checkoutDocsFromRef 从分支提取文档
  const success = checkoutDocsFromRef(refName, versionDir);
  
  if (!success) {
    console.warn(`从分支 ${refName} 提取文档失败，尝试使用当前工作目录的 docs/src`);
    // 如果从分支提取失败，尝试使用当前工作目录（适用于 develop 分支推送时）
    if (fs.existsSync(srcDir)) {
      if (fs.existsSync(versionDir)) {
        fs.rmSync(versionDir, { recursive: true, force: true });
      }
      
      // 递归复制目录
      function copyDir(src, dest) {
        if (!fs.existsSync(dest)) {
          fs.mkdirSync(dest, { recursive: true });
        }
        
        const entries = fs.readdirSync(src, { withFileTypes: true });
        for (const entry of entries) {
          const srcPath = path.join(src, entry.name);
          const destPath = path.join(dest, entry.name);
          
          if (entry.isDirectory()) {
            copyDir(srcPath, destPath);
          } else {
            fs.copyFileSync(srcPath, destPath);
          }
        }
      }
      
      copyDir(srcDir, versionDir);
      console.log(`✓ 成功从当前工作目录创建版本目录: ${versionDir}`);
    } else {
      console.error(`源目录不存在: ${srcDir}`);
    }
  }
}

// 主函数
function main() {
  const args = process.argv.slice(2);
  const force = args.includes('--force');
  
  if (args.includes('--tag')) {
    const tagIndex = args.indexOf('--tag');
    const tagName = args[tagIndex + 1];
    if (tagName) {
      const versionName = tagName.startsWith('v') ? tagName.substring(1) : tagName;
      checkoutDocsFromRef(tagName, path.join(versionsDir, versionName));
    } else {
      console.error('请指定标签名称: --tag <tag-name>');
      process.exit(1);
    }
  } else if (args.includes('--branch')) {
    const branchIndex = args.indexOf('--branch');
    const branchName = args[branchIndex + 1];
    if (branchName) {
      syncFromBranch(branchName);
    } else {
      console.error('请指定分支名称: --branch <branch-name>');
      process.exit(1);
    }
  } else {
    // 默认：同步所有标签（使用 --force 强制同步所有版本）
    syncAllVersions(force);
  }
}

main();
