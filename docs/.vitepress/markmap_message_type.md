## zj_humanoid数据结构
<Markmap :content="basicContent" />

<script setup>
import basicContentMd from './zj_humanoid_types.md?raw'
const frontmatter = `---
title: markmap
markmap:
  initialExpandLevel: 4
  colorFreezeLevel: 3
---

`
const basicContent = frontmatter + basicContentMd
</script>