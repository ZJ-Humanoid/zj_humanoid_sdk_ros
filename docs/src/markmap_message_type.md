---
layout: doc
title: Message Type 导图
description: zj_humanoid 消息类型数据结构导图
---

# zj_humanoid 数据结构导图

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

