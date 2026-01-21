---
title: Develop
---

<script setup>
import { onMounted } from 'vue'
import { withBase } from 'vitepress'

onMounted(() => {
  // 兼容 GitHub Pages base
  window.location.replace(withBase('/versions/develop/'))
})
</script>

正在跳转到 develop 版本文档：[`/versions/develop/`](/versions/develop/)

