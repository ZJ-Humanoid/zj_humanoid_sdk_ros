import DefaultTheme from 'vitepress/theme'
import Markmap from '../components/Markmap.vue'
import MarkdownInclude from '../components/MarkdownInclude.vue'
import './custom.css'

export default {
  extends: DefaultTheme,
  enhanceApp({ app }) {
    app.component('Markmap', Markmap)
    app.component('MarkdownInclude', MarkdownInclude)
  }
}
