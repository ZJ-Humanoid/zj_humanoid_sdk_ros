import DefaultTheme from 'vitepress/theme'
import Markmap from '../components/Markmap.vue'
import './custom.css'

export default {
  extends: DefaultTheme,
  enhanceApp({ app }) {
    app.component('Markmap', Markmap)
  }
}
