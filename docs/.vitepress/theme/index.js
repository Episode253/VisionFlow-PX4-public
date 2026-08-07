// https://vitepress.dev/guide/custom-theme
import DefaultTheme from "vitepress/theme";
import "./style.css";

// medium-zoom for clickable images
import { onMounted, watch, nextTick } from "vue";
import { useRoute, inBrowser } from "vitepress";
import mediumZoom from "medium-zoom";
import { setupMermaidZoom } from "./mermaid-zoom.js";

/** @type {import('vitepress').Theme} */
export default {
  extends: DefaultTheme,
  enhanceApp({ app }) {
    app.config.globalProperties.$buildTime = JSON.stringify(
      new Date().toISOString()
    );
  },

  // medium zoom: https://github.com/vuejs/vitepress/issues/854
  setup() {
    const route = useRoute();
    const initZoom = () => {
      mediumZoom(".main img", { background: "var(--vp-c-bg)" });
    };
    // Mermaid renders SVG asynchronously; retry so late diagrams get wired up.
    const initMermaidZoom = () => {
      let tries = 0;
      const tick = () => {
        setupMermaidZoom();
        if (++tries < 10) setTimeout(tick, 300);
      };
      tick();
    };
    onMounted(() => {
      initZoom();
      initMermaidZoom();
      // Re-scroll to hash after fonts/layout settle (large pages).
      if (inBrowser && location.hash) {
        const id = decodeURIComponent(location.hash.slice(1));
        const fontsReady = document.fonts?.ready ?? Promise.resolve();
        const loadReady =
          document.readyState === "complete"
            ? Promise.resolve()
            : new Promise((r) =>
                window.addEventListener("load", r, { once: true })
              );
        Promise.all([fontsReady, loadReady]).then(() => {
          requestAnimationFrame(() =>
            requestAnimationFrame(() => {
              const el = document.getElementById(id);
              if (el) el.scrollIntoView();
            })
          );
        });
      }
    });
    watch(
      () => route.path,
      () =>
        nextTick(() => {
          initZoom();
          initMermaidZoom();
        })
    );
  },
};
