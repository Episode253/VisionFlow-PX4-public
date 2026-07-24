import { defineConfig } from "vitepress";
import { withMermaid } from "vitepress-plugin-mermaid";
import navbarData from "./navbar.json" with { type: "json" };
import { sidebar } from "./get_sidebar.js";

// GitHub Pages base: https://<user>.github.io/VisionFlow-PX4/
// Override with BASE env var for other hosts (use "/" for root).
const base = process.env.BASE ?? "/VisionFlow-PX4/";

const config = defineConfig({
  title: "VisionFlow-PX4",
  description:
    "VisionFlow-PX4 定制化 PX4 Autopilot — 无人机-机械臂协同操作仿真与控制开发手册",
  base,
  lang: "zh-CN",
  outDir: "../site",
  cleanUrls: true,
  ignoreDeadLinks: true,
  metaChunk: true,
  srcExclude: ["**/SUMMARY.md"],

  head: [
    ["link", { rel: "icon", type: "image/svg+xml", href: `${base}favicon.svg` }],
    ["meta", { property: "og:type", content: "website" }],
    ["meta", { property: "og:site_name", content: "VisionFlow-PX4" }],
  ],

  markdown: {
    math: false,
    lineNumbers: true,
  },

  locales: {
    zh: {
      label: "中文",
      lang: "zh-CN",
      themeConfig: {
        sidebar: sidebar("zh"),
        nav: [
          { text: "首页", link: "/zh/" },
          { text: "快速开始", link: "/zh/getting-started/" },
          { text: "系统架构", link: "/zh/architecture/overview" },
          { text: "核心模块", link: "/zh/modules/" },
          ...navbarData.nav,
        ],
        outline: { level: [2, 3], label: "本页目录" },
        docFooter: { prev: "上一页", next: "下一页" },
        lastUpdatedText: "最后更新",
        returnToTopLabel: "返回顶部",
        sidebarMenuLabel: "菜单",
        darkModeSwitchLabel: "外观",
        lightModeSwitchTitle: "切换到浅色模式",
        darkModeSwitchTitle: "切换到深色模式",
        langMenuLabel: "切换语言",
      },
    },
    en: {
      label: "English",
      lang: "en-US",
      themeConfig: {
        sidebar: sidebar("en"),
        nav: [
          { text: "Home", link: "/en/" },
          { text: "Getting Started", link: "/en/getting-started/" },
          { text: "Architecture", link: "/en/architecture/overview" },
          { text: "Modules", link: "/en/modules/" },
          {
            text: "Related",
            items: [
              { text: "PX4 Autopilot", link: "https://px4.io/" },
              { text: "PX4 Docs", link: "https://docs.px4.io/main/en/" },
              { text: "QGroundControl", link: "http://qgroundcontrol.com/" },
              { text: "MAVLink", link: "https://mavlink.io/en/" },
              { text: "ROS 2 Humble", link: "https://docs.ros.org/en/humble/" },
              { text: "Gazebo Harmonic", link: "https://gazebosim.org/" },
            ],
          },
          {
            text: "Papers",
            items: [
              { text: "PreGME (arXiv:2512.22957)", link: "https://arxiv.org/abs/2512.22957" },
              { text: "Theory paper (PDF)", link: "/references/pregme-paper.pdf" },
              { text: "Parameter reference (PDF)", link: "/references/pregme-parameter-reference.pdf" },
            ],
          },
        ],
        outline: { level: [2, 3], label: "On this page" },
      },
    },
  },

  themeConfig: {
    logo: "/logo.svg",
    siteTitle: "VisionFlow-PX4",

    search: {
      provider: "local",
      options: {
        locales: {
          zh: {
            translations: {
              button: { buttonText: "搜索文档", buttonAriaLabel: "搜索文档" },
              modal: {
                noResultsText: "无法找到相关结果",
                resetButtonTitle: "清除查询条件",
                footer: { selectText: "选择", navigateText: "切换", closeText: "关闭" },
              },
            },
          },
        },
      },
    },

    socialLinks: [
      { icon: "github", link: "https://github.com/Renwang-Huang/VisionFlow-PX4" },
    ],

    editLink: {
      pattern:
        "https://github.com/Renwang-Huang/VisionFlow-PX4/edit/simulation/docs/:path",
      text: "在 GitHub 上编辑此页",
    },

    footer: {
      message:
        '基于 <a href="https://github.com/PX4/PX4-Autopilot">PX4 Autopilot</a> · 由 WindyLab 开发 · CC BY 4.0',
      copyright:
        'Copyright © 2025-present <a href="https://github.com/Renwang-Huang">Renwang Huang</a> / WindyLab',
    },
  },
});

export default withMermaid(config);
