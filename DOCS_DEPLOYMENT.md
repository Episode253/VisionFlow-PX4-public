# 文档网站构建与托管指南 / Docs Build & Hosting Guide

> VisionFlow-PX4 开发手册基于 **VitePress**（与 PX4 官方文档同栈）。
> 源文件全部在 [`docs/`](docs/)；构建产物 `site/` 由 `npm run build` 现场生成，**不纳入 git**（已在 `.gitignore`）。
> 日常只需两步：本地 `npm run dev` 预览 + 推送触发云端自动部署。本指南其余章节（本地构建、其他托管方式）作为可选备查。

---

## 1. 技术栈与目录速览

| 项 | 值 |
|----|----|
| 生成器 | VitePress `^1.6.3`（Vue / Vite） |
| 内容源 | `docs/zh/`（默认中文）、`docs/en/`（英文），首页 `docs/index.md` |
| 侧边栏 | 由 `docs/<lang>/SUMMARY.md` 自动生成（`docs/.vitepress/get_sidebar.js`） |
| 主配置 | `docs/.vitepress/config.mjs` |
| 主题 | `docs/.vitepress/theme/`（靛蓝品牌色 + Hero 渐变 + medium-zoom） |
| 静态资源 | `docs/public/`（logo、favicon、PDF） |
| 构建产物 | `site/`（`outDir`，本地 build 时生成，已 gitignore，不入库） |
| 部署基路径 | `base = /VisionFlow-PX4/`（GitHub Pages 项目站点子路径） |

关键命令都在 `docs/package.json` 里：`dev` / `build` / `preview`。

---

## 2. 环境准备（首次）

推荐 **Node.js 20 LTS**（与 CI 一致；18+ 均可）。

```bash
cd docs
npm install          # 安装 VitePress、mermaid 等依赖，生成 node_modules/
```

> `docs/node_modules/` 不应提交 git。若根目录 `.gitignore` 未忽略，请补充 `docs/node_modules/`。

---

## 3. 本地开发预览（热重载）★ 日常主用

这是你日常唯一需要的本地命令：改 `.md` 存盘即刷新，**不生成 `site/`、不写磁盘产物**。

```bash
cd docs
npm run dev
```

打开 <http://localhost:5173/VisionFlow-PX4/>（注意结尾的子路径）。

- 中文文档：`/VisionFlow-PX4/zh/`
- 英文文档：`/VisionFlow-PX4/en/`
- 端口占用时 VitePress 会自动换端口，看终端输出。

---

## 4. 本地构建 + 产物预览（可选，一般用不到）

> 你的常规流程是「dev 预览 + 云端部署」，**本节可跳过**。仅当你想在本地离线验证
> 真实构建产物、或临时手动出一份静态文件时才需要。运行后会在 `docs/../site/`
> 生成产物（已 gitignore，不会入库）。

```bash
cd docs
npm run build        # 输出到 ../site/
npm run preview      # 本地起静态服务器预览 site/
```

`build` 做的事：
- 清空并重新生成 `site/`（VitePress 默认清 `outDir`）。
- 侧边栏从 `SUMMARY.md` 重新解析，mermaid 图在浏览器端渲染。
- 所有内部链接、资源自动加上 `base` 前缀 `/VisionFlow-PX4/`。

`preview` 默认在 <http://localhost:4173/VisionFlow-PX4/>。

### 常用检查

```bash
find site -name '*.html' | wc -l        # 页面总数（当前约 116）
python3 -m http.server 8080 -d site     # 备选：任意静态服务器预览
```

> ⚠️ 用 `python3 -m http.server` 预览时，因为页面里资源是 `/VisionFlow-PX4/...` 绝对路径，
> 需访问 <http://localhost:8080/VisionFlow-PX4/> 才能正确加载（或改用下方「根路径构建」）。

### 根路径构建（自定义域名 / 根目录托管时）

若网站托管在域名根目录（如 `https://docs.example.com/` 而非 `.../VisionFlow-PX4/`）：

```bash
cd docs
BASE=/ npm run build
```

`config.mjs` 已支持 `BASE` 环境变量覆盖，默认 `/VisionFlow-PX4/`。

---

## 5. 服务器托管

### 方案 A：GitHub Pages + GitHub Actions（推荐，自动化）

你仓库里现有的 `docs_deploy.yml`、`docs-orchestrator.yml` 是**从上游 PX4 继承的**，
部署目标是 `PX4/docs.px4.io` 和 PX4 的 AWS，用的是 PX4 的 secrets——**对你的 fork 不生效**，
可以忽略或删除。下面是一套面向你自己仓库的干净工作流。

**步骤：**

1. 在 GitHub 仓库 **Settings → Pages → Build and deployment → Source** 选择 **GitHub Actions**。
2. 新建 `.github/workflows/deploy-docs.yml`（内容见下）。
3. 确认 `base` 与仓库名一致：仓库是 `VisionFlow-PX4` → 站点地址
   `https://<用户名>.github.io/VisionFlow-PX4/`，`base` 保持默认即可。
4. push 到目标分支即自动构建部署。

```yaml
name: Deploy Docs to GitHub Pages

on:
  push:
    branches: [simulation]        # 改成你的发布分支（如 main）
    paths:
      - 'docs/**'
      - '.github/workflows/deploy-docs.yml'
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: pages
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm
          cache-dependency-path: docs/package-lock.json
      - name: Install dependencies
        working-directory: docs
        run: npm ci
      - name: Build with VitePress
        working-directory: docs
        run: npm run build
      - uses: actions/configure-pages@v5
      - uses: actions/upload-pages-artifact@v3
        with:
          path: site          # 与 config.mjs 的 outDir 一致

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - id: deployment
        uses: actions/deploy-pages@v4
```

> 该工作流用 GitHub 官方 Pages Action 直接部署 artifact，**不需要 gh-pages 分支、不需要额外 secret**。
> `npm ci` 需要 `docs/package-lock.json`（首次 `npm install` 已生成，记得提交）。

### 方案 B：GitHub Pages + gh-pages 分支（手动 / 无 Actions）

适合想手动控制、或不想开 Actions 的情况。

```bash
cd docs && npm run build && cd ..
touch site/.nojekyll                    # 关键：禁用 Jekyll，否则 _ 开头资源被吞
npx gh-pages -d site -b gh-pages        # 需要 npx，会推到 gh-pages 分支
```

然后 **Settings → Pages → Source** 选 `gh-pages` 分支、`/ (root)`。

### 方案 C：通用静态服务器（Nginx / 自建 VPS）

VitePress 产物是纯静态文件，任何静态服务器都能托管。

**若托管在子路径 `/VisionFlow-PX4/`**（保持默认 `base`）：

```nginx
server {
    listen 80;
    server_name docs.example.com;

    location /VisionFlow-PX4/ {
        alias /var/www/visionflow/site/;
        try_files $uri $uri/ $uri.html /VisionFlow-PX4/404.html;
    }
}
```

**若托管在根路径**（用 `BASE=/ npm run build` 重新构建后）：

```nginx
server {
    listen 80;
    server_name docs.example.com;
    root /var/www/visionflow/site;
    index index.html;
    location / {
        try_files $uri $uri/ $uri.html /404.html;
    }
}
```

部署即「把 `site/` 传上去」：

```bash
BASE=/ npm run build --prefix docs           # 根路径构建
rsync -avz --delete site/ user@server:/var/www/visionflow/site/
```

> `cleanUrls: true` 已开启，所以 URL 无 `.html` 后缀，`try_files` 里的 `$uri.html` 回退很重要。

### 方案 D：Docker（可选，与项目 Docker 工作流统一）

在仓库根放一个 `docs/Dockerfile` 或用现成 nginx 镜像：

```bash
cd docs && BASE=/ npm run build && cd ..
docker run --rm -d -p 8080:80 \
  -v "$(pwd)/site:/usr/share/nginx/html:ro" \
  --name visionflow-docs nginx:alpine
# 访问 http://localhost:8080
```

---

## 6. 修改后如何更新线上

| 场景 | 操作 |
|------|------|
| 改文档内容 | 编辑 `docs/zh/**.md` 或 `docs/en/**.md`（两语请同步） |
| 加/删页面、调整章节顺序 | 同时改对应 `docs/<lang>/SUMMARY.md`（侧边栏由它生成） |
| 换 logo/favicon/PDF | 放进 `docs/public/`，Markdown 里用 `/文件名` 引用 |
| 改导航栏外链 | 编辑 `docs/.vitepress/navbar.json` 与 `config.mjs` 的 `nav` |
| 改配色/首页 | `docs/.vitepress/theme/style.css`、`docs/index.md`（Hero） |

改完统一走：`本地 npm run dev 预览 → 提交源码（docs/）→ 推送触发云端自动构建部署`。
（无需本地 build，也无需提交任何产物。）

---

## 7. 常见问题（FAQ）

- **页面 404 / 样式丢失**：多半是 `base` 与实际托管路径不符。子路径托管用默认
  `/VisionFlow-PX4/`；根路径托管用 `BASE=/ npm run build`。
- **GitHub Pages 上 `_` 开头资源加载失败**：`site/` 缺 `.nojekyll`。方案 A 的官方
  Action 会自动处理；方案 B 记得手动 `touch site/.nojekyll`。
- **mermaid 图不显示**：确认代码块语言标注是 ```` ```mermaid ````；图是客户端渲染，
  `npm run preview` 或真实浏览器里查看（直接看 HTML 源码看不到 SVG 属正常）。
- **`npm ci` 报错找不到 lock 文件**：先本地 `cd docs && npm install` 生成
  `package-lock.json` 并提交。
- **构建内存不足（大仓 OOM）**：`build` 脚本已带 `--max-old-space-size=8192`，
  一般够用；不够再调大。
- **中英内容不同步**：`docs/zh/` 与 `docs/en/` 目录结构必须镜像一致，
  两边 `SUMMARY.md` 的条目也要对应，否则语言切换会落到 404。

---

## 8. 命令速查

```bash
# 首次
cd docs && npm install

# 日常写作
cd docs && npm run dev            # http://localhost:5173/VisionFlow-PX4/

# 发布前验证
cd docs && npm run build          # → ../site/
cd docs && npm run preview        # http://localhost:4173/VisionFlow-PX4/

# 根路径部署构建
cd docs && BASE=/ npm run build

# 手动推 GitHub Pages（方案 B）
cd docs && npm run build && cd .. && touch site/.nojekyll && npx gh-pages -d site
```

