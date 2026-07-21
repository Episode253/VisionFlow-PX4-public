/**
 * VisionFlow-PX4 Custom JavaScript
 *
 * Features:
 * - Unified navigation bar with dropdown menus
 * - Keyboard shortcut for search (Ctrl+K / Cmd+K)
 * - Three-column layout with right-side TOC scroll spy
 * - Mobile hamburger drawer + mobile sidebar drawer
 * - Mobile TOC floating button + dropdown
 * - Back to top floating button
 * - Code block language labels and copy feedback
 * - File reference card "Copy path" functionality
 * - Page feedback ("Was this helpful?")
 * - Breadcrumb and page description injection
 * - External link handling
 * - Smooth scroll with hash sync (respects prefers-reduced-motion)
 * - Search placeholder localization
 * - Accessibility: aria attributes, keyboard nav, focus management
 */

(function () {
  "use strict";

  // =========================================================================
  // Helpers
  // =========================================================================

  function ready(fn) {
    if (document.readyState !== "loading") {
      fn();
    } else {
      document.addEventListener("DOMContentLoaded", fn);
    }
  }

  function prefersReducedMotion() {
    return window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  }

  function getStickyHeaderHeight() {
    var header = document.querySelector(".md-header");
    if (header) {
      var height = header.getBoundingClientRect().height;
      if (height > 0) return height + 10;
    }
    var styles = getComputedStyle(document.documentElement);
    var h = parseInt(styles.getPropertyValue("--vf-header-height"), 10) || 68;
    return h + 10;
  }

  function getLang() {
    var lang = document.documentElement.lang || "";
    return lang.startsWith("zh") ? "zh" : "en";
  }

  /**
   * Resolve a URL to an absolute, normalized site path.
   */
  function resolveUrl(path) {
    if (/^https?:\/\//.test(path)) return path;
    var base = window.__VF_BASE_URL || "";
    var clean = path.replace(/^\.\//, "");
    clean = clean.replace(/\/index(\.md)?$/, "/").replace(/\.md$/, "/");
    clean = clean.replace(/\/+/g, "/");
    if (/^\/(en|zh)\//.test(clean)) {
      clean = clean.replace(/\/+$/, "");
      return base + clean + "/";
    }
    clean = clean.replace(/^\/+/, "").replace(/\/+$/, "");
    if (clean === "") {
      return getLang() === "zh" ? (base + "/") : (base + "/en/");
    }
    var lang = getLang();
    return lang === "zh" ? (base + "/" + clean + "/") : (base + "/en/" + clean + "/");
  }

  function normalizePathname(path) {
    var segments = path
      .replace(/\/index\.html$/, "")
      .replace(/\/+$/, "")
      .split("/")
      .filter(Boolean);
    if (segments.length > 0 && (segments[0] === "en" || segments[0] === "zh")) {
      segments = segments.slice(1);
    }
    return segments.length === 0 ? "index" : segments[0];
  }

  // =========================================================================
  // Initialization
  // =========================================================================

  ready(function () {
    initExternalLinks();
    initSmoothScroll();
    enhanceSearchExperience();
    initMobileTOC();
    initScrollSpy();
    initHashHandling();
    initReducedMotionObserver();
    initBackToTop();
    initCodeBlocks();
    initFileCards();
    initPageFeedback();
    initMobileSidebar();
    injectPageMeta();
    initSearchKeyboardShortcut();
    // Nav must be last since it depends on DOM from other init functions
    try { enhanceAriaLabels(); } catch (e) { console.warn("enhanceAriaLabels:", e); }
    try { initUnifiedNav(); } catch (e) { console.warn("initUnifiedNav:", e); }
  });

  // Material SPA navigation — re-scan Mermaid diagrams after each navigation.
  // Material for MkDocs natively handles Mermaid rendering (loads mermaid.min.js
  // dynamically, calls mermaid.initialize + mermaid.render, injects SVG into
  // shadow DOM). This module only wraps already-rendered SVGs with a lightbox.
  if (typeof document$ !== "undefined") {
    document$.subscribe(function () {
      tryWrapMermaid(0);
    });
  }

  // =========================================================================
  // Accessibility — aria-label improvements
  // =========================================================================

  function enhanceAriaLabels() {
    // Right sidebar TOC
    var tocNav = document.querySelector(".md-nav--secondary");
    if (tocNav) {
      var lang = getLang();
      tocNav.setAttribute("aria-label", lang === "zh" ? "本页目录" : "On this page");
    }

    // Ensure back-to-top has proper aria-label
    var backToTopBtn = document.querySelector(".md-back-to-top");
    if (backToTopBtn && !backToTopBtn.getAttribute("aria-label")) {
      backToTopBtn.setAttribute("aria-label", getLang() === "zh" ? "返回顶部" : "Back to top");
    }

    // Set aria-label on nav search
    var searchLabels = document.querySelectorAll('.md-header__button[for="__search"]');
    searchLabels.forEach(function (btn) {
      if (!btn.getAttribute("aria-label")) {
        btn.setAttribute("aria-label", getLang() === "zh" ? "搜索" : "Search");
      }
    });
  }

  // =========================================================================
  // External links
  // =========================================================================

  function initExternalLinks() {
    var links = document.querySelectorAll(
      '.md-content a[href^="http"]:not([href*="' +
        window.location.hostname +
        '"]):not(.md-header__dropdown-link)'
    );
    links.forEach(function (link) {
      if (!link.querySelector(".md-icon")) {
        link.setAttribute("rel", "noopener noreferrer");
        link.setAttribute("target", "_blank");
      }
    });
  }

  // =========================================================================
  // Smooth scroll for anchor links
  // =========================================================================

  function initSmoothScroll() {
    document.addEventListener("click", function (e) {
      var link = e.target.closest('a[href^="#"]');
      if (!link) return;

      var rawHref = link.getAttribute("href");
      if (!rawHref || rawHref === "#") return;

      var targetId = rawHref.substring(1);
      var target = document.getElementById(targetId);
      if (!target) return;

      e.preventDefault();

      var behavior = prefersReducedMotion() ? "instant" : "smooth";
      var topOffset = getStickyHeaderHeight();
      var targetTop = target.getBoundingClientRect().top + window.pageYOffset;

      window.scrollTo({
        top: targetTop - topOffset,
        behavior: behavior,
      });

      var currentPath = window.location.pathname + window.location.search;
      history.pushState(null, null, currentPath + "#" + targetId);

      target.setAttribute("tabindex", "-1");
      target.focus({ preventScroll: true });
    });
  }

  // =========================================================================
  // Search Enhancements
  // =========================================================================

  function enhanceSearchExperience() {
    var searchInput = document.querySelector('.md-search__input[type="text"]');
    if (searchInput) {
      var lang = getLang();
      searchInput.setAttribute("placeholder", lang === "zh" ? "搜索文档…" : "Search documentation");
    }

    // Add keyboard shortcut hint to search
    addSearchKbdHint();
  }

  function addSearchKbdHint() {
    var labels = document.querySelectorAll('.md-header__button[for="__search"]');
    labels.forEach(function (label) {
      if (label.querySelector(".vf-search-kbd")) return; // already added
      var kbdHint = document.createElement("span");
      kbdHint.className = "vf-search-kbd";
      kbdHint.setAttribute("aria-hidden", "true");

      var isMac = /Mac|iPhone|iPad|iPod/.test(navigator.platform || navigator.userAgentData?.platform || "");
      var modKey = isMac ? "⌘" : "Ctrl";

      var kbd1 = document.createElement("kbd");
      kbd1.textContent = modKey;
      var kbd2 = document.createElement("kbd");
      kbd2.textContent = "K";

      kbdHint.appendChild(kbd1);
      kbdHint.appendChild(kbd2);
      label.appendChild(kbdHint);
    });
  }

  function initSearchKeyboardShortcut() {
    document.addEventListener("keydown", function (e) {
      // Ctrl+K or Cmd+K
      if ((e.ctrlKey || e.metaKey) && e.key === "k") {
        e.preventDefault();
        // Click the search toggle to open the search overlay
        var searchToggle = document.querySelector('.md-header__button[for="__search"]');
        if (searchToggle) {
          searchToggle.click();
        }
      }
    });
  }

  // =========================================================================
  // Unified Navigation
  // =========================================================================

  function getActiveSection() {
    return normalizePathname(window.location.pathname);
  }

  function initUnifiedNav() {
    var navContainer = document.getElementById("md-main-nav");
    if (!navContainer) return;

    var lang = getLang();
    var activeSection = getActiveSection();
    var isEn = lang === "en";

    var navItems = [
      {
        type: "link",
        label: isEn ? "Overview" : "概述",
        href: resolveUrl("/"),
        section: "index",
      },
      {
        type: "link",
        label: isEn ? "Getting Started" : "快速开始",
        href: resolveUrl("/getting-started/"),
        section: "getting-started",
      },
      {
        type: "link",
        label: isEn ? "Architecture" : "系统架构",
        href: resolveUrl("/architecture/"),
        section: "architecture",
      },
      {
        type: "link",
        label: isEn ? "Core Modules" : "核心模块",
        href: resolveUrl("/modules/"),
        section: "modules",
      },
      {
        type: "link",
        label: isEn ? "Simulation" : "仿真资产",
        href: resolveUrl("/simulation/"),
        section: "simulation",
      },
      {
        type: "link",
        label: isEn ? "Toolchain" : "工具链",
        href: resolveUrl("/tools/"),
        section: "tools",
      },
      {
        type: "dropdown",
        label: isEn ? "Resources" : "资源",
        id: "dropdown-resources",
        items: [
          { type: "label", label: isEn ? "Ecosystem" : "生态系统" },
          { type: "external", label: "PX4", href: "https://px4.io/" },
          { type: "external", label: "Gazebo", href: "https://gazebosim.org/" },
          { type: "external", label: "ROS 2", href: "https://www.ros.org/" },
          { type: "external", label: "Zenoh", href: "https://zenoh.io/" },
          { type: "external", label: "MAVLink", href: "https://mavlink.io/" },
        ],
      },
      {
        type: "dropdown",
        label: isEn ? "More" : "更多",
        id: "dropdown-more",
        items: [
          { type: "label", label: isEn ? "Documentation" : "文档" },
          {
            type: "link",
            label: isEn ? "Hardware" : "硬件支持",
            href: resolveUrl("/hardware/"),
            section: "hardware",
          },
          {
            type: "link",
            label: isEn ? "Protocols" : "通信协议",
            href: resolveUrl("/messages/"),
            section: "messages",
          },
          {
            type: "link",
            label: isEn ? "Development" : "开发指南",
            href: resolveUrl("/development/"),
            section: "development",
          },
          {
            type: "link",
            label: isEn ? "References" : "参考资料",
            href: resolveUrl("/references/"),
            section: "references",
          },
        ],
      },
    ];

    var allDropdowns = [];

    navItems.forEach(function (item) {
      if (item.type === "link") {
        var li = createNavLink(item.label, item.href, item.section === activeSection, item.section);
        navContainer.appendChild(li);
      } else if (item.type === "dropdown") {
        var result = createNavDropdown(item.label, item.id, item.items, activeSection);
        navContainer.appendChild(result.li);
        allDropdowns.push({
          trigger: result.trigger,
          menu: result.menu,
          item: result.li,
        });
      }
    });

    // Click-outside handler
    document.addEventListener("click", function (e) {
      allDropdowns.forEach(function (dd) {
        if (!dd.item.contains(e.target)) {
          closeDropdown(dd);
        }
      });
    });

    // Escape key handler
    document.addEventListener("keydown", function (e) {
      if (e.key === "Escape") {
        var anyOpen = false;
        allDropdowns.forEach(function (dd) {
          if (dd.menu.classList.contains("is-open")) {
            closeDropdown(dd);
            dd.trigger.focus();
            anyOpen = true;
          }
        });
        if (anyOpen) e.preventDefault();
      }
    });

    initMobileNav(navItems, activeSection);
    initResponsiveNav(navContainer, allDropdowns, navItems);
  }

  function createNavLink(label, href, isActive, section) {
    var li = document.createElement("div");
    li.className = "md-header__nav-item";
    if (section) li.setAttribute("data-section", section);

    var a = document.createElement("a");
    a.className = "md-header__nav-link";
    if (isActive) {
      a.classList.add("md-header__nav-link--active");
      a.setAttribute("aria-current", "page");
    }
    a.href = href;
    a.textContent = label;

    li.appendChild(a);
    return li;
  }

  function createNavDropdown(label, id, items, activeSection) {
    var li = document.createElement("div");
    li.className = "md-header__nav-item";

    var trigger = document.createElement("button");
    trigger.className = "md-header__nav-link md-header__nav-link--dropdown";
    trigger.setAttribute("type", "button");
    trigger.setAttribute("aria-expanded", "false");
    trigger.setAttribute("aria-haspopup", "true");
    trigger.setAttribute("aria-controls", id);
    trigger.textContent = label;

    var hasActiveChild = items.some(function (item) {
      return item.section && item.section === activeSection;
    });
    if (hasActiveChild) {
      trigger.classList.add("md-header__nav-link--active");
    }

    var menu = document.createElement("div");
    menu.className = "md-header__dropdown";
    menu.id = id;
    menu.setAttribute("role", "menu");
    menu.setAttribute("aria-label", label);

    var lastLabelIdx = -1;
    items.forEach(function (item, idx) {
      if (item.type === "label") {
        if (lastLabelIdx >= 0 && idx - lastLabelIdx > 1) {
          var sep = document.createElement("hr");
          sep.className = "md-header__dropdown-separator";
          sep.setAttribute("role", "separator");
          menu.appendChild(sep);
        }
        var labelEl = document.createElement("span");
        labelEl.className = "md-header__dropdown-label";
        labelEl.textContent = item.label;
        menu.appendChild(labelEl);
        lastLabelIdx = idx;
      } else if (item.type === "link") {
        var a = document.createElement("a");
        a.className = "md-header__dropdown-link";
        if (item.section === activeSection) {
          a.classList.add("md-header__dropdown-link--active");
        }
        a.href = item.href;
        a.setAttribute("role", "menuitem");
        a.setAttribute("tabindex", "-1");
        a.textContent = item.label;
        menu.appendChild(a);
      } else if (item.type === "external") {
        var extA = document.createElement("a");
        extA.className = "md-header__dropdown-link md-header__dropdown-link--external";
        extA.href = item.href;
        extA.setAttribute("role", "menuitem");
        extA.setAttribute("tabindex", "-1");
        extA.setAttribute("target", "_blank");
        extA.setAttribute("rel", "noopener noreferrer");
        extA.textContent = item.label;
        menu.appendChild(extA);
      }
    });

    trigger.addEventListener("click", function (e) {
      e.preventDefault();
      e.stopPropagation();
      if (menu.classList.contains("is-open")) {
        closeDropdown({ trigger: trigger, menu: menu, item: li });
      } else {
        openDropdown({ trigger: trigger, menu: menu, item: li });
      }
    });

    trigger.addEventListener("keydown", function (e) {
      if (e.key === "Enter" || e.key === " " || e.key === "ArrowDown") {
        e.preventDefault();
        e.stopPropagation();
        if (!menu.classList.contains("is-open")) {
          openDropdown({ trigger: trigger, menu: menu, item: li });
        }
        var firstLink = menu.querySelector(".md-header__dropdown-link");
        if (firstLink) {
          setTimeout(function () { firstLink.focus(); }, 80);
        }
      }
    });

    menu.addEventListener("keydown", function (e) {
      var items = Array.from(menu.querySelectorAll(".md-header__dropdown-link"));
      var currentIdx = items.indexOf(document.activeElement);

      if (e.key === "ArrowDown") {
        e.preventDefault();
        var nextIdx = (currentIdx + 1) % items.length;
        items[nextIdx].focus();
      } else if (e.key === "ArrowUp") {
        e.preventDefault();
        var prevIdx = (currentIdx - 1 + items.length) % items.length;
        items[prevIdx].focus();
      } else if (e.key === "Escape") {
        e.preventDefault();
        closeDropdown({ trigger: trigger, menu: menu, item: li });
        trigger.focus();
      } else if (e.key === "Tab" && !e.shiftKey && currentIdx === items.length - 1) {
        closeDropdown({ trigger: trigger, menu: menu, item: li });
      }
    });

    li.appendChild(trigger);
    li.appendChild(menu);

    return { li: li, trigger: trigger, menu: menu };
  }

  function openDropdown(dd) {
    dd.menu.classList.add("is-open");
    dd.item.classList.add("is-open");
    dd.trigger.setAttribute("aria-expanded", "true");
  }

  function closeDropdown(dd) {
    dd.menu.classList.remove("is-open");
    dd.item.classList.remove("is-open");
    dd.trigger.setAttribute("aria-expanded", "false");
  }

  // =========================================================================
  // Responsive Nav
  // =========================================================================

  function initResponsiveNav(navContainer, allDropdowns, navItems) {
    var moreDropdown = null;
    for (var i = allDropdowns.length - 1; i >= 0; i--) {
      if (allDropdowns[i].menu.id === "dropdown-more") {
        moreDropdown = allDropdowns[i];
        break;
      }
    }
    if (!moreDropdown) return;

    var movedSections = {};
    var resizeTicking = false;

    function applyVisibility() {
      var width = window.innerWidth;
      var toMove = [];

      // At <= 1400px, move Simulation and Toolchain into More
      if (width <= 1400) {
        toMove.push("simulation", "tools");
      }

      toMove.forEach(function (section) {
        if (movedSections[section]) return;
        var el = navContainer.querySelector('.md-header__nav-item[data-section="' + section + '"]');
        if (!el) return;
        movedSections[section] = { el: el, nextSibling: el.nextSibling };
        var moreMenu = moreDropdown.menu;
        var linkEl = document.createElement("a");
        var origLink = el.querySelector(".md-header__nav-link");
        linkEl.className = "md-header__dropdown-link";
        linkEl.href = origLink.getAttribute("href");
        linkEl.textContent = origLink.textContent;
        linkEl.setAttribute("role", "menuitem");
        linkEl.setAttribute("tabindex", "-1");
        if (origLink.getAttribute("aria-current") === "page") {
          linkEl.classList.add("md-header__dropdown-link--active");
        }
        el._moreClone = linkEl;
        el.style.display = "none";
        moreMenu.appendChild(linkEl);
      });

      Object.keys(movedSections).forEach(function (section) {
        if (toMove.indexOf(section) !== -1) return;
        var info = movedSections[section];
        if (info.el._moreClone && info.el._moreClone.parentNode) {
          info.el._moreClone.parentNode.removeChild(info.el._moreClone);
          info.el._moreClone = null;
        }
        info.el.style.display = "";
        delete movedSections[section];
      });
    }

    function onResize() {
      if (!resizeTicking) {
        requestAnimationFrame(function () {
          applyVisibility();
          resizeTicking = false;
        });
        resizeTicking = true;
      }
    }

    window.addEventListener("resize", onResize, { passive: true });
    applyVisibility();
  }

  // =========================================================================
  // Mobile Navigation
  // =========================================================================

  function initMobileNav(navItems, activeSection) {
    var overlay = document.createElement("div");
    overlay.className = "md-mobile-nav__overlay";
    overlay.setAttribute("aria-hidden", "true");

    var drawer = document.createElement("nav");
    drawer.className = "md-mobile-nav";
    drawer.setAttribute("aria-label", getLang() === "zh" ? "移动端导航" : "Mobile navigation");
    drawer.setAttribute("id", "md-mobile-nav");

    navItems.forEach(function (item) {
      if (item.type === "link") {
        var li = document.createElement("div");
        li.className = "md-header__nav-item";
        var a = document.createElement("a");
        a.className = "md-header__nav-link";
        if (item.section === activeSection) {
          a.classList.add("md-header__nav-link--active");
          a.setAttribute("aria-current", "page");
        }
        a.href = item.href;
        a.textContent = item.label;
        a.addEventListener("click", closeMobileNav);
        li.appendChild(a);
        drawer.appendChild(li);
      } else if (item.type === "dropdown") {
        var result = createNavDropdown(item.label, item.id + "-mobile", item.items, activeSection);
        var links = result.menu.querySelectorAll("a");
        links.forEach(function (link) { link.addEventListener("click", closeMobileNav); });
        drawer.appendChild(result.li);
      }
    });

    document.body.appendChild(overlay);
    document.body.appendChild(drawer);

    var menuBtn = document.createElement("button");
    menuBtn.className = "md-header__menu-btn";
    menuBtn.setAttribute("type", "button");
    menuBtn.setAttribute("aria-label", getLang() === "zh" ? "打开菜单" : "Open menu");
    menuBtn.setAttribute("aria-expanded", "false");
    menuBtn.setAttribute("aria-controls", "md-mobile-nav");

    var svgNS = "http://www.w3.org/2000/svg";
    var icon = document.createElementNS(svgNS, "svg");
    icon.setAttribute("viewBox", "0 0 24 24");
    icon.setAttribute("fill", "none");
    icon.setAttribute("stroke", "currentColor");
    icon.setAttribute("stroke-width", "2");
    icon.setAttribute("stroke-linecap", "round");
    icon.setAttribute("stroke-linejoin", "round");
    icon.setAttribute("width", "20");
    icon.setAttribute("height", "20");
    icon.setAttribute("aria-hidden", "true");
    [
      {x1: "3", y1: "6", x2: "21", y2: "6"},
      {x1: "3", y1: "12", x2: "21", y2: "12"},
      {x1: "3", y1: "18", x2: "21", y2: "18"}
    ].forEach(function (a) {
      var line = document.createElementNS(svgNS, "line");
      line.setAttribute("x1", a.x1);
      line.setAttribute("y1", a.y1);
      line.setAttribute("x2", a.x2);
      line.setAttribute("y2", a.y2);
      icon.appendChild(line);
    });
    menuBtn.appendChild(icon);

    var toolsGroup = document.querySelector(".md-header__tools");
    if (toolsGroup) {
      toolsGroup.insertBefore(menuBtn, toolsGroup.firstChild);
    }

    var isMobileOpen = false;

    function openMobileNav() {
      isMobileOpen = true;
      overlay.classList.add("is-open");
      drawer.classList.add("is-open");
      menuBtn.setAttribute("aria-expanded", "true");
      menuBtn.setAttribute("aria-label", getLang() === "zh" ? "关闭菜单" : "Close menu");
      overlay.setAttribute("aria-hidden", "false");
      document.body.style.overflow = "hidden";
      var firstLink = drawer.querySelector(".md-header__nav-link");
      if (firstLink) {
        setTimeout(function () { firstLink.focus(); }, 100);
      }
    }

    function closeMobileNav() {
      isMobileOpen = false;
      overlay.classList.remove("is-open");
      drawer.classList.remove("is-open");
      menuBtn.setAttribute("aria-expanded", "false");
      menuBtn.setAttribute("aria-label", getLang() === "zh" ? "打开菜单" : "Open menu");
      overlay.setAttribute("aria-hidden", "true");
      document.body.style.overflow = "";
    }

    menuBtn.addEventListener("click", function (e) {
      e.preventDefault();
      e.stopPropagation();
      if (isMobileOpen) closeMobileNav();
      else openMobileNav();
    });

    overlay.addEventListener("click", closeMobileNav);

    document.addEventListener("keydown", function (e) {
      if (e.key === "Escape" && isMobileOpen) {
        closeMobileNav();
        menuBtn.focus();
      }
    });
  }

  // =========================================================================
  // Mobile Sidebar Drawer (< 768px)
  // =========================================================================

  function initMobileSidebar() {
    var sidebar = document.querySelector(".md-sidebar--primary");
    if (!sidebar) return;

    var overlay = document.createElement("div");
    overlay.className = "vf-sidebar-overlay";
    overlay.setAttribute("aria-hidden", "true");
    document.body.appendChild(overlay);

    var isOpen = false;

    function openSidebar() {
      isOpen = true;
      sidebar.classList.add("is-open");
      overlay.classList.add("is-open");
      overlay.setAttribute("aria-hidden", "false");
      document.body.style.overflow = "hidden";
      var firstLink = sidebar.querySelector(".md-nav__link");
      if (firstLink) {
        setTimeout(function () { firstLink.focus(); }, 100);
      }
    }

    function closeSidebar() {
      isOpen = false;
      sidebar.classList.remove("is-open");
      overlay.classList.remove("is-open");
      overlay.setAttribute("aria-hidden", "true");
      document.body.style.overflow = "";
    }

    overlay.addEventListener("click", closeSidebar);

    document.addEventListener("keydown", function (e) {
      if (e.key === "Escape" && isOpen) {
        closeSidebar();
      }
    });

    // Close sidebar when a nav link is clicked on mobile
    sidebar.addEventListener("click", function (e) {
      if (window.innerWidth < 768) {
        var link = e.target.closest(".md-nav__link");
        if (link && link.getAttribute("href") && !link.getAttribute("href").startsWith("#")) {
          setTimeout(closeSidebar, 150);
        }
      }
    });

    // Watch the Material drawer toggle to open our sidebar on mobile
    var drawerToggle = document.querySelector('.md-header__button[for="__drawer"]');
    if (drawerToggle) {
      drawerToggle.addEventListener("click", function (e) {
        if (window.innerWidth < 768) {
          e.preventDefault();
          e.stopPropagation();
          if (isOpen) closeSidebar();
          else openSidebar();
        }
      });
    }
  }

  // =========================================================================
  // Back to Top
  // =========================================================================

  function initBackToTop() {
    var btn = document.createElement("button");
    btn.className = "md-back-to-top";
    btn.setAttribute("aria-label", getLang() === "zh" ? "返回顶部" : "Back to top");
    btn.setAttribute("type", "button");
    btn.setAttribute("data-tooltip", getLang() === "zh" ? "返回顶部" : "Back to top");

    var icon = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    icon.setAttribute("viewBox", "0 0 24 24");
    icon.setAttribute("fill", "none");
    icon.setAttribute("stroke", "currentColor");
    icon.setAttribute("stroke-width", "2.5");
    icon.setAttribute("stroke-linecap", "round");
    icon.setAttribute("stroke-linejoin", "round");
    icon.classList.add("md-back-to-top__icon");
    icon.setAttribute("aria-hidden", "true");
    var path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("d", "M12 19V5M5 12l7-7 7 7");
    icon.appendChild(path);
    btn.appendChild(icon);

    document.body.appendChild(btn);

    var showThreshold = Math.max(500, window.innerHeight);
    var ticking = false;
    var isVisible = false;

    function updateVisibility() {
      var shouldShow = window.scrollY > showThreshold;
      if (shouldShow !== isVisible) {
        isVisible = shouldShow;
        if (shouldShow) {
          btn.classList.add("is-visible");
        } else {
          btn.classList.remove("is-visible");
        }
      }
      ticking = false;
    }

    window.addEventListener("scroll", function () {
      if (!ticking) {
        requestAnimationFrame(updateVisibility);
        ticking = true;
      }
    }, { passive: true });

    var resizeTimer;
    window.addEventListener("resize", function () {
      clearTimeout(resizeTimer);
      resizeTimer = setTimeout(function () {
        showThreshold = Math.max(500, window.innerHeight);
      }, 250);
    }, { passive: true });

    updateVisibility();

    btn.addEventListener("click", function () {
      var behavior = prefersReducedMotion() ? "instant" : "smooth";
      window.scrollTo({ top: 0, behavior: behavior });
      var firstHeading = document.querySelector(".md-content h1");
      if (firstHeading) {
        firstHeading.setAttribute("tabindex", "-1");
        setTimeout(function () {
          firstHeading.focus({ preventScroll: true });
        }, behavior === "smooth" ? 400 : 50);
      }
    });
  }

  // =========================================================================
  // Mobile TOC
  // =========================================================================

  function initMobileTOC() {
    var mediaQuery = window.matchMedia("(max-width: 63.9375em)");
    var tocNav = document.querySelector(".md-nav--secondary");
    if (!tocNav) return;

    var tocItems = collectTOCItems(tocNav);
    if (tocItems.length === 0) return;

    var overlay = createTocOverlay();
    var dropdown = createTocDropdown(tocItems);
    var button = createTOCButton();

    document.body.appendChild(overlay);
    document.body.appendChild(dropdown);
    document.body.appendChild(button);

    var isOpen = false;

    function openToc() {
      isOpen = true;
      overlay.classList.add("is-open");
      dropdown.classList.add("is-open");
      button.setAttribute("aria-expanded", "true");
      var firstLink = dropdown.querySelector(".md-mobile-toc__link");
      if (firstLink) {
        setTimeout(function () { firstLink.focus(); }, 100);
      }
    }

    function closeToc() {
      isOpen = false;
      overlay.classList.remove("is-open");
      dropdown.classList.remove("is-open");
      button.setAttribute("aria-expanded", "false");
    }

    button.addEventListener("click", function (e) {
      e.preventDefault();
      e.stopPropagation();
      if (isOpen) closeToc();
      else openToc();
    });

    button.addEventListener("keydown", function (e) {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        if (isOpen) closeToc();
        else openToc();
      } else if (e.key === "Escape" && isOpen) {
        e.preventDefault();
        closeToc();
        button.focus();
      }
    });

    overlay.addEventListener("click", closeToc);

    document.addEventListener("keydown", function (e) {
      if (e.key === "Escape" && isOpen) {
        closeToc();
        button.focus();
      }
    });

    dropdown.addEventListener("click", function (e) {
      var link = e.target.closest(".md-mobile-toc__link");
      if (!link) return;
      e.preventDefault();
      var targetId = link.getAttribute("href").substring(1);
      var target = document.getElementById(targetId);
      if (!target) return;
      closeToc();
      var behavior = prefersReducedMotion() ? "instant" : "smooth";
      var topOffset = getStickyHeaderHeight();
      var targetTop = target.getBoundingClientRect().top + window.pageYOffset;
      window.scrollTo({ top: targetTop - topOffset, behavior: behavior });
      var currentPath = window.location.pathname + window.location.search;
      history.pushState(null, null, currentPath + "#" + targetId);
    });

    dropdown.addEventListener("keydown", function (e) {
      var items = Array.from(dropdown.querySelectorAll(".md-mobile-toc__link"));
      var currentIndex = items.indexOf(document.activeElement);
      if (e.key === "ArrowDown" || e.key === "ArrowUp") {
        e.preventDefault();
        var nextIndex;
        if (e.key === "ArrowDown") {
          nextIndex = (currentIndex + 1) % items.length;
        } else {
          nextIndex = (currentIndex - 1 + items.length) % items.length;
        }
        items[nextIndex].focus();
      } else if (e.key === "Escape") {
        closeToc();
        button.focus();
      }
    });

    function handleMediaChange(mql) {
      if (mql.matches) {
        button.style.display = "flex";
      } else {
        button.style.display = "none";
        closeToc();
      }
    }

    mediaQuery.addEventListener("change", handleMediaChange);
    handleMediaChange(mediaQuery);

    window.addEventListener("md-toc-active-change", function (e) {
      if (e.detail && e.detail.id) {
        updateTocActiveState(e.detail.id, e.detail.text, button, dropdown);
      }
    });

    window.__updateMobileTOC = function (activeId, activeText) {
      updateTocActiveState(activeId, activeText, button, dropdown);
    };
  }

  function collectTOCItems(tocNav) {
    var items = [];
    var links = tocNav.querySelectorAll('.md-nav__list[data-md-component="toc"] .md-nav__link');
    links.forEach(function (link) {
      var href = link.getAttribute("href");
      if (href && href.startsWith("#")) {
        items.push({ id: href.substring(1), text: link.textContent.trim(), level: 2 });
      }
    });
    return items;
  }

  function createTocOverlay() {
    var el = document.createElement("div");
    el.className = "md-mobile-toc__overlay";
    el.setAttribute("aria-hidden", "true");
    return el;
  }

  function createTocDropdown(items) {
    var el = document.createElement("div");
    el.className = "md-mobile-toc__dropdown";
    el.setAttribute("role", "menu");
    el.setAttribute("aria-label", getLang() === "zh" ? "本页目录" : "On this page");
    var title = document.createElement("div");
    title.className = "md-mobile-toc__dropdown-title";
    title.textContent = getLang() === "zh" ? "本页目录" : "On this page";
    el.appendChild(title);
    items.forEach(function (item) {
      var link = document.createElement("a");
      link.className = "md-mobile-toc__link";
      link.href = "#" + item.id;
      link.textContent = item.text;
      link.setAttribute("role", "menuitem");
      link.setAttribute("tabindex", "-1");
      el.appendChild(link);
    });
    return el;
  }

  function createTOCButton() {
    var el = document.createElement("button");
    el.className = "md-mobile-toc";
    el.setAttribute("aria-label", getLang() === "zh" ? "本页目录" : "On this page");
    el.setAttribute("aria-haspopup", "true");
    el.setAttribute("aria-expanded", "false");
    el.setAttribute("type", "button");

    var icon = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    icon.setAttribute("viewBox", "0 0 24 24");
    icon.setAttribute("fill", "none");
    icon.setAttribute("stroke", "currentColor");
    icon.setAttribute("stroke-width", "2");
    icon.setAttribute("stroke-linecap", "round");
    icon.setAttribute("stroke-linejoin", "round");
    icon.classList.add("md-mobile-toc__icon");
    var path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("d", "M4 6h16M4 12h16M4 18h16");
    icon.appendChild(path);
    el.appendChild(icon);

    var label = document.createElement("span");
    label.className = "md-mobile-toc__label";
    label.textContent = getLang() === "zh" ? "本页目录" : "On this page";
    el.appendChild(label);
    return el;
  }

  function updateTocActiveState(activeId, activeText, button, dropdown) {
    var label = button.querySelector(".md-mobile-toc__label");
    if (label && activeText) {
      label.textContent = activeText;
    }
    var links = dropdown.querySelectorAll(".md-mobile-toc__link");
    links.forEach(function (link) {
      var href = link.getAttribute("href");
      if (href === "#" + activeId) {
        link.classList.add("md-mobile-toc__link--active");
      } else {
        link.classList.remove("md-mobile-toc__link--active");
      }
    });
  }

  // =========================================================================
  // Scroll Spy — IntersectionObserver for heading visibility
  // =========================================================================

  function initScrollSpy() {
    var headings = document.querySelectorAll(".md-content :is(h1[id], h2[id], h3[id])");
    if (headings.length === 0) return;

    var activeId = null;
    var headingMap = {};
    var headingArray = [];

    headings.forEach(function (h) {
      var info = { id: h.id, text: h.textContent.trim(), level: parseInt(h.tagName.substring(1), 10), el: h };
      headingMap[h.id] = info;
      headingArray.push(info);
    });

    function getSidebarTOCLinks() {
      var map = {};
      // Check both secondary sidebar and primary sidebar for TOC links
      var tocNavs = document.querySelectorAll(".md-nav--secondary");
      tocNavs.forEach(function (tocNav) {
        var links = tocNav.querySelectorAll(".md-nav__link");
        links.forEach(function (link) {
          var href = link.getAttribute("href");
          if (href && href.startsWith("#")) {
            map[href.substring(1)] = link;
          }
        });
      });
      return map;
    }

    var observerOptions = {
      rootMargin: "-" + (getStickyHeaderHeight() + 20) + "px 0px -40% 0px",
      threshold: 0,
    };

    var observer = new IntersectionObserver(function (entries) {
      var visibleEntries = entries.filter(function (e) { return e.isIntersecting; });

      if (visibleEntries.length === 0) {
        var firstHeading = headingArray[0];
        if (firstHeading && firstHeading.el.getBoundingClientRect().bottom < getStickyHeaderHeight()) {
          var bestHeading = null;
          var bestDist = Infinity;
          entries.forEach(function (e) {
            var dist = Math.abs(e.boundingClientRect.bottom - getStickyHeaderHeight());
            if (e.boundingClientRect.bottom > 0 && dist < bestDist) {
              bestDist = dist;
              bestHeading = e.target;
            }
          });
          if (!bestHeading && entries.length > 0) {
            var lastEntry = entries[entries.length - 1];
            if (lastEntry.boundingClientRect.top < window.innerHeight) {
              bestHeading = lastEntry.target;
            }
          }
          if (bestHeading) {
            visibleEntries = [{ target: bestHeading, isIntersecting: true }];
          }
        }
        if (visibleEntries.length === 0) return;
      }

      var sorted = visibleEntries.sort(function (a, b) {
        return a.boundingClientRect.top - b.boundingClientRect.top;
      });

      var firstVisible = sorted[0].target;
      var newActiveId = firstVisible.id;

      if (newActiveId !== activeId) {
        activeId = newActiveId;
        var info = headingMap[activeId];

        var tocLinks = getSidebarTOCLinks();
        Object.keys(tocLinks).forEach(function (key) {
          var link = tocLinks[key];
          if (key === activeId) {
            link.setAttribute("aria-current", "location");
          } else {
            link.removeAttribute("aria-current");
          }
        });

        var event = new CustomEvent("md-toc-active-change", {
          detail: { id: activeId, text: info ? info.text : "", level: info ? info.level : 2 },
        });
        window.dispatchEvent(event);

        if (window.__updateMobileTOC && info) {
          window.__updateMobileTOC(info.id, info.text);
        }
      }
    }, observerOptions);

    headings.forEach(function (h) { observer.observe(h); });

    // Initial hash-based activation
    setTimeout(function () {
      var hashId = window.location.hash ? window.location.hash.substring(1) : null;
      if (hashId && headingMap[hashId]) {
        activeId = hashId;
        var info = headingMap[hashId];
        var tocLinks = getSidebarTOCLinks();
        Object.keys(tocLinks).forEach(function (key) {
          tocLinks[key].setAttribute("aria-current", key === activeId ? "location" : null);
        });
        if (window.__updateMobileTOC && info) {
          window.__updateMobileTOC(info.id, info.text);
        }
      }
    }, 200);
  }

  // =========================================================================
  // URL Hash Handling
  // =========================================================================

  function initHashHandling() {
    if (window.location.hash) {
      var targetId = window.location.hash.substring(1);
      setTimeout(function () { scrollToHash(targetId); }, 150);
    }
    window.addEventListener("popstate", function () {
      if (window.location.hash) {
        var targetId = window.location.hash.substring(1);
        scrollToHash(targetId);
      }
    });
  }

  function scrollToHash(targetId) {
    if (!targetId) return;
    var target = document.getElementById(targetId);
    if (!target) return;
    var behavior = prefersReducedMotion() ? "instant" : "smooth";
    var topOffset = getStickyHeaderHeight();
    var targetTop = target.getBoundingClientRect().top + window.pageYOffset;
    window.scrollTo({ top: targetTop - topOffset, behavior: behavior });
    target.setAttribute("tabindex", "-1");
    target.focus({ preventScroll: true });
  }

  // =========================================================================
  // Reduced Motion Observer
  // =========================================================================

  function initReducedMotionObserver() {
    var mediaQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
    mediaQuery.addEventListener("change", function () {});
  }

  // =========================================================================
  // Code Block Enhancements
  // =========================================================================

  function initCodeBlocks() {
    // Add language labels to code blocks (where not already present)
    var codeBlocks = document.querySelectorAll(".md-typeset pre > code");
    codeBlocks.forEach(function (code) {
      var pre = code.parentElement;
      if (pre.querySelector(".vf-code-lang")) return; // already labeled

      // Detect language from class
      var classes = code.className.split(/\s+/);
      var langClass = classes.find(function (c) { return c.startsWith("language-") || c.startsWith("lang-"); });
      if (langClass) {
        var lang = langClass.replace(/^(language-|lang-)/, "");
        var label = document.createElement("div");
        label.className = "vf-code-lang";
        label.textContent = lang;
        label.setAttribute("aria-hidden", "true");
        // Add label styles dynamically
        label.style.cssText =
          "position:absolute;top:0;right:3rem;padding:0.15rem 0.6rem;" +
          "font-size:0.7rem;font-weight:600;text-transform:uppercase;" +
          "color:var(--vf-body-secondary);background:transparent;" +
          "pointer-events:none;z-index:2;letter-spacing:0.03em;";
        pre.style.position = "relative";
        pre.insertBefore(label, pre.firstChild);
      }
    });

    // Enhance copy buttons
    enhanceCopyButtons();
  }

  function enhanceCopyButtons() {
    // Material for MkDocs already provides copy buttons.
    // We add custom feedback text "Copied!" on click.
    document.addEventListener("click", function (e) {
      var btn = e.target.closest(".md-clipboard");
      if (!btn) return;

      // Let Material handle the actual copy; we add visual feedback
      var originalText = btn.textContent;
      btn.textContent = getLang() === "zh" ? "已复制" : "Copied!";
      btn.style.color = "#2e7d32";
      setTimeout(function () {
        btn.textContent = originalText;
        btn.style.color = "";
      }, 2000);
    });
  }

  // =========================================================================
  // File Reference Cards — Copy path functionality
  // =========================================================================

  function initFileCards() {
    document.addEventListener("click", function (e) {
      var btn = e.target.closest(".vf-file-card__btn");
      if (!btn) return;

      if (btn.hasAttribute("data-copy-path")) {
        e.preventDefault();
        var path = btn.getAttribute("data-copy-path");
        copyToClipboard(path, btn);
      }
    });
  }

  function copyToClipboard(text, btn) {
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(function () {
        showCopied(btn);
      }).catch(function () {
        fallbackCopy(text, btn);
      });
    } else {
      fallbackCopy(text, btn);
    }
  }

  function fallbackCopy(text, btn) {
    var textarea = document.createElement("textarea");
    textarea.value = text;
    textarea.style.position = "fixed";
    textarea.style.opacity = "0";
    document.body.appendChild(textarea);
    textarea.select();
    try {
      document.execCommand("copy");
      showCopied(btn);
    } catch (err) {
      // silent
    }
    document.body.removeChild(textarea);
  }

  function showCopied(btn) {
    var originalText = btn.textContent;
    btn.textContent = getLang() === "zh" ? "✓ 已复制" : "✓ Copied";
    btn.classList.add("vf-file-card__btn--copied");
    setTimeout(function () {
      btn.textContent = originalText;
      btn.classList.remove("vf-file-card__btn--copied");
    }, 2000);
  }

  // =========================================================================
  // Page Feedback
  // =========================================================================

  function initPageFeedback() {
    var contentArea = document.querySelector(".md-content__inner");
    if (!contentArea) return;

    var isEn = getLang() === "en";

    var feedback = document.createElement("div");
    feedback.className = "vf-feedback";
    feedback.innerHTML =
      '<span class="vf-feedback__question">' +
      (isEn ? "Was this page helpful?" : "此页面是否有帮助？") +
      "</span>" +
      '<div class="vf-feedback__actions">' +
      '<button class="vf-feedback__btn" data-feedback="yes" aria-label="' +
      (isEn ? "Yes, this page was helpful" : "是，此页面有帮助") +
      '">' +
      (isEn ? "👍 Yes" : "👍 是") +
      "</button>" +
      '<button class="vf-feedback__btn" data-feedback="no" aria-label="' +
      (isEn ? "No, this page needs improvement" : "否，需要改进") +
      '">' +
      (isEn ? "👎 No" : "👎 否") +
      "</button>" +
      "</div>" +
      '<span class="vf-feedback__thanks" style="display:none">' +
      (isEn ? "Thank you for your feedback!" : "感谢你的反馈！") +
      "</span>";

    contentArea.appendChild(feedback);

    feedback.addEventListener("click", function (e) {
      var btn = e.target.closest(".vf-feedback__btn");
      if (!btn) return;

      // Hide buttons, show thanks
      var actions = feedback.querySelector(".vf-feedback__actions");
      var thanks = feedback.querySelector(".vf-feedback__thanks");
      if (actions) actions.style.display = "none";
      if (thanks) thanks.style.display = "inline";
    });
  }

  // =========================================================================
  // Mermaid Diagram Lightbox Viewer
  //
  // IMPORTANT: This module NEVER calls mermaid.initialize(), mermaid.run(),
  // mermaid.init(), or mermaid.contentLoaded().
  //
  // Material for MkDocs natively handles Mermaid rendering:
  //   1. Detects <pre class="mermaid"><code>...</code></pre> in the DOM
  //   2. Dynamically loads mermaid.min.js from CDN if not already loaded
  //   3. Calls mermaid.initialize({startOnLoad: false, ...})
  //   4. Calls mermaid.render() for each diagram
  //   5. Injects the rendered SVG into <div class="mermaid"> shadow DOM
  //
  // This module only wraps the already-rendered SVGs with a lightbox preview.
  // It retries with backoff because Material loads mermaid asynchronously.
  // =========================================================================

  // Module-level state — persists across SPA navigations
  var _mvLightbox = null;
  var _mvStage = null;
  var _mvDiagram = null;
  var _mvCurSvg = null;
  var _mvScale = 1;
  var _mvTx = 0;
  var _mvTy = 0;
  var _mvScrollY = 0;
  var _mvMinScale = 0.3;
  var _mvMaxScale = 10;
  var _mvZoomStep = 0.25;
  var _mvFitPending = false;
  var _mvWrapTimer = null; // retry timer handle

  /**
   * Find a rendered SVG inside a .mermaid element.
   * Checks light DOM first, then shadow DOM (Material for MkDocs uses
   * shadow DOM to encapsulate Mermaid's rendered SVG).
   */
  function findSvgInMermaid(host) {
    return (
      host.querySelector("svg") ||
      host.shadowRoot?.querySelector("svg") ||
      window.__shadowRootRegistry?.get(host)?.querySelector("svg") ||
      null
    );
  }

  /**
   * Retry wrapping Mermaid diagrams until SVGs appear or timeout.
   *
   * Material loads mermaid.min.js asynchronously on first use and renders
   * diagrams inside a document$ subscription. This races with our own
   * document$ subscription — poll with backoff until SVGs are ready.
   */
  function tryWrapMermaid(attempt) {
    var MAX_ATTEMPTS = 50; // 50 × 100ms = 5 seconds
    var els = document.querySelectorAll(".mermaid");
    if (els.length === 0) return;

    var hasRenderable = false;
    for (var i = 0; i < els.length; i++) {
      var svg = findSvgInMermaid(els[i]);
      if (svg && svg.getAttribute("aria-roledescription") !== "error") {
        hasRenderable = true;
        break;
      }
    }

    if (hasRenderable) {
      initMermaidViewer();
    } else if (attempt < MAX_ATTEMPTS) {
      _mvWrapTimer = setTimeout(function () { tryWrapMermaid(attempt + 1); }, 100);
    }
  }

  function initMermaidViewer() {
    if (_mvWrapTimer) { clearTimeout(_mvWrapTimer); _mvWrapTimer = null; }
    wrapDiagrams();
    if (!_mvLightbox) createLightbox();
  }

  function wrapDiagrams() {
    var els = document.querySelectorAll(".mermaid");
    for (var i = 0; i < els.length; i++) {
      var el = els[i];
      if (el.closest(".vf-mermaid-wrapper")) continue;
      var svg = findSvgInMermaid(el);
      if (!svg) continue;
      if (svg.getAttribute("aria-roledescription") === "error") continue;

      svg.setAttribute("role", "img");
      var firstLine = "";
      try {
        // Material replaces <pre> with <div> — try to read source from
        // the original pre's textContent stored on the element, or from
        // the SVG's aria-label set by Material.
        firstLine = el.getAttribute("data-mermaid-source") || "";
      } catch (e) { /* ignore */ }
      var ariaLabel = svg.getAttribute("aria-label") || firstLine || "Diagram";
      if (!svg.getAttribute("aria-label")) {
        svg.setAttribute("aria-label", ariaLabel);
      }

      var wrapper = document.createElement("div");
      wrapper.className = "vf-mermaid-wrapper";
      wrapper.setAttribute("tabindex", "0");
      wrapper.setAttribute("role", "figure");
      wrapper.setAttribute("aria-label", ariaLabel);

      var hint = document.createElement("div");
      hint.className = "vf-mermaid-hint";
      hint.setAttribute("aria-hidden", "true");
      var lang = getLang();
      hint.innerHTML =
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">' +
        '<circle cx="11" cy="11" r="8"/><path d="M21 21l-4.35-4.35"/>' +
        '<path d="M11 8v6"/><path d="M8 11h6"/>' +
        "</svg>" +
        (lang === "zh" ? "点击查看大图，可缩放和拖动" : "Click to zoom; pinch and drag to explore");

      el.parentNode.insertBefore(wrapper, el);
      wrapper.appendChild(el);
      wrapper.appendChild(hint);

      wrapper.addEventListener("click", function () {
        openLightbox(el, ariaLabel);
      });
      wrapper.addEventListener("keydown", function (e) {
        if (e.key === "Enter" || e.key === " ") {
          e.preventDefault();
          openLightbox(el, ariaLabel);
        }
      });
    }
  }

  function createLightbox() {
    if (document.querySelector(".vf-lightbox")) return;
    _mvScrollY = 0;

    _mvLightbox = document.createElement("div");
    _mvLightbox.className = "vf-lightbox";
    _mvLightbox.setAttribute("role", "dialog");
    _mvLightbox.setAttribute("aria-modal", "true");
    _mvLightbox.setAttribute("aria-label", getLang() === "zh" ? "图表预览" : "Diagram preview");

    var toolbar = document.createElement("div");
    toolbar.className = "vf-lightbox-toolbar";
    toolbar.setAttribute("role", "toolbar");
    toolbar.setAttribute("aria-label", getLang() === "zh" ? "缩放控制" : "Zoom controls");

    function makeBtn(iconPaths, label, action) {
      var btn = document.createElement("button");
      btn.className = "vf-lightbox-btn";
      btn.setAttribute("type", "button");
      btn.setAttribute("aria-label", label);
      btn.setAttribute("title", label);
      btn.innerHTML =
        '<svg viewBox="0 0 24 24">' +
        iconPaths.map(function (d) { return '<path d="' + d + '"/>'; }).join("") +
        "</svg>";
      btn.addEventListener("click", function (e) {
        e.stopPropagation();
        action();
      });
      return btn;
    }

    var isZh = getLang() === "zh";

    toolbar.appendChild(makeBtn(
      ["M12 5v14M5 12h14"], isZh ? "放大" : "Zoom in",
      function () { zoomStep(_mvZoomStep); }
    ));
    toolbar.appendChild(makeBtn(
      ["M5 12h14"], isZh ? "缩小" : "Zoom out",
      function () { zoomStep(-_mvZoomStep); }
    ));
    toolbar.appendChild(makeBtn(
      ["M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4"],
      isZh ? "适应窗口" : "Fit to screen",
      function () { fitToScreen(); }
    ));
    toolbar.appendChild(makeBtn(
      ["M3 6h18M3 6v12M3 12h18M3 18h18"],
      isZh ? "原始大小" : "Reset 100%",
      function () { resetZoom(); }
    ));

    var closeBtn = document.createElement("button");
    closeBtn.className = "vf-lightbox-btn vf-lightbox-btn--close";
    closeBtn.setAttribute("type", "button");
    closeBtn.setAttribute("aria-label", isZh ? "关闭" : "Close");
    closeBtn.setAttribute("title", isZh ? "关闭 (Esc)" : "Close (Esc)");
    closeBtn.innerHTML = '<svg viewBox="0 0 24 24"><path d="M18 6L6 18"/><path d="M6 6l12 12"/></svg>';
    closeBtn.addEventListener("click", function (e) { e.stopPropagation(); closeLightbox(); });
    toolbar.appendChild(closeBtn);

    _mvStage = document.createElement("div");
    _mvStage.className = "vf-lightbox-stage";

    // Loading indicator
    var loadingEl = document.createElement("div");
    loadingEl.className = "vf-lightbox-status vf-lightbox-loading";
    loadingEl.setAttribute("aria-live", "polite");
    loadingEl.innerHTML =
      '<div class="vf-lightbox-spinner"></div>' +
      '<span class="vf-lightbox-status-text">' +
      (isZh ? "加载中…" : "Loading…") +
      "</span>";

    // Error state
    var errorEl = document.createElement("div");
    errorEl.className = "vf-lightbox-status vf-lightbox-error";
    errorEl.setAttribute("aria-live", "assertive");
    errorEl.style.display = "none";
    errorEl.innerHTML =
      '<svg class="vf-lightbox-error-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">' +
      '<circle cx="12" cy="12" r="10"/>' +
      '<path d="M12 8v4M12 16h.01"/>' +
      "</svg>" +
      '<span class="vf-lightbox-status-text">' +
      (isZh ? "图片加载失败" : "Image failed to load") +
      "</span>" +
      '<button class="vf-lightbox-retry-btn" type="button">' +
      (isZh ? "重试" : "Retry") +
      "</button>";

    _mvDiagram = document.createElement("div");
    _mvDiagram.className = "vf-lightbox-diagram";

    _mvStage.appendChild(loadingEl);
    _mvStage.appendChild(errorEl);
    _mvStage.appendChild(_mvDiagram);
    _mvLightbox.appendChild(toolbar);
    _mvLightbox.appendChild(_mvStage);
    document.body.appendChild(_mvLightbox);

    // Wire retry button
    var retryBtn = errorEl.querySelector(".vf-lightbox-retry-btn");
    retryBtn.addEventListener("click", function (e) {
      e.stopPropagation();
      if (_mvCurSvg) {
        var lbl = _mvCurSvg.getAttribute("aria-label") || "";
        openLightboxContent(_mvCurSvg, lbl);
      }
    });

    _mvStage.addEventListener("click", function (e) {
      if (e.target === _mvStage) closeLightbox();
    });

    // Mouse wheel zoom
    _mvStage.addEventListener("wheel", function (e) {
      e.preventDefault();
      var rect = _mvStage.getBoundingClientRect();
      var cx = e.clientX - rect.left - rect.width / 2;
      var cy = e.clientY - rect.top - rect.height / 2;
      zoomAt(_mvScale + (e.deltaY > 0 ? -_mvZoomStep : _mvZoomStep), cx, cy);
    }, { passive: false });

    // Double-click zoom
    _mvStage.addEventListener("dblclick", function (e) {
      e.preventDefault();
      var rect = _mvStage.getBoundingClientRect();
      var cx = e.clientX - rect.left - rect.width / 2;
      var cy = e.clientY - rect.top - rect.height / 2;
      if (Math.abs(_mvScale - 1) < 0.05) { zoomAt(2.5, cx, cy); }
      else { resetZoom(); }
    });

    // Drag to pan
    var panStartX = 0, panStartY = 0, panTx = 0, panTy = 0, isPanning = false;

    function onPanStart(e) {
      isPanning = true;
      var cx = e.touches ? e.touches[0].clientX : e.clientX;
      var cy = e.touches ? e.touches[0].clientY : e.clientY;
      panStartX = cx; panStartY = cy;
      panTx = _mvTx; panTy = _mvTy;
      _mvStage.classList.add("is-grabbing");
      _mvStage.classList.remove("is-transitioning");
      _mvDiagram.classList.remove("is-transitioning");
    }
    function onPanMove(e) {
      if (!isPanning) return;
      e.preventDefault();
      var cx = e.touches ? e.touches[0].clientX : e.clientX;
      var cy = e.touches ? e.touches[0].clientY : e.clientY;
      _mvTx = panTx + (cx - panStartX);
      _mvTy = panTy + (cy - panStartY);
      applyTransform(false);
    }
    function onPanEnd() { isPanning = false; _mvStage.classList.remove("is-grabbing"); }

    _mvStage.addEventListener("mousedown", onPanStart);
    _mvStage.addEventListener("touchstart", onPanStart, { passive: true });
    document.addEventListener("mousemove", onPanMove);
    document.addEventListener("touchmove", onPanMove, { passive: false });
    document.addEventListener("mouseup", onPanEnd);
    document.addEventListener("touchend", onPanEnd);

    // Touch pinch zoom
    var pinchStartDist = 0, pinchStartScale = 1, pinchMidX = 0, pinchMidY = 0;
    _mvStage.addEventListener("touchstart", function (e) {
      if (e.touches.length === 2) {
        isPanning = false;
        pinchStartDist = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
        pinchStartScale = _mvScale;
        pinchMidX = (e.touches[0].clientX + e.touches[1].clientX) / 2;
        pinchMidY = (e.touches[0].clientY + e.touches[1].clientY) / 2;
      }
    }, { passive: true });
    _mvStage.addEventListener("touchmove", function (e) {
      if (e.touches.length === 2 && pinchStartDist > 0) {
        e.preventDefault();
        var dist = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
        var newScale = Math.min(_mvMaxScale, Math.max(_mvMinScale, pinchStartScale * (dist / pinchStartDist)));
        var rect = _mvStage.getBoundingClientRect();
        var cx = pinchMidX - rect.left - rect.width / 2;
        var cy = pinchMidY - rect.top - rect.height / 2;
        _mvScale = newScale;
        _mvTx = cx - (cx - _mvTx) * (newScale / pinchStartScale);
        _mvTy = cy - (cy - _mvTy) * (newScale / pinchStartScale);
        applyTransform(false);
      }
    }, { passive: false });

    // Keyboard
    document.addEventListener("keydown", function (e) {
      if (!_mvLightbox || !_mvLightbox.classList.contains("is-open")) return;
      if (e.key === "Escape") { e.preventDefault(); closeLightbox(); }
      else if (e.key === "+" || e.key === "=") { e.preventDefault(); zoomStep(_mvZoomStep); }
      else if (e.key === "-") { e.preventDefault(); zoomStep(-_mvZoomStep); }
      else if (e.key === "0") { e.preventDefault(); resetZoom(); }
      else if (e.key === "f" || e.key === "F") { e.preventDefault(); fitToScreen(); }
    });
  }

  /**
   * Open lightbox with the page's existing SVG diagram.
   *
   * CRITICAL: Only clones the already-rendered SVG from the page.
   * NEVER calls any Mermaid API. Material for MkDocs renders Mermaid
   * diagrams into shadow DOM — findSvgInMermaid handles both light
   * and shadow DOM queries.
   */
  function openLightbox(mermaidEl, ariaLabel) {
    if (!_mvLightbox) return;

    var svg = findSvgInMermaid(mermaidEl);
    if (!svg) return;

    openLightboxContent(svg, ariaLabel);
  }

  /**
   * Core: insert an SVG clone into the lightbox.
   */
  function openLightboxContent(svgEl, ariaLabel) {
    if (!_mvLightbox) return;
    _mvScrollY = window.scrollY;
    _mvCurSvg = svgEl;

    showLightboxLoading();

    _mvLightbox.classList.add("is-open");
    document.body.classList.add("vf-lightbox-open");

    // Clone the page SVG — NO Mermaid API calls
    var clone = svgEl.cloneNode(true);
    clone.removeAttribute("width");
    clone.removeAttribute("height");
    clone.style.maxWidth = "none";
    clone.style.maxHeight = "none";
    clone.style.width = "100%";
    clone.style.height = "100%";
    clone.style.display = "block";
    clone.style.margin = "0 auto";
    clone.classList.remove("mermaid");
    clone.removeAttribute("data-processed");
    if (ariaLabel) {
      clone.setAttribute("aria-label", ariaLabel);
    }

    _mvDiagram.innerHTML = "";
    _mvDiagram.appendChild(clone);

    _mvLightbox.setAttribute(
      "aria-label",
      getLang() === "zh" ? "图表预览：" + (ariaLabel || "") : "Diagram preview: " + (ariaLabel || "")
    );

    requestAnimationFrame(function () {
      var attempts = 0;
      function tryFit() {
        var svg = _mvDiagram.querySelector("svg");
        if (!svg) {
          showLightboxError("SVG element missing after clone");
          return;
        }
        var svgW = getSvgWidth(svg);
        var svgH = getSvgHeight(svg);
        if ((!svgW || !svgH || svgW === 0 || svgH === 0) && attempts < 20) {
          attempts++;
          requestAnimationFrame(tryFit);
          return;
        }
        hideLightboxLoading();
        fitToScreen();
        var closeBtn = _mvLightbox.querySelector(".vf-lightbox-btn--close");
        if (closeBtn) closeBtn.focus();
      }
      tryFit();
    });
  }

  function closeLightbox() {
    if (!_mvLightbox) return;
    _mvLightbox.classList.remove("is-open");
    document.body.classList.remove("vf-lightbox-open");
    _mvDiagram.innerHTML = "";
    _mvCurSvg = null;
    _mvScale = 1;
    _mvTx = 0;
    _mvTy = 0;
    _mvFitPending = false;
    hideLightboxLoading();
    hideLightboxError();
    requestAnimationFrame(function () {
      window.scrollTo({ top: _mvScrollY, behavior: "instant" });
    });
  }

  // ----- Loading / Error state helpers -----

  function showLightboxLoading() {
    var loadingEl = _mvStage ? _mvStage.querySelector(".vf-lightbox-loading") : null;
    if (loadingEl) loadingEl.style.display = "";
    hideLightboxError();
    if (_mvDiagram) {
      _mvDiagram.style.opacity = "0";
      _mvDiagram.style.pointerEvents = "none";
    }
  }

  function hideLightboxLoading() {
    var loadingEl = _mvStage ? _mvStage.querySelector(".vf-lightbox-loading") : null;
    if (loadingEl) loadingEl.style.display = "none";
    if (_mvDiagram) {
      _mvDiagram.style.opacity = "";
      _mvDiagram.style.pointerEvents = "";
    }
  }

  function showLightboxError(msg) {
    hideLightboxLoading();
    if (_mvDiagram) {
      _mvDiagram.innerHTML = "";
      _mvDiagram.style.opacity = "";
      _mvDiagram.style.pointerEvents = "";
    }
    var errorEl = _mvStage ? _mvStage.querySelector(".vf-lightbox-error") : null;
    if (errorEl) {
      errorEl.style.display = "";
    }
    if (msg) {
      console.error("[VF Lightbox] " + msg);
    }
  }

  function hideLightboxError() {
    var errorEl = _mvStage ? _mvStage.querySelector(".vf-lightbox-error") : null;
    if (errorEl) {
      errorEl.style.display = "none";
    }
  }

  // ----- SVG dimension helpers -----

  function getSvgWidth(svg) {
    if (svg.viewBox && svg.viewBox.baseVal && svg.viewBox.baseVal.width > 0) {
      return svg.viewBox.baseVal.width;
    }
    var wAttr = svg.getAttribute("width");
    if (wAttr && parseFloat(wAttr) > 0) {
      return parseFloat(wAttr);
    }
    var rect = svg.getBoundingClientRect();
    if (rect && rect.width > 0) return rect.width;
    return 0;
  }

  function getSvgHeight(svg) {
    if (svg.viewBox && svg.viewBox.baseVal && svg.viewBox.baseVal.height > 0) {
      return svg.viewBox.baseVal.height;
    }
    var hAttr = svg.getAttribute("height");
    if (hAttr && parseFloat(hAttr) > 0) {
      return parseFloat(hAttr);
    }
    var rect = svg.getBoundingClientRect();
    if (rect && rect.height > 0) return rect.height;
    return 0;
  }

  // ----- Zoom / Transform -----

  function zoomStep(delta) { zoomAt(_mvScale + delta, 0, 0); }

  function zoomAt(newScale, cx, cy) {
    var oldScale = _mvScale;
    _mvScale = Math.min(_mvMaxScale, Math.max(_mvMinScale, newScale));
    var ratio = _mvScale / oldScale;
    _mvTx = cx - (cx - _mvTx) * ratio;
    _mvTy = cy - (cy - _mvTy) * ratio;
    applyTransform(true);
  }

  function fitToScreen() {
    if (!_mvDiagram || !_mvStage) return;
    var svg = _mvDiagram.querySelector("svg");
    if (!svg) return;

    if (_mvFitPending) return;
    _mvFitPending = true;
    requestAnimationFrame(function () {
      _mvFitPending = false;
      _doFitToScreen(svg);
    });
  }

  function _doFitToScreen(svg) {
    var stageW = _mvStage.clientWidth;
    var stageH = _mvStage.clientHeight;
    var svgW = getSvgWidth(svg);
    var svgH = getSvgHeight(svg);

    if (!stageW || !stageH || stageW === 0 || stageH === 0) return;

    if (!svgW || !svgH || svgW === 0 || svgH === 0) {
      svgW = 800;
      svgH = 600;
    }

    _mvScale = Math.min((stageW * 0.9) / svgW, (stageH * 0.85) / svgH, 1.5);
    if (!isFinite(_mvScale) || _mvScale <= 0) {
      _mvScale = 1;
    }
    _mvTx = 0;
    _mvTy = 0;
    applyTransform(true);
  }

  function resetZoom() {
    _mvScale = 1;
    _mvTx = 0;
    _mvTy = 0;
    applyTransform(true);
  }

  function applyTransform(animate) {
    if (!_mvDiagram) return;
    if (animate) {
      _mvDiagram.classList.add("is-transitioning");
      clearTimeout(_mvDiagram._transitionTimer);
      _mvDiagram._transitionTimer = setTimeout(function () {
        _mvDiagram.classList.remove("is-transitioning");
      }, 260);
    }
    _mvDiagram.style.transform = "translate(" + _mvTx + "px, " + _mvTy + "px) scale(" + _mvScale + ")";
  }

  // =========================================================================
  // Page Meta — inject page description (summary below H1)
  // =========================================================================

  function injectPageMeta() {
    var contentArea = document.querySelector(".md-content__inner");
    if (!contentArea) return;

    var h1 = contentArea.querySelector("h1");
    if (!h1) return;

    // Extract first paragraph as description (blockquote or paragraph after h1)
    var nextEl = h1.nextElementSibling;
    if (nextEl && nextEl.tagName === "BLOCKQUOTE") {
      var desc = document.createElement("div");
      desc.className = "md-page-description";
      var text = nextEl.textContent.trim();
      // Truncate long descriptions
      if (text.length > 200) text = text.substring(0, 197) + "...";
      desc.textContent = text;
      h1.insertAdjacentElement("afterend", desc);
    }
  }
})();
