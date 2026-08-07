/**
 * Click-to-zoom for Mermaid diagrams.
 *
 * Mermaid renders inline <svg>, which medium-zoom (img-only) can't handle.
 * This adds a full-screen overlay with drag-to-pan and wheel/button zoom.
 * Framework-free, idempotent, and safe to call on every route change.
 */

const OVERLAY_ID = "mermaid-zoom-overlay";

function buildOverlay() {
  if (document.getElementById(OVERLAY_ID)) {
    return document.getElementById(OVERLAY_ID);
  }
  const overlay = document.createElement("div");
  overlay.id = OVERLAY_ID;
  overlay.setAttribute("role", "dialog");
  overlay.setAttribute("aria-modal", "true");
  overlay.setAttribute("aria-label", "图表预览");
  overlay.hidden = true;
  overlay.innerHTML = `
    <div class="mz-stage" data-mz-stage>
      <div class="mz-canvas" data-mz-canvas></div>
    </div>
    <div class="mz-toolbar" role="toolbar" aria-label="缩放控制">
      <button type="button" data-mz="zoom-out" title="缩小" aria-label="缩小">−</button>
      <button type="button" data-mz="reset" title="重置" aria-label="重置">↺</button>
      <button type="button" data-mz="zoom-in" title="放大" aria-label="放大">+</button>
      <button type="button" data-mz="close" title="关闭 (Esc)" aria-label="关闭">✕</button>
    </div>
    <div class="mz-hint">滚轮缩放 · 拖拽平移 · Esc 关闭</div>
  `;
  document.body.appendChild(overlay);
  return overlay;
}

export function setupMermaidZoom() {
  if (typeof document === "undefined") return;

  const diagrams = document.querySelectorAll(".vp-doc .mermaid svg");
  if (!diagrams.length) return;

  const overlay = buildOverlay();
  const canvas = overlay.querySelector("[data-mz-canvas]");
  const stage = overlay.querySelector("[data-mz-stage]");

  const state = { scale: 1, x: 0, y: 0, dragging: false, sx: 0, sy: 0 };

  const apply = () => {
    canvas.style.transform = `translate(${state.x}px, ${state.y}px) scale(${state.scale})`;
  };
  const reset = () => {
    state.scale = 1;
    state.x = 0;
    state.y = 0;
    apply();
  };

  const open = (svg) => {
    canvas.innerHTML = "";
    const clone = svg.cloneNode(true);
    clone.removeAttribute("style"); // drop max-width so it can grow
    clone.style.width = "auto";
    clone.style.height = "auto";
    canvas.appendChild(clone);
    reset();
    overlay.hidden = false;
    document.documentElement.style.overflow = "hidden";
  };
  const close = () => {
    overlay.hidden = true;
    canvas.innerHTML = "";
    document.documentElement.style.overflow = "";
  };

  // Bind overlay controls once.
  if (!overlay.dataset.bound) {
    overlay.dataset.bound = "1";
    const zoomAt = (factor) => {
      state.scale = Math.min(8, Math.max(0.2, state.scale * factor));
      apply();
    };
    overlay.querySelector('[data-mz="zoom-in"]').onclick = () => zoomAt(1.25);
    overlay.querySelector('[data-mz="zoom-out"]').onclick = () => zoomAt(0.8);
    overlay.querySelector('[data-mz="reset"]').onclick = reset;
    overlay.querySelector('[data-mz="close"]').onclick = close;

    stage.addEventListener("click", (e) => {
      if (e.target === stage) close(); // click backdrop
    });
    document.addEventListener("keydown", (e) => {
      if (!overlay.hidden && e.key === "Escape") close();
    });

    // Wheel zoom centred on cursor.
    stage.addEventListener(
      "wheel",
      (e) => {
        e.preventDefault();
        const rect = stage.getBoundingClientRect();
        const cx = e.clientX - rect.left - rect.width / 2;
        const cy = e.clientY - rect.top - rect.height / 2;
        const prev = state.scale;
        const next = Math.min(
          8,
          Math.max(0.2, prev * (e.deltaY < 0 ? 1.12 : 0.89))
        );
        const k = next / prev;
        state.x = cx - (cx - state.x) * k;
        state.y = cy - (cy - state.y) * k;
        state.scale = next;
        apply();
      },
      { passive: false }
    );

    // Drag to pan.
    stage.addEventListener("pointerdown", (e) => {
      state.dragging = true;
      state.sx = e.clientX - state.x;
      state.sy = e.clientY - state.y;
      stage.setPointerCapture(e.pointerId);
      stage.classList.add("mz-grabbing");
    });
    stage.addEventListener("pointermove", (e) => {
      if (!state.dragging) return;
      state.x = e.clientX - state.sx;
      state.y = e.clientY - state.sy;
      apply();
    });
    const endDrag = () => {
      state.dragging = false;
      stage.classList.remove("mz-grabbing");
    };
    stage.addEventListener("pointerup", endDrag);
    stage.addEventListener("pointercancel", endDrag);

    overlay.__open = open;
  }
  overlay.__open = open;

  // Make each diagram clickable (idempotent).
  diagrams.forEach((svg) => {
    const host = svg.closest(".mermaid");
    if (!host || host.dataset.mzReady) return;
    host.dataset.mzReady = "1";
    host.classList.add("mz-zoomable");
    host.setAttribute("role", "button");
    host.setAttribute("tabindex", "0");
    host.setAttribute("aria-label", "点击放大图表");
    const trigger = () => overlay.__open(svg);
    host.addEventListener("click", trigger);
    host.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        trigger();
      }
    });
  });
}
