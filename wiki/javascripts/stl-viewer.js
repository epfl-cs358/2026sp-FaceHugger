/**
 * STL viewer helper for the printing guide.
 *
 * Looks for `.stl-viewer-root` elements on the page. Inside each root it
 * expects:
 *   - a <model-viewer> with id="stl-viewer-mv"
 *   - buttons with data-model="<relative path to .glb>"
 *
 * Clicking a button swaps the model-viewer src and highlights the active item.
 */
document.addEventListener("DOMContentLoaded", () => {
  document.querySelectorAll(".stl-viewer-root").forEach((root) => {
    const mv = root.querySelector("model-viewer");
    const buttons = root.querySelectorAll("[data-model]");

    buttons.forEach((btn) => {
      btn.addEventListener("click", () => {
        buttons.forEach((b) => b.classList.remove("stl-active"));
        btn.classList.add("stl-active");
        mv.src = btn.dataset.model;
      });
    });

    // Activate first button by default
    if (buttons.length > 0) {
      buttons[0].click();
    }
  });
});
