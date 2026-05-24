window.MathJax = {
  tex: {
    inlineMath: [["\\(", "\\)"], ["$", "$"]],
    displayMath: [["\\[", "\\]"], ["$$", "$$"]],
    processEscapes: true,
    processEnvironments: true
  },
  options: {
    // Ignore code/pre blocks; process everything else (needed for mkdocs-jupyter notebook pages
    // which don't emit the arithmatex class that normal markdown math gets).
    ignoreHtmlClass: "highlight|language-"
  }
};
document$.subscribe(() => { MathJax.typesetPromise(); });
