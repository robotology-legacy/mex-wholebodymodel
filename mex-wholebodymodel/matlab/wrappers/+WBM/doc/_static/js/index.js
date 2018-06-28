window.MathJax = {
  jax: ["input/TeX", "input/MathML", "input/AsciiMath", "output/HTML-CSS", "output/NativeMML"],
  extensions: ["tex2jax.js", "mml2jax.js", "MathMenu.js", "MathZoom.js"],
  TeX: {
    //extensions: ["AMSmath.js", "AMSsymbols.js", "autoload-all.js","[Contrib]/siunitx/unpacked/siunitx.js"]
    //extensions: ["AMSmath.js", "AMSsymbols.js", "autoload-all.js", "[Contrib]/siunitx.js"]
    extensions: ["AMSmath.js", "AMSsymbols.js", "autoload-all.js", "siunitx.js"] // for the last two ...
  },
  MathML: {
    extensions: ["mml3.js", "content-mathml.js"]
  },
  tex2jax: {
    inlineMath: [
      ['$', '$'],
      ["\\(", "\\)"]
    ],
    processEscapes: true
  },
  AuthorInit: function() {
     //MathJax.Ajax.config.path["Contrib"] = '//rawgit.com/burnpanck/MathJax-third-party-extensions/add-siunitx-tex-extension'; // URL for development
     //MathJax.Ajax.config.path["Contrib"] = '//rawgit.com/burnpanck/MathJax-siunitx/master'; // URL for development

     MathJax.Ajax.config.path["Contrib"] = 'https://cdn.rawgit.com/burnpanck/MathJax-siunitx/master'; // URL for production use
     //MathJax.Ajax.config.path["Contrib"] = 'file:///home/ganymed/Downloads/Software/MathJax/MathJax-siunitx'; // local
  }
};

(function(d, script) {
  script = d.createElement('script');
  script.type = 'text/javascript';
  script.async = true;
  script.onload = function() {
    // remote script has loaded
  };
  script.src = 'https://cdn.rawgit.com/mathjax/MathJax/2.7.2/MathJax.js?config=TeX-AMS-MML_HTMLorMML'; // URL for production use
  //script.src = 'https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-AMS-MML_HTMLorMML';
  d.getElementsByTagName('head')[0].appendChild(script);
}(document));
