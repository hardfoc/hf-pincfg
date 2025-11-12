# Doxygen Extensions

This directory contains all Doxygen-specific customizations and extensions for the HF-PinCfg documentation.

## 📁 Contents

```
doxygen-extensions/
├── README.md                           # This file
├── header.html                         # Custom Doxygen header template
├── footer.html                         # Custom Doxygen footer template
├── custom-styles.css                   # Custom CSS for HF-PinCfg branding
└── doxygen-awesome-css/                # Git submodule with modern theme
    ├── doxygen-awesome.css
    ├── doxygen-awesome-sidebar-only.css
    └── *.js files for interactive features
```

## 🎨 Custom Components

### Header (`header.html`)
- Branded header with HF-PinCfg styling
- Gradient background and feature badges
- Responsive design with dark mode support

### Footer (`footer.html`)
- Professional footer with project links
- Copyright information and generation details
- Responsive layout

### Custom Styles (`custom-styles.css`)
- HF-PinCfg specific branding
- Dark mode compatibility
- Mobile-first responsive design
- Integration with doxygen-awesome-css theme

## 🚀 Theme Integration

The doxygen-awesome-css submodule provides:
- Modern, professional appearance
- Interactive table of contents
- Code copy buttons
- Dark/light mode toggle
- Responsive sidebar navigation
- Tab interface support

## ⚙️ Doxyfile Configuration

These files are referenced in the main Doxyfile:

```ini
HTML_HEADER = _config/doxygen-extensions/header.html
HTML_FOOTER = _config/doxygen-extensions/footer.html

HTML_EXTRA_STYLESHEET = _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome.css \
                       _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome-sidebar-only.css \
                       _config/doxygen-extensions/custom-styles.css

HTML_EXTRA_FILES = _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome-darkmode-toggle.js \
                  _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome-fragment-copy-button.js \
                  _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome-interactive-toc.js \
                  _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome-paragraph-link.js \
                  _config/doxygen-extensions/doxygen-awesome-css/doxygen-awesome-tabs.js
```

## 🔄 Submodule Management

To update the doxygen-awesome-css theme:
```bash
git submodule update --remote _config/doxygen-extensions/doxygen-awesome-css
```

## 🎯 Organization

This directory keeps all Doxygen-specific files separate from Jekyll configuration:
- **Jekyll files**: `_config/_config.yml`, `_config/_layouts/`, `_config/_includes/`
- **Doxygen files**: `_config/doxygen-extensions/` (this directory)

This separation ensures clean organization and prevents confusion between the two documentation systems.