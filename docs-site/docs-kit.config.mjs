// The docs site's configuration: everything docs-kit does not default.
// https://github.com/branes-ai/docs-kit
export default {
  repo: 'reflex',
  title: 'Reflex',
  description: 'The involuntary nervous system of the Branes.AI platform — header-only C++20 control and safety components, from PID to non-linear MPC.',
  groupLabels: { dsp: 'Signal Processing' },
  // The session logs are working records, not documentation.
  unpublished: ['sessions'],
  accent: {
    dark: { low: '#3a2410', accent: '#f08a24', high: '#ffd3a3' },
    light: { low: '#fde3c6', accent: '#b85c0a', high: '#4a2505' },
  },
  // LaTeX math (`$…$`, `$$…$$`), rendered with KaTeX at build time.
  math: true,
  customCss: ['./src/styles/custom.css'],
  api: {
    name: 'Reflex C++ API',
    brief: 'branes::reflex — header-only control and safety components',
    input: ['sdk/include'],
  },
};
