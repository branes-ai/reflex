// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import rehypeRewrite from 'rehype-rewrite';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

// Deployment configuration.
//   For GitHub Pages: DEPLOY_TARGET=github-pages (sets the project base path).
//   For local dev / other hosts: no env vars needed.
const isGitHubPages = process.env.DEPLOY_TARGET === 'github-pages';
const site = isGitHubPages
  ? 'https://branes-ai.github.io'
  : (process.env.SITE_URL || 'http://localhost:4321');
const base = isGitHubPages ? '/reflex/' : '/';

// The Doxygen C++ API reference is generated into public/api/ and served at
// <base>api/. Starlight prepends `base` to internal sidebar links, so this must
// be base-relative (prefixing it here too would double it).
const apiHref = '/api/';

// Rehype plugin to rewrite absolute internal links with the base path so
// authored `/control/...` links resolve under /reflex/ on Pages. Protocol-
// relative `//host` links and already-prefixed links are left untouched.
const basePrefix = base.replace(/\/$/, ''); // '/reflex'
const rehypeBaseLinks = isGitHubPages ? [
  [
    rehypeRewrite,
    {
      rewrite: (node) => {
        if (node.type !== 'element') return;
        const attr = node.tagName === 'a' ? 'href' : node.tagName === 'img' ? 'src' : null;
        if (!attr || typeof node.properties?.[attr] !== 'string') return;
        const val = node.properties[attr];
        const isRootRelative = val.startsWith('/') && !val.startsWith('//');
        const alreadyPrefixed = val === basePrefix || val.startsWith(basePrefix + '/');
        if (isRootRelative && !alreadyPrefixed) {
          node.properties[attr] = basePrefix + val;
        }
      },
    },
  ],
] : [];

// https://astro.build/config
export default defineConfig({
  server: { host: '0.0.0.0' },
  site,
  base,
  trailingSlash: 'always',
  // LaTeX math: remark-math parses `$…$` / `$$…$$`, rehype-katex renders it at
  // build time (KaTeX CSS is loaded via Starlight `customCss` below).
  markdown: {
    remarkPlugins: [remarkMath],
    rehypePlugins: [...rehypeBaseLinks, rehypeKatex],
  },
  integrations: [
    starlight({
      title: 'Reflex',
      description: 'The involuntary nervous system of the Branes.AI platform — header-only C++20 control and safety components, from PID to non-linear MPC.',
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/branes-ai/reflex' },
      ],
      editLink: {
        baseUrl: 'https://github.com/branes-ai/reflex/edit/main/docs-site/',
      },
      sidebar: [
        {
          label: 'Getting Started',
          items: [
            { label: 'Introduction', slug: 'getting-started/introduction' },
            { label: 'Build & Test', slug: 'getting-started/build-and-test' },
            { label: 'Repository Layout', slug: 'getting-started/repository-layout' },
          ],
        },
        {
          label: 'Control',
          items: [
            { label: 'PID Controller', slug: 'control/pid' },
          ],
        },
        {
          label: 'Examples',
          items: [
            { label: 'Mixed-Precision Gyro Filter', slug: 'examples/gyro-lowpass' },
          ],
        },
        {
          label: 'Roadmap',
          items: [
            { label: 'Roadmap', slug: 'roadmap' },
          ],
        },
        {
          label: 'API Reference',
          items: [
            { label: 'C++ API (Doxygen)', link: apiHref, attrs: { target: '_blank' } },
          ],
        },
      ],
      customCss: ['katex/dist/katex.min.css', './src/styles/custom.css'],
    }),
  ],
});
