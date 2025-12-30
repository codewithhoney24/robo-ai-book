import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import path from 'path';

const config: Config = {
  // --- PROJECT METADATA ---
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence: From Simulation to Reality',
  favicon: 'img/favicon.ico',

  // --- DEPLOYMENT CONFIG ---
  url: 'http://localhost:3000',
  baseUrl: '/',

  organizationName: 'codewithhoney24',
  projectName: 'robo-ai-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          path: 'docs',
          routeBasePath: 'module',
          sidebarPath: require.resolve('./sidebars.ts'),
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    // Define custom fields to pass environment variables to the client
    customFields: {
      // These values will be available in the client-side code
      REACT_APP_API_BASE_URL: process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api',
      onBrokenMarkdownImages: 'warn',
    },

    navbar: {
      title: 'PHYSICAL AI ROBOTICS',
      logo: {
        alt: 'Robo AI Logo',
        src: 'img/robot.jpg',
      },
      items: [
        {
          type: 'doc',
          docId: 'module-1/ros2-fundamentals',
          position: 'left',
          label: 'Module 1: ROS 2',
        },
        {
          href: 'https://github.com/codewithhoney24/robo-ai-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            { label: 'Module 1: ROS 2 Fundamentals', to: '/module/module-1/ros2-fundamentals' },
          ],
        },
        {
          title: 'Project Goals',
          items: [
            { label: 'Embodied Intelligence', href: 'https://panaversity.org' },
            { label: 'Start Learning', to: '/module/module-1/ros2-fundamentals' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub Repo', href: 'https://github.com/codewithhoney24/robo-ai-book' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;