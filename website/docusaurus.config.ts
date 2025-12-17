import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';



const config: Config = {
  // --- PROJECT METADATA ---
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence: From Simulation to Reality',
  favicon: 'img/favicon.ico',

  // --- DEPLOYMENT CONFIG ---
  url: 'http://localhost:3000',
  baseUrl: '/',

  organizationName: 'your-username',
  projectName: 'ai-robotics-textbook',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  // Naya 'markdown' block yahan add karen
  markdown: {
    hooks: {
      // Broken image warnings ko 'warn' par set kar diya
      onBrokenMarkdownImages: 'warn',
    },
  },

  presets: [
    [
      'classic',
      {
        docs: false,
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'default',
        path: '../docs',
        routeBasePath: 'module',
        sidebarPath: require.resolve('./sidebars.ts'),
        exclude: [
          '**/module-3/**/*.mdx',
          '**/module-4/**/*.mdx',
          '**/module-4/**/*.md',
          '**/tutorials/isaac-ros/**/*.mdx',
          '**/tutorials/isaac-sim/quickstart.md',
        ],
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
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
          label: 'Start Learning',
        },
        {
          label: 'SYSTEM ACTIVE',
          position: 'right',
          className: 'header-system-active',
          to: '#',
        },
        {
          href: 'https://github.com/your-username/ai-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
        {
          label: 'Login',
          position: 'right',
          to: '/auth',
        },
        {
          label: 'Get Started',
          position: 'right',
          className: 'header-get-started',
          to: '/module/module-1/ros2-fundamentals',
        },
      ],
    },


    footer: {
      style: 'dark',
      links: [
        {
          title: 'Core Modules',
          items: [
            { label: 'ROS 2 Fundamentals', to: '/module/module-1/ros2-fundamentals' },
            { label: 'Gazebo Physics Simulation', to: '/module/module-1/ros2-fundamentals' },
            { label: 'Unity Rendering & HRI', to: '/module/module-1/ros2-fundamentals' },
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
            { label: 'GitHub Repo', href: 'https://github.com/your-username/ai-robotics-textbook' },
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

// Yahan sirf ek hi default export hona chahiye
export default config;