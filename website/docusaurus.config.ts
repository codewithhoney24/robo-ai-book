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
  markdown: {
    format: 'detect',
    mermaid: false,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
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
    './src/plugins/docusaurus-plugin-api-config', // Custom plugin to expose API config
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'default',
        path: 'docs', // Changed from '../docs' to 'docs' to look in the local directory
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

    // Define custom fields to pass environment variables to the client
    customFields: {
      // These values will be available in the client-side code
      REACT_APP_API_BASE_URL: process.env.REACT_APP_API_BASE_URL || 'https://web-production-3ced.up.railway.app/api/v1',
      onBrokenMarkdownImages: 'warn',
    },

    // MDX components configuration
    mdx: {
      components: {
        path: './src/theme/MDXComponents',
        global: true,
      },
    },

    navbar: {
      title: 'PHYSICAL AI ROBOTICS',
      logo: {
        alt: 'Robo AI Logo',
        src: 'img/robot.jpg',
      },
      items: [
        {
          label: 'Start Learning',
          position: 'left',
          className: 'header-start-learning',
          to: '/module/module-1/ros2-fundamentals',
        },
        //  {
        //   label: 'SYSTEM ACTIVE',
        //   position: 'right',
        //   className: 'header-system-active',
        //   to: '#',
        // },
        {
          href: 'https://github.com/codewithhoney24/robo-ai-book',
          label: 'GitHub',
          position: 'right',
        },
        // {
        //   label: 'Start Learning',
        //   position: 'right',
        //   className: 'header-start-learning',
        //   to: '/module/module-1/ros2-fundamentals',
        // },
        {
          href: '/signup',
          label: 'Sign Up',
          position: 'right',
        },
        {
          href: '/signin',
          label: 'Sign In',
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
            { label: 'Module 2: Gazebo Physics Simulation', to: '/module/m2-w6-gazebo-physics-simulation' },
            { label: 'Module 2: Unity Rendering & HRI', to: '/module/m2-w7-sensor-simulation-unity' },
            { label: 'Module 3: Isaac Sim & ROS', to: '/module/m3-w8-isaac-sim-synthetic-data' },
            { label: 'Module 3: Isaac ROS Navigation', to: '/module/m3-w9-isaac-ros-vslam-nav2' },
          ],
        },
        {
          title: 'Modules (continued)',
          items: [
            { label: 'Module 3: RL Sim-to-Real', to: '/module/m3-w10-rl-sim-to-real' },
            { label: 'Module 4: Humanoid Kinematics', to: '/module/m4-w11-humanoid-kinematics-balance' },
            { label: 'Module 4: Manipulation & HRI', to: '/module/m4-w12-manipulation-hri-design' },
            { label: 'Module 4: Conversational Robotics', to: '/module/m4-w13a-conversational-robotics' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub Repo', href: 'https://github.com/codewithhoney24/robo-ai-book' },
            { label: 'Embodied Intelligence', href: 'https://panaversity.org' },
            { label: 'Start Learning', to: '/module/module-1/ros2-fundamentals' },
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