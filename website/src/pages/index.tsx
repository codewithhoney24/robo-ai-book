import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures/index';
import Heading from '@theme/Heading';

import styles from './index.module.css';


function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={styles.container}>
        {/* Top Navbar Buttons */}
        <div className={styles.topButtons}>
          <Link to="/module/m0-w1-2-introduction-to-physical-ai" className={styles.navButton}>Start Learning</Link>
          <span className={styles.systemBadge}>SYSTEM ACTIVE</span>
          <Link to="/module/m0-w1-2-introduction-to-physical-ai" className={styles.navButton}>Get Started</Link>

        </div>

        {/* Hero Title */}
        <Heading as="h1" className={styles.heroTitle}>
          {siteConfig.title}
        </Heading>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>

        {/* Interactive AI Nodes */}
        <div className={styles.aiNodes}>
          <span className={styles.node}>ROS 2</span>
          <span className={styles.node}>NVIDIA Isaac</span>
          <span className={styles.node}>AI</span>
        </div>

        {/* Main CTA Buttons */}
        <div className={styles.buttons}>
          <Link to="/module/m0-w1-2-introduction-to-physical-ai" className={styles.primaryButton}>Start Learning</Link>
          <Link
  to={String(siteConfig.customFields?.githubUrl || '/')}
  className={styles.secondaryButton}
  target="_blank"
  rel="noopener noreferrer"
>

  View on GitHub
</Link>

        </div>

      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics">
      <HomepageHeader />

      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
