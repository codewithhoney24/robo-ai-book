import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css'; // Use a dedicated CSS file


type FeatureItem = {
  number: string;
  icon: string;
  title: string;
  description: ReactNode;
  weeks: string;
  tech: string;
};

const FeatureList: FeatureItem[] = [
  {
    number: '01',
    icon: '🤖',
    title: 'The Robotic Nervous System',
    description: (
      <>
        Master ROS 2 - the middleware powering modern robots. Build nodes, topics, and services to create distributed robotic systems.
      </>
    ),
    weeks: 'Weeks 1-5',
    tech: 'Python • rclpy • URDF',
  },
  {
    number: '02',
    icon: '💻',
    title: 'The Digital Twin',
    description: (
      <>
        Simulate physics-accurate environments in Gazebo. Test robots in virtual worlds before deploying to expensive hardware.
      </>
    ),
    weeks: 'Weeks 6-7',
    tech: 'Gazebo • Unity • SDF',
  },
  {
    number: '03',
    icon: '🧠',
    title: 'The AI-Robot Brain',
    description: (
      <>
        Advanced perception with NVIDIA Isaac. Photorealistic simulation, synthetic data generation, and hardware-accelerated SLAM.
      </>
    ),
    weeks: 'Weeks 8-12',
    tech: 'Isaac Sim • Isaac ROS • Nav2',
  },
];

function Feature({number, icon, title, description, weeks, tech}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.card}>
        <div className={styles.cardNumber}>{number}</div>
        <div className={styles.cardIcon}>{icon}</div>
        <Heading as="h3" className={styles.cardTitle}>{title}</Heading>
        <p className={styles.cardDescription}>{description}</p>
        <p className={styles.cardWeeks}>{weeks}</p>
        <p className={styles.cardTech}>{tech}</p>
        <a href={`/docs/module-${number}`} className={styles.featureButton}>Learn More</a>

      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        {/* --- ADDED TITLE AND PARAGRAPH HERE --- */}
        <Heading as="h2" className="text--center">
          Our Robotics Curriculum
        </Heading>
        <p className="text--center">
          Human-Robot Interaction
        </p>
        {/* ------------------------------------- */}

        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}

        </div>
      </div>
    </section>
  );
}