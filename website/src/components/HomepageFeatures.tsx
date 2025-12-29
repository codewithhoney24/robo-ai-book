import React, { JSX } from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';
import { usePersonalization } from '../contexts/PersonalizationContext';

type FeatureItem = {
  title: string;
  description: JSX.Element;
};

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  const { personalizationData } = usePersonalization();
  const { preferences } = personalizationData;

  // Define feature list based on personalization preferences
  let FeatureList: FeatureItem[] = [];


  if (preferences.difficulty !== 'beginner') {
    FeatureList.push({
      title: 'Advanced Content',
      description: (
        <>
          Access to more complex concepts and advanced materials based on your
          <strong> {preferences.difficulty} </strong> level.
        </>
      ),
    });
  }

  if (preferences.learningStyle === 'textual') {
    FeatureList.push({
      title: 'Textual Learning',
      description: (
        <>
          Detailed explanations, written examples, and text-based content to support
          your <strong>textual</strong> learning preference.
        </>
      ),
    });
  } else if (preferences.learningStyle === 'handsOn') {
    FeatureList.push({
      title: 'Hands-On Learning',
      description: (
        <>
          Practical exercises, coding challenges, and interactive simulations to support
          your <strong>hands-on</strong> learning preference.
        </>
      ),
    });
  }

  if (preferences.contentAccess === 'realHardware') {
    FeatureList.push({
      title: 'Real Hardware Access',
      description: (
        <>
          Content and examples focused on <strong>real hardware</strong> implementation
          rather than just simulation.
        </>
      ),
    });
  }


  // If no preferences are set or no personalized features apply, show default features
  if (FeatureList.length === 0) {
    FeatureList = [
      {
        title: 'Easy to Use',
        description: (
          <>
            Docusaurus was designed from the ground up to be easily installed and
            used to get your website up and running quickly.
          </>
        ),
      },
      {
        title: 'Focus on What Matters',
        description: (
          <>
            Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go
            ahead and move your docs into the <code>docs</code> directory.
          </>
        ),
      },
      {
        title: 'Powered by React',
        description: (
          <>
            Extend or customize your website layout by reusing React. Docusaurus can
            be extended while reusing the same header and footer.
          </>
        ),
      },
    ];
  }

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}