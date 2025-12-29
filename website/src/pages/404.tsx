import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './NotFound.css'; // Import custom styles

const NotFound = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout title="Page Not Found" description="The requested page could not be found">
      <div className="not-found-container">
        <div className="not-found-content">
          <div className="not-found-animation">
            <div className="robot-404">
              <div className="robot-head">
                <div className="robot-eyes">
                  <div className="robot-eye left-eye"></div>
                  <div className="robot-eye right-eye"></div>
                </div>
              </div>
              <div className="robot-body">
                <div className="robot-antenna"></div>
                <div className="robot-chest-light"></div>
              </div>
              <div className="robot-arms">
                <div className="robot-arm left-arm"></div>
                <div className="robot-arm right-arm"></div>
              </div>
              <div className="robot-legs">
                <div className="robot-leg left-leg"></div>
                <div className="robot-leg right-leg"></div>
              </div>
            </div>
          </div>

          <h1 className="not-found-title">404</h1>
          <p className="not-found-subtitle">Oops! Page Not Found</p>
          <p className="not-found-message">
            The page you're looking for seems to have wandered off into the robotics lab.
            Don't worry, our AI robots are searching for it!
          </p>

          <div className="not-found-actions">
            <Link to="/" className="not-found-button primary-button">
              Go to Homepage
            </Link>
            <Link to="/module/m0-w1-2-introduction-to-physical-ai" className="not-found-button secondary-button">
              Start Learning
            </Link>
            <Link to="/module/module-1/ros2-fundamentals" className="not-found-button tertiary-button">
              Browse Modules
            </Link>
          </div>

          <div className="not-found-footer">
            <p>Need help? <Link to="/signin">Sign In</Link> or <Link to="/signup">Sign Up</Link></p>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default NotFound;