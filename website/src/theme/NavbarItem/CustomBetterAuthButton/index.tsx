import React, { JSX } from 'react';
import type { Props } from '@theme/NavbarItem/DefaultNavbarItem'; // Docusaurus Props
import Link from '@docusaurus/Link';

// Props ko define karein, bhale hi hum inhein istemaal na karein
type CustomBetterAuthButtonProps = Props & {
  label: string;
  position: 'left' | 'right';
  type: 'custom-better-auth-button';
};

// Component ko props ke saath define karein
export default function CustomBetterAuthButton(props: CustomBetterAuthButtonProps): JSX.Element {
  return (
    <Link
      className="navbar__item button button--secondary button--sm" // Docusaurus navbar style
      to="/signin-signup-page" // Sahi page par link karein
      style={{
        fontWeight: 'bold',
        marginLeft: '8px',
        border: '1px solid #00ffeb',
        color: '#00ffeb', 
        backgroundColor: 'transparent',
      }}
    >
      Sign In/Up
    </Link>
  );
}