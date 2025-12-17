import React, { ReactNode } from 'react';

interface LayoutWrapperProps {
  children: ReactNode; // ✅ Explicit type for children
}

const LayoutWrapper: React.FC<LayoutWrapperProps> = ({ children }) => {
  return <>{children}</>;
};

export default LayoutWrapper;
