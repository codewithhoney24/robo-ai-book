import React from 'react';
import MDXPage from '@theme-original/MDXPage';

type MDXPageType = React.ComponentType<any>;

const CustomMDXPage: MDXPageType = (props) => {
  return <MDXPage {...props} />;
};

export default CustomMDXPage;