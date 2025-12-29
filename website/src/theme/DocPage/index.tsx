import React from 'react';
import DocPage from '@theme-original/DocPage';

type DocPageType = React.ComponentType<any>;

const CustomDocPage: DocPageType = (props) => {
  return <DocPage {...props} />;
};

export default CustomDocPage;