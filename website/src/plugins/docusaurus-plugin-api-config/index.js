// website/src/plugins/docusaurus-plugin-api-config/index.js

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-api-config',
    
    getClientModules() {
      return [require.resolve('./apiConfig')];
    },
  };
};