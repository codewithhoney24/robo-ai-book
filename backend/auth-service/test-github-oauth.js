#!/usr/bin/env node

/**
 * Test script to verify GitHub OAuth functionality
 * This script checks if the GitHub OAuth configuration is properly set up
 */

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

// For ES modules, we need to resolve __dirname equivalent
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Read .env file manually since we can't use dotenv in this context
let envVars = {};
const envPath = path.join(__dirname, '.env');

if (fs.existsSync(envPath)) {
  const envContent = fs.readFileSync(envPath, 'utf8');
  envContent.split('\n').forEach(line => {
    if (line.trim() && !line.startsWith('#')) {
      const [key, ...value] = line.split('=');
      envVars[key.trim()] = value.join('=').trim();
    }
  });
}

// Check if environment variables are set
const githubClientId = envVars.GITHUB_CLIENT_ID;
const githubClientSecret = envVars.GITHUB_CLIENT_SECRET;

console.log('🔍 GitHub OAuth Configuration Test');

if (!githubClientId || !githubClientSecret) {
  console.log('❌ GitHub OAuth is not fully configured');
  console.log('   Missing environment variables:');
  if (!githubClientId) console.log('   - GITHUB_CLIENT_ID');
  if (!githubClientSecret) console.log('   - GITHUB_CLIENT_SECRET');
  console.log('\n💡 To set up GitHub OAuth:');
  console.log('   1. Go to https://github.com/settings/developers');
  console.log('   2. Register a new OAuth application');
  console.log('   3. Add the client ID and secret to your .env file');
  process.exit(1);
} else {
  console.log('✅ GitHub OAuth environment variables are set');
}

// Check if auth config file exists
const authConfigPath = path.join(__dirname, '../auth/auth.config.ts');
if (fs.existsSync(authConfigPath)) {
  const authConfigContent = fs.readFileSync(authConfigPath, 'utf8');

  if (authConfigContent.includes('github') && authConfigContent.includes('GITHUB_CLIENT_ID')) {
    console.log('✅ GitHub OAuth is properly configured in auth.config.ts');
  } else {
    console.log('❌ GitHub OAuth is not properly configured in auth.config.ts');
    process.exit(1);
  }
} else {
  console.log('❌ auth.config.ts file not found');
  process.exit(1);
}

// Check if website API endpoints include GitHub endpoint
const apiEndpointsPath = path.join(__dirname, '../../website/src/config/api.ts');
if (fs.existsSync(apiEndpointsPath)) {
  const apiEndpointsContent = fs.readFileSync(apiEndpointsPath, 'utf8');

  if (apiEndpointsContent.includes('AUTH_GITHUB')) {
    console.log('✅ GitHub OAuth endpoint is properly configured in frontend');
  } else {
    console.log('❌ GitHub OAuth endpoint is not configured in frontend');
    process.exit(1);
  }
} else {
  console.log('❌ API endpoints file not found');
  process.exit(1);
}

// Check if auth page includes GitHub button
const authPagePath = path.join(__dirname, '../../website/src/pages/auth.tsx');
if (fs.existsSync(authPagePath)) {
  const authPageContent = fs.readFileSync(authPagePath, 'utf8');

  if (authPageContent.includes('handleGitHubSignIn') && authPageContent.includes('GitHub')) {
    console.log('✅ GitHub OAuth button is properly implemented in frontend');
  } else {
    console.log('❌ GitHub OAuth button is not implemented in frontend');
    process.exit(1);
  }
} else {
  console.log('❌ Auth page file not found');
  process.exit(1);
}

console.log('\n🎉 GitHub OAuth configuration is complete!');
console.log('\nTo test the functionality:');
console.log('1. Make sure your backend auth service is running');
console.log('2. Visit your auth page and click the "Sign in with GitHub" button');
console.log('3. Complete the GitHub OAuth flow');