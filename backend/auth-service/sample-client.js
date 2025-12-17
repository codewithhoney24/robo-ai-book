// Sample client code to demonstrate proper API calls to the auth service

// Signup function
async function signupUser({
  email,
  password,
  name,
  programmingLevel = "beginner",
  knownLanguages = "",
  aiMlExperience = false,
  rosRoboticsExperience = false,
  hasJetson = false,
  hasRTXGPU = false,
  hasRobotHardware = false,
  simulationOnly = true
}) {
  try {
    const response = await fetch('http://localhost:8000/api/auth/sign-up', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email,
        password,
        name,
        programmingLevel,
        knownLanguages,
        aiMlExperience,
        rosRoboticsExperience,
        hasJetson,
        hasRTXGPU,
        hasRobotHardware,
        simulationOnly
      }),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || 'Signup failed');
    }

    console.log('Signup successful:', data);
    return data;
  } catch (error) {
    console.error('Signup error:', error.message);
    throw error;
  }
}

// Signin function
async function signinUser(email, password) {
  try {
    const response = await fetch('http://localhost:8000/api/auth/sign-in', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email,
        password
      }),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || 'Signin failed');
    }

    console.log('Signin successful:', data);
    return data;
  } catch (error) {
    console.error('Signin error:', error.message);
    throw error;
  }
}

// Example usage:
// signupUser({
//   email: 'test@example.com',
//   password: 'password123',
//   name: 'Test User',
//   programmingLevel: 'beginner',
//   knownLanguages: 'Python, JavaScript',
//   aiMlExperience: false,
//   rosRoboticsExperience: false,
//   hasJetson: false,
//   hasRTXGPU: false,
//   hasRobotHardware: false,
//   simulationOnly: true
// });

export { signupUser, signinUser };