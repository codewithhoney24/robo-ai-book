import requests
import json

# Test the signup endpoint with an existing user to check error handling
def test_signup_duplicate_user():
    # Define the API endpoint
    base_url = "http://localhost:8000/api"
    signup_url = f"{base_url}/v1/auth/signup-with-background"

    # Test data with the same email as the previous test
    test_data = {
        "email": "testuser2@example.com",  # This user was created in the previous test
        "password": "testpassword123",
        "name": "Test User",
        "softwareBackground": "beginner",
        "hardwareBackground": "no_gpu"
    }

    print("Testing signup endpoint with duplicate email...")
    print(f"URL: {signup_url}")
    print(f"Data: {json.dumps(test_data, indent=2)}")

    try:
        response = requests.post(signup_url, json=test_data)

        print(f"Response Status Code: {response.status_code}")
        print(f"Response Headers: {dict(response.headers)}")

        # Check if response is JSON
        content_type = response.headers.get('content-type', '')
        if 'application/json' in content_type:
            print(f"Response JSON: {json.dumps(response.json(), indent=2)}")
            print("SUCCESS: Error response returned valid JSON")
        else:
            print(f"Response Text: {response.text[:500]}...")  # First 500 chars
            print("ERROR: Error response did not return JSON")

    except requests.exceptions.ConnectionError:
        print("ERROR: Could not connect to the server. Make sure the backend is running on http://localhost:8000")
    except Exception as e:
        print(f"ERROR: {str(e)}")

if __name__ == "__main__":
    test_signup_duplicate_user()