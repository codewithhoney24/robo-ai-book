import requests
import json

def test_all_auth_endpoints():
    base_url = "http://localhost:8000/api"

    # Test signup
    print("1. Testing signup endpoint...")
    signup_data = {
        "email": "finaltest@example.com",
        "password": "testpassword123",
        "name": "Final Test User",
        "softwareBackground": "beginner",
        "hardwareBackground": "no_gpu"
    }

    signup_response = requests.post(f"{base_url}/v1/auth/signup-with-background", json=signup_data)
    print(f"   Signup Status: {signup_response.status_code}")
    print(f"   Signup Content-Type: {signup_response.headers.get('content-type')}")

    if signup_response.status_code == 200:
        print("   SUCCESS: Signup successful")
        token = signup_response.json().get('access_token')

        # Test session (get user info with token)
        print("\n2. Testing session endpoint...")
        headers = {"Authorization": f"Bearer {token}"}
        session_response = requests.get(f"{base_url}/v1/auth/session", headers=headers)
        print(f"   Session Status: {session_response.status_code}")
        print(f"   Session Content-Type: {session_response.headers.get('content-type')}")

        if session_response.status_code == 200:
            print("   SUCCESS: Session endpoint working")
        else:
            print("   ERROR: Session endpoint failed")

        # Test signout
        print("\n3. Testing signout endpoint...")
        signout_response = requests.delete(f"{base_url}/v1/auth/session", headers=headers)
        print(f"   Signout Status: {signout_response.status_code}")
        print(f"   Signout Content-Type: {signout_response.headers.get('content-type')}")

        if signout_response.status_code == 200:
            print("   SUCCESS: Signout successful")
        else:
            print("   ERROR: Signout failed")
    elif signup_response.status_code == 500 and "already exists" in signup_response.text:
        print("   NOTE: User already exists (expected if running test multiple times)")
    else:
        print(f"   ERROR: Signup failed with status {signup_response.status_code}")
        print(f"   Response: {signup_response.text}")

if __name__ == "__main__":
    test_all_auth_endpoints()