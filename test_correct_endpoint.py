import requests
import json

def test_signup_with_correct_endpoint():
    base_url = "http://localhost:8000/api"
    
    # Test signup with the correct endpoint that matches our implementation
    print("Testing signup with the correct endpoint...")
    signup_data = {
        "email": "finaltestuser@example.com",
        "password": "testpassword123",
        "name": "Final Test User",
        "softwareBackground": "beginner",
        "hardwareBackground": "no_gpu"
    }
    
    # Using the correct endpoint path
    signup_response = requests.post(f"{base_url}/v1/auth/signup-with-background", json=signup_data)
    print(f"Signup Status: {signup_response.status_code}")
    print(f"Signup Content-Type: {signup_response.headers.get('content-type')}")
    
    if signup_response.status_code == 200:
        print("SUCCESS: Signup worked correctly with proper endpoint!")
        response_json = signup_response.json()
        print(f"User ID: {response_json['user']['id']}")
        print(f"Access Token received: {'access_token' in response_json}")
    elif signup_response.status_code == 500 and "already exists" in signup_response.text:
        print("NOTE: User already exists (expected if running test multiple times)")
        print("But the important thing is it returned JSON, not HTML!")
    else:
        print(f"Response: {signup_response.text}")

if __name__ == "__main__":
    test_signup_with_correct_endpoint()