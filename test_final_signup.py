import requests
import json

def test_final_signup():
    base_url = "http://localhost:8000/api"
    
    # Test signup with a completely new email
    print("Testing signup endpoint with a new user...")
    signup_data = {
        "email": "completelynewuser@example.com",
        "password": "testpassword123",
        "name": "Completely New User",
        "softwareBackground": "beginner",
        "hardwareBackground": "no_gpu"
    }
    
    signup_response = requests.post(f"{base_url}/v1/auth/signup-with-background", json=signup_data)
    print(f"Signup Status: {signup_response.status_code}")
    print(f"Signup Content-Type: {signup_response.headers.get('content-type')}")
    
    if signup_response.status_code == 200:
        print("SUCCESS: New user signup worked correctly!")
        response_json = signup_response.json()
        print(f"User ID: {response_json['user']['id']}")
        print(f"Access Token received: {'access_token' in response_json}")
        print("The signup endpoint is now working correctly and returning valid JSON!")
    else:
        print(f"Response: {signup_response.text}")

if __name__ == "__main__":
    test_final_signup()