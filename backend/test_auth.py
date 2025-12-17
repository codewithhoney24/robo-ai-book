import asyncio
import httpx
import json
from typing import Optional

# Test script to check the authentication system
BASE_URL = "http://127.0.0.1:8000"


async def test_auth_endpoints():
    """Test the authentication endpoints"""
    async with httpx.AsyncClient(timeout=30.0) as client:
        print("Testing authentication endpoints...")

        # Test 1: Health check
        print("\n1. Testing health endpoint:")
        try:
            response = await client.get(f"{BASE_URL}/health")
            print(f"Health status: {response.status_code}")
            print(f"Health response: {response.json()}")
        except Exception as e:
            print(f"Health check failed: {e}")

        # Test 2: Register a new user
        print("\n2. Testing user registration:")
        try:
            register_data = {
                "name": "Test User",
                "email": "testuser@example.com",
                "password": "securepassword123",
                "software_background": "python_intermediate",
                "hardware_background": "rtx_laptop"
            }
            # The route seems to be doubled up - checking the actual route
            response = await client.post(f"{BASE_URL}/api/v1/auth/register", json=register_data)
            print(f"Registration status: {response.status_code}")
            if response.status_code == 200:
                print(f"Registration response: {response.json()}")
            elif response.status_code == 422:
                print(f"Registration validation error: {response.json()}")
            else:
                print(f"Registration failed: {response.text}")

            # If the first attempt fails, try the doubled route
            if response.status_code == 404:
                print("Trying alternative registration route...")
                response = await client.post(f"{BASE_URL}/api/v1/auth/register/register", json=register_data)
                print(f"Alternative registration status: {response.status_code}")
                if response.status_code == 200:
                    print(f"Registration response: {response.json()}")
                elif response.status_code == 422:
                    print(f"Registration validation error: {response.json()}")
                else:
                    print(f"Registration failed: {response.text}")
        except Exception as e:
            print(f"Registration test failed: {e}")

        # Test 3: Login with the user
        print("\n3. Testing user login:")
        try:
            # FastAPI Users login expects form data, not JSON
            login_data = {
                "username": "testuser@example.com",  # Note: 'username', not 'email'
                "password": "securepassword123"
            }
            response = await client.post(
                f"{BASE_URL}/api/v1/auth/login",
                data=login_data,
                headers={"Content-Type": "application/x-www-form-urlencoded"}
            )
            print(f"Login status: {response.status_code}")
            if response.status_code == 200:
                login_response = response.json()
                print(f"Login successful: {login_response}")
                access_token = login_response.get('access_token', '')
            elif response.status_code == 422:
                print(f"Login validation error: {response.json()}")
            else:
                print(f"Login failed: {response.text}")
        except Exception as e:
            print(f"Login test failed: {e}")

        # Test 4: Get user profile (using a placeholder token)
        print("\n4. Testing get user profile:")
        try:
            # We'll need a valid token for this to work
            headers = {"Authorization": "Bearer <valid_token_here>"}
            response = await client.get(f"{BASE_URL}/api/v1/auth/users/me", headers=headers)
            print(f"Get user profile status: {response.status_code}")
            if response.status_code == 200:
                print(f"Get user profile response: {response.json()}")
            else:
                print(f"Get user profile failed: {response.text}")
        except Exception as e:
            print(f"Get user profile test failed: {e}")

        # Test 5: Test OPTIONS request to users/me
        print("\n5. Testing OPTIONS request to users/me:")
        try:
            response = await client.options(f"{BASE_URL}/api/v1/auth/users/me")
            print(f"OPTIONS status: {response.status_code}")
            print(f"OPTIONS headers: {dict(response.headers)}")
        except Exception as e:
            print(f"OPTIONS request failed: {e}")


if __name__ == "__main__":
    asyncio.run(test_auth_endpoints())