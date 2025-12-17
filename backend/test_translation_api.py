import requests
import json

# Test the translation endpoint with a simple request
def test_translation_endpoint():
    url = "http://localhost:8000/api/v1/translate"
    
    # Test data with both text and selected_text
    test_data = {
        "text": "This is the main text that will be ignored if selected_text is present.",
        "selected_text": "This is the selected text to translate to Urdu."
    }
    
    print("Testing translation endpoint with both 'text' and 'selected_text' provided...")
    print(f"Request data: {json.dumps(test_data, indent=2)}")
    
    try:
        response = requests.post(url, json=test_data, headers={"Content-Type": "application/json"})
        
        print(f"Response status code: {response.status_code}")
        print(f"Response headers: {dict(response.headers)}")
        
        if response.status_code == 200:
            response_data = response.json()
            print(f"Response data: {json.dumps(response_data, indent=2)}")
        else:
            print(f"Response content: {response.text}")
            
    except requests.exceptions.ConnectionError:
        print("Connection error: Could not connect to the server.")
        print("Make sure the FastAPI server is running on http://localhost:8000")
    
    print("\n" + "="*50 + "\n")
    
    # Test data with only text
    test_data2 = {
        "text": "This is a test for the translation API."
    }
    
    print("Testing translation endpoint with only 'text' provided...")
    print(f"Request data: {json.dumps(test_data2, indent=2)}")
    
    try:
        response = requests.post(url, json=test_data2, headers={"Content-Type": "application/json"})
        
        print(f"Response status code: {response.status_code}")
        
        if response.status_code == 200:
            response_data = response.json()
            print(f"Response data: {json.dumps(response_data, indent=2)}")
        else:
            print(f"Response content: {response.text}")
            
    except requests.exceptions.ConnectionError:
        print("Connection error: Could not connect to the server.")
        print("Make sure the FastAPI server is running on http://localhost:8000")

if __name__ == "__main__":
    test_translation_endpoint()