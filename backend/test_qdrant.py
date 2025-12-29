import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# .env file se variables load karein
load_dotenv()

def check_qdrant_connection():
    # Environment variables se data nikaalein
    endpoint = os.getenv("QDRANT_ENDPOINT")
    api_key = os.getenv("QDRANT_API_KEY")
    client_url = os.getenv("QDRANT_CLIENT_URL")

    print("Checking Qdrant configuration...")
    print(f"QDRANT_ENDPOINT: {endpoint}")
    print(f"QDRANT_API_KEY: {'***' if api_key else 'Not set'}")
    print(f"QDRANT_CLIENT_URL: {client_url}")

    # Determine which configuration to use
    # First try the method used by the application code (endpoint + api_key)
    if endpoint and api_key:
        # Using Qdrant Cloud with API key (matches application code)
        print(f"\nConnecting to Qdrant Cloud: {endpoint}...")
        try:
            client = QdrantClient(
                url=endpoint,
                api_key=api_key,
                timeout=10,
                check_compatibility=False  # Skip initial compatibility check like in app code
            )
        except Exception as e:
            print(f"\n[ERROR] Failed to initialize Qdrant client with endpoint and API key: {str(e)}")
            return False
    elif client_url:
        # Using local Qdrant instance
        print(f"\nConnecting to local Qdrant: {client_url}...")
        try:
            client = QdrantClient(
                url=client_url,
                timeout=10,
                check_compatibility=False  # Skip initial compatibility check
            )
        except Exception as e:
            print(f"\n[ERROR] Failed to initialize Qdrant client with QDRANT_CLIENT_URL: {str(e)}")
            return False
    else:
        print("\n[ERROR] No Qdrant configuration found!")
        print("Please set either QDRANT_CLIENT_URL for local instance or both QDRANT_ENDPOINT and QDRANT_API_KEY for cloud.")
        return False

    try:
        # Collections ki list mangwayein
        collections = client.get_collections()

        print("\n[SUCCESS] Connection Successful!")
        print(f"Total Collections found: {len(collections.collections)}")

        for col in collections.collections:
            print(f" - Collection Name: {col.name}")

        return True

    except Exception as e:
        print("\n[ERROR] Connection Failed!")
        print(f"Error: {str(e)}")
        return False

if __name__ == "__main__":
    check_qdrant_connection()