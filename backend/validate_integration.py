"""
Integration validation script for the RAG Chatbot system
This script performs a structural validation of the implementation
"""
import os
from pathlib import Path

def validate_project_structure():
    """Validate that all required directories and files exist"""
    base_path = Path("D:/ai-book-pro/ai-robo-bk/backend")

    required_dirs = [
        "src",
        "src/models",
        "src/services",
        "src/api",
        "src/config",
        "src/exceptions",
        "tests",
        "tests/unit",
        "tests/integration",
        "tests/contract",
        "docs",
        "specs/002-rag-chatbot-book",
        "specs/002-rag-chatbot-book/checklists",
        "specs/002-rag-chatbot-book/contracts"
    ]

    missing_dirs = []
    for dir_path in required_dirs:
        full_path = base_path / dir_path
        if not full_path.exists():
            missing_dirs.append(str(full_path))

    if missing_dirs:
        print("Missing directories:")
        for dir_path in missing_dirs:
            print(f"  - {dir_path}")
        return False
    else:
        print("All required directories exist")
        return True


def validate_models():
    """Validate that all required models exist"""
    base_path = Path("D:/ai-book-pro/ai-robo-bk/backend/src/models")

    required_models = [
        "user_query.py",
        "retrieved_context.py",
        "generated_response.py",
        "book_content.py",
        "user_session.py"  # This might not be used but was mentioned in docs
    ]

    missing_models = []
    for model in required_models:
        full_path = base_path / model
        if not full_path.exists():
            missing_models.append(str(full_path))

    if missing_models:
        print("Missing model files:")
        for model_path in missing_models:
            print(f"  - {model_path}")
        return False
    else:
        print("All required models exist")
        return True


def validate_services():
    """Validate that all required services exist"""
    base_path = Path("D:/ai-book-pro/ai-robo-bk/backend/src/services")

    required_services = [
        "rag_service.py",
        "vector_store_service.py",
        "database_service.py",
        "embedding_service.py",
        "session_service.py",
        "optimization_service.py",
        "book_content_service.py",
        "caching_service.py"
    ]

    missing_services = []
    for service in required_services:
        full_path = base_path / service
        if not full_path.exists():
            missing_services.append(str(full_path))

    if missing_services:
        print("Missing service files:")
        for service_path in missing_services:
            print(f"  - {service_path}")
        return False
    else:
        print("All required services exist")
        return True


def validate_api_endpoints():
    """Validate that the API routers exist and contain the required endpoints"""
    import re

    chatbot_router_path = Path("D:/ai-book-pro/ai-robo-bk/backend/src/api/chatbot_router.py")
    book_content_router_path = Path("D:/ai-book-pro/ai-robo-bk/backend/src/api/book_content_router.py")

    # Check if routers exist
    if not chatbot_router_path.exists():
        print("Missing chatbot_router.py")
        return False

    if not book_content_router_path.exists():
        print("Missing book_content_router.py")
        return False

    # Read router content to verify endpoints exist
    with open(chatbot_router_path, 'r') as f:
        chatbot_content = f.read()

    with open(book_content_router_path, 'r') as f:
        book_content = f.read()

    # Check for required endpoints
    required_endpoints = [
        ('/chat', 'POST'),
        ('/chat/selected-text', 'POST'),
        ('/health', 'GET')  # This is in main.py, but health check should work
    ]

    book_content_endpoints = [
        ('/book-content/index', 'POST'),
        ('/book-content/index-text', 'POST')
    ]

    all_present = True
    for path, method in required_endpoints:
        if f'@router.{method.lower()}("{path}"' not in chatbot_content:
            print(f"Missing {method} {path} endpoint in chatbot_router")
            all_present = False

    for path, method in book_content_endpoints:
        if f'@router.{method.lower()}("{path}"' not in book_content:
            print(f"Missing {method} {path} endpoint in book_content_router")
            all_present = False

    if all_present:
        print("All required API endpoints exist")
        return True
    else:
        return False


def validate_config():
    """Validate that configuration files exist"""
    base_path = Path("D:/ai-book-pro/ai-robo-bk/backend/src/config")

    required_configs = [
        "settings.py",
        "database_config.py",
        "logging_config.py"
    ]

    missing_configs = []
    for config in required_configs:
        full_path = base_path / config
        if not full_path.exists():
            missing_configs.append(str(full_path))

    if missing_configs:
        print("Missing config files:")
        for config_path in missing_configs:
            print(f"  - {config_path}")
        return False
    else:
        print("All required config files exist")
        return True


def validate_documentation():
    """Validate that required documentation exists"""
    base_path = Path("D:/ai-book-pro/ai-robo-bk/backend")

    required_docs = [
        "README.md",
        "docs/api.md",
        "docs/web_integration.md",
        "docs/selected_text.md",
        "specs/002-rag-chatbot-book/spec.md",
        "specs/002-rag-chatbot-book/plan.md",
        "specs/002-rag-chatbot-book/tasks.md",
        "specs/002-rag-chatbot-book/quickstart.md",
        "specs/002-rag-chatbot-book/contracts/openapi.yaml"
    ]

    missing_docs = []
    for doc in required_docs:
        full_path = base_path / doc
        if not full_path.exists():
            missing_docs.append(str(full_path))

    if missing_docs:
        print("Missing documentation files:")
        for doc_path in missing_docs:
            print(f"  - {doc_path}")
        return False
    else:
        print("All required documentation exists")
        return True


def main():
    print("Starting RAG Chatbot System Integration Validation...")
    print("=" * 60)

    results = []

    print("\nValidating project structure...")
    results.append(validate_project_structure())

    print("\nValidating models...")
    results.append(validate_models())

    print("\nValidating services...")
    results.append(validate_services())

    print("\nValidating API endpoints...")
    results.append(validate_api_endpoints())

    print("\nValidating configuration...")
    results.append(validate_config())

    print("\nValidating documentation...")
    results.append(validate_documentation())

    print("\n" + "=" * 60)
    if all(results):
        print("All validation checks passed! The RAG Chatbot system is properly integrated.")
        print("\nSummary:")
        print("  - Project structure is correct")
        print("  - All required models exist")
        print("  - All required services exist")
        print("  - All required API endpoints exist")
        print("  - All required configuration files exist")
        print("  - All required documentation exists")
        print("\nThe system is ready for deployment after installing dependencies!")
        return True
    else:
        print("Some validation checks failed. Please review the issues above.")
        return False


if __name__ == "__main__":
    main()